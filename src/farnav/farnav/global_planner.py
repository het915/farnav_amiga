"""
Reads lat/lon waypoints (A, intermediates, B) from YAML config,
converts to UTM, transforms into the odom frame using GPS+odom alignment,
fits a cubic spline, parameterizes by arc-length theta,
and publishes the path for the MPC controller.
"""

import json
import numpy as np
from pyproj import Transformer
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from std_srvs.srv import Trigger


def _yaw_from_quat(q):
    """Extract yaw from geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


class GlobalPlanner(Node):

    def __init__(self):
        super().__init__('global_planner')

        # --- Declare parameters ---
        self.declare_parameter('utm_epsg', 'EPSG:32616')
        self.declare_parameter('spline_resolution', 500)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('path_frame_id', 'odom')
        self.declare_parameter('active_row', 0)

        self.utm_epsg = self.get_parameter('utm_epsg').get_parameter_value().string_value
        self.spline_resolution = self.get_parameter('spline_resolution').get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.frame_id = self.get_parameter('path_frame_id').get_parameter_value().string_value
        self.active_row_idx = self.get_parameter('active_row').get_parameter_value().integer_value

        # --- Parse rows from parameters ---
        self.rows = self._parse_rows()
        self.get_logger().info(f'Loaded {len(self.rows)} rows from config')

        # --- Coordinate transformer: WGS84 (lat/lon) -> UTM ---
        self.transformer = Transformer.from_crs('EPSG:4326', self.utm_epsg, always_xy=False)

        # --- Publishers ---
        self.path_pub = self.create_publisher(Path, '/farnav/global_path', 10)
        self.info_pub = self.create_publisher(String, '/farnav/path_info', 10)
        self.marker_pub = self.create_publisher(Marker, '/farnav/active_row_marker', 10)

        # --- Service ---
        self.set_row_srv = self.create_service(
            Trigger, '/farnav/set_active_row', self.set_active_row_callback
        )

        # --- Alignment state ---
        # We need one GPS fix + one odom reading to compute the UTM->odom transform
        self.aligned = False
        self.odom_at_align = None   # (x, y, yaw) in odom frame
        self.gps_at_align = None    # (utm_e, utm_n) at alignment time
        self.utm_to_odom_offset = None  # (dx, dy)
        self.utm_to_odom_angle = None   # rotation angle

        self.spline_path = None
        self.arc_lengths = None

        # --- Subscribers for alignment ---
        self._odom_sub = self.create_subscription(
            Odometry, '/amiga/odom', self._odom_cb, 10)
        self._gps_sub = self.create_subscription(
            NavSatFix, '/amiga/gps', self._gps_cb, 10)

        self._latest_odom = None
        self._latest_gps = None

        # --- Timer for periodic publishing ---
        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self._publish_all)

        self.get_logger().info(
            f'Global Planner waiting for GPS + odom to align frames...'
        )

    # ------------------------------------------------------------------
    # Alignment callbacks
    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._latest_odom = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )
        self._try_align()

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            return  # no fix
        n, e = self.transformer.transform(msg.latitude, msg.longitude)
        self._latest_gps = (e, n)
        self._try_align()

    def _try_align(self):
        """Once we have both GPS and odom, compute the transform and build the path."""
        if self.aligned or self._latest_odom is None or self._latest_gps is None:
            return

        odom_x, odom_y, odom_yaw = self._latest_odom
        gps_e, gps_n = self._latest_gps

        self.odom_at_align = self._latest_odom
        self.gps_at_align = self._latest_gps

        # The robot's GPS position in UTM is (gps_e, gps_n)
        # The robot's odom position is (odom_x, odom_y) with heading odom_yaw
        # We need to transform UTM points so that (gps_e, gps_n) maps to (odom_x, odom_y)
        # and the UTM north direction is rotated to match odom heading

        # UTM heading of the path (north = +Y in UTM, east = +X)
        # In UTM, heading 0 = east. Robot odom_yaw is also from +X axis.
        # The GPS doesn't give heading, so we use the path direction at Point A
        # to determine the rotation.

        # For now: compute the UTM position of Point A
        row = self.rows[self.active_row_idx]
        lat_a, lon_a = row['A']
        n_a, e_a = self.transformer.transform(lat_a, lon_a)

        # Path heading at Point A (from A toward next waypoint or B)
        if row['waypoints']:
            lat_next, lon_next = row['waypoints'][0]
        else:
            lat_next, lon_next = row['B']
        n_next, e_next = self.transformer.transform(lat_next, lon_next)
        path_heading_utm = np.arctan2(n_next - n_a, e_next - e_a)

        # The rotation needed: from UTM heading to odom heading
        # At Point A, the path heading in UTM is path_heading_utm
        # The robot is currently near Point A, heading odom_yaw
        # So rotation = odom_yaw - path_heading_utm
        self.utm_to_odom_angle = odom_yaw - path_heading_utm

        self.get_logger().info(
            f'Alignment: odom_yaw={np.degrees(odom_yaw):.1f} deg, '
            f'path_heading_utm={np.degrees(path_heading_utm):.1f} deg, '
            f'rotation={np.degrees(self.utm_to_odom_angle):.1f} deg'
        )

        # Store UTM origin (Point A) and odom position for transform
        self._utm_origin = np.array([e_a, n_a])
        self._odom_origin = np.array([odom_x, odom_y])

        self.aligned = True
        self.get_logger().info('GPS/odom alignment complete, building path...')

        self._build_path()

        self.get_logger().info(
            f'Global Planner started — active row: {self.active_row_idx}, '
            f'UTM: {self.utm_epsg}, resolution: {self.spline_resolution}'
        )

    def _utm_to_odom(self, utm_points):
        """
        Transform UTM points to odom frame.
        1. Subtract UTM origin (Point A)
        2. Rotate by utm_to_odom_angle
        3. Add odom origin (robot position at alignment)
        """
        shifted = utm_points - self._utm_origin
        cos_a = np.cos(self.utm_to_odom_angle)
        sin_a = np.sin(self.utm_to_odom_angle)
        rotated = np.column_stack([
            cos_a * shifted[:, 0] - sin_a * shifted[:, 1],
            sin_a * shifted[:, 0] + cos_a * shifted[:, 1],
        ])
        return rotated + self._odom_origin

    # ------------------------------------------------------------------
    # Parameter parsing
    # ------------------------------------------------------------------
    def _parse_rows(self):
        """Parse row definitions from declared parameters."""
        rows = []
        row_idx = 1
        while True:
            prefix = f'rows.row_{row_idx}'
            self.declare_parameter(f'{prefix}.name', '')
            self.declare_parameter(f'{prefix}.A', [0.0, 0.0])
            self.declare_parameter(f'{prefix}.B', [0.0, 0.0])
            self.declare_parameter(f'{prefix}.waypoints', [0.0])

            name = self.get_parameter(f'{prefix}.name').get_parameter_value().string_value
            if not name:
                break

            a = self.get_parameter(f'{prefix}.A').get_parameter_value().double_array_value
            b = self.get_parameter(f'{prefix}.B').get_parameter_value().double_array_value
            wp_flat = self.get_parameter(f'{prefix}.waypoints').get_parameter_value().double_array_value

            # waypoints come as flat array [lat1, lon1, lat2, lon2, ...]
            waypoints = []
            if len(wp_flat) >= 2:
                for i in range(0, len(wp_flat) - 1, 2):
                    waypoints.append([wp_flat[i], wp_flat[i + 1]])

            rows.append({
                'name': name,
                'A': list(a),
                'waypoints': waypoints,
                'B': list(b),
            })
            row_idx += 1

        return rows

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------
    def _latlon_to_utm(self, lat, lon):
        """Convert lat/lon (WGS84) to UTM easting/northing."""
        northing, easting = self.transformer.transform(lat, lon)
        return easting, northing

    # ------------------------------------------------------------------
    # Spline path building
    # ------------------------------------------------------------------
    def _build_path(self):
        """Build cubic spline path for the active row."""
        if self.active_row_idx >= len(self.rows):
            self.get_logger().error(
                f'Active row index {self.active_row_idx} out of range (have {len(self.rows)} rows)'
            )
            return

        row = self.rows[self.active_row_idx]

        # Collect all points: A -> waypoints -> B
        all_latlon = [row['A']] + row['waypoints'] + [row['B']]

        # Convert to UTM
        utm_points = []
        for lat, lon in all_latlon:
            e, n = self._latlon_to_utm(lat, lon)
            utm_points.append([e, n])

        utm_points = np.array(utm_points)

        # Transform to odom frame
        odom_points = self._utm_to_odom(utm_points)

        self.get_logger().info(
            f'Row "{row["name"]}": {len(odom_points)} control points -> spline with {self.spline_resolution} samples'
        )
        self.get_logger().info(
            f'Path start (odom): ({odom_points[0, 0]:.2f}, {odom_points[0, 1]:.2f}), '
            f'end: ({odom_points[-1, 0]:.2f}, {odom_points[-1, 1]:.2f})'
        )

        # Parameterize by cumulative chord length
        diffs = np.diff(odom_points, axis=0)
        chord_lengths = np.sqrt((diffs ** 2).sum(axis=1))
        t = np.concatenate([[0.0], np.cumsum(chord_lengths)])
        t_normalized = t / t[-1]  # normalize to [0, 1]

        # Fit cubic splines for x(t) and y(t)
        cs_x = CubicSpline(t_normalized, odom_points[:, 0], bc_type='clamped')
        cs_y = CubicSpline(t_normalized, odom_points[:, 1], bc_type='clamped')

        # Sample the spline at uniform parameter values
        t_sample = np.linspace(0.0, 1.0, self.spline_resolution)
        x_sample = cs_x(t_sample)
        y_sample = cs_y(t_sample)

        # Compute arc-length theta for each sample point
        dx = np.diff(x_sample)
        dy = np.diff(y_sample)
        ds = np.sqrt(dx ** 2 + dy ** 2)
        arc_lengths = np.concatenate([[0.0], np.cumsum(ds)])

        self.spline_path = np.column_stack([x_sample, y_sample])
        self.arc_lengths = arc_lengths
        self.total_arc_length = arc_lengths[-1]

        self.get_logger().info(f'Spline total arc length: {self.total_arc_length:.2f} m')

    def _publish_all(self):
        """Publish path, info, and marker."""
        if self.spline_path is None:
            return

        stamp = self.get_clock().now().to_msg()
        self._publish_path(stamp)
        self._publish_info(stamp)
        self._publish_marker(stamp)

    def _publish_path(self, stamp):
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.frame_id

        for i in range(len(self.spline_path)):
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = self.spline_path[i, 0]
            pose.pose.position.y = self.spline_path[i, 1]
            pose.pose.position.z = self.arc_lengths[i]  # theta (arc-length param)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def _publish_info(self, stamp):
        """Publish JSON metadata about the current path."""
        row = self.rows[self.active_row_idx]
        info = {
            'active_row': self.active_row_idx,
            'row_name': row['name'],
            'total_arc_length_m': round(self.total_arc_length, 3),
            'num_spline_points': self.spline_resolution,
            'utm_epsg': self.utm_epsg,
            'frame_id': self.frame_id,
        }
        msg = String()
        msg.data = json.dumps(info)
        self.info_pub.publish(msg)

    def _publish_marker(self, stamp):
        """Publish LINE_STRIP marker for RViz."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = 'farnav_global_path'
        marker.id = self.active_row_idx
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.3  # line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0

        for i in range(len(self.spline_path)):
            p = Point()
            p.x = self.spline_path[i, 0]
            p.y = self.spline_path[i, 1]
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)

    # ------------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------------
    def set_active_row_callback(self, request, response):
        """Cycle to the next row. Use Trigger since we just advance by one."""
        if not self.aligned:
            response.success = False
            response.message = 'Not yet aligned — waiting for GPS + odom'
            return response

        next_idx = (self.active_row_idx + 1) % len(self.rows)
        self.active_row_idx = next_idx
        self._build_path()

        row_name = self.rows[self.active_row_idx]['name']
        response.success = True
        response.message = f'Switched to row {self.active_row_idx}: {row_name}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
