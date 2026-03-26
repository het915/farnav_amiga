"""
Path overlay node: projects the global path and MPC predicted path
onto the OAK RGB camera image and publishes the annotated image.
"""
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TransformStamped

import tf2_ros


def _yaw_from_quat(q):
    """Extract yaw from geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


class PathOverlayNode(Node):

    def __init__(self):
        super().__init__('path_overlay')

        # Camera intrinsics (OAK-D RGB, native 3840x2160 scaled to 1920x1080)
        # From robot calibration: camera_data[0] (camera_number=0, RGB)
        self.declare_parameter('fx', 1155.08)
        self.declare_parameter('fy', 1154.84)
        self.declare_parameter('cx', 967.98)
        self.declare_parameter('cy', 523.38)
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)

        # Camera mount: position relative to base_link
        self.declare_parameter('cam_x', 0.0)    # forward from base_link (m)
        self.declare_parameter('cam_y', 0.0)     # left from base_link (m)
        self.declare_parameter('cam_z', 2.15)    # height above ground (m) — 7.05 ft
        self.declare_parameter('cam_pitch', 1.0)  # downward tilt (rad, ~57 deg, positive = look down)

        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.cam_x = self.get_parameter('cam_x').value
        self.cam_y = self.get_parameter('cam_y').value
        self.cam_z = self.get_parameter('cam_z').value
        self.cam_pitch = self.get_parameter('cam_pitch').value

        # State
        self.global_path = None
        self.mpc_path = None
        self.odom_pose = None  # (x, y, yaw)
        self.latest_image = None

        # Subscribers
        self.create_subscription(
            CompressedImage, '/amiga/oak/rgb/compressed', self._image_cb, 10)
        self.create_subscription(
            Path, '/farnav/global_path', self._global_path_cb, 10)
        self.create_subscription(
            Path, '/farnav/mpc_predicted_path', self._mpc_path_cb, 10)
        self.create_subscription(
            Odometry, '/amiga/odom', self._odom_cb, 10)

        # Publisher
        self.overlay_pub = self.create_publisher(
            CompressedImage, '/farnav/path_overlay/compressed', 10)

        # Timer for overlay at ~10 Hz
        self.create_timer(0.1, self._overlay_loop)

        self.get_logger().info('Path overlay node started')

    def _image_cb(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def _global_path_cb(self, msg: Path):
        if len(msg.poses) < 2:
            return
        self.global_path = np.array([
            [p.pose.position.x, p.pose.position.y, 0.0]
            for p in msg.poses
        ])

    def _mpc_path_cb(self, msg: Path):
        if len(msg.poses) < 2:
            return
        self.mpc_path = np.array([
            [p.pose.position.x, p.pose.position.y, 0.0]
            for p in msg.poses
        ])

    def _odom_cb(self, msg: Odometry):
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self.odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )

    def _world_to_camera_pixels(self, world_points, robot_x, robot_y, robot_yaw):
        """
        Project 3D world points (in odom frame) to 2D camera pixel coordinates.

        Camera convention: Z forward, X right, Y down.
        """
        cos_yaw = np.cos(robot_yaw)
        sin_yaw = np.sin(robot_yaw)
        cos_pitch = np.cos(self.cam_pitch)
        sin_pitch = np.sin(self.cam_pitch)

        # Camera position in world
        cam_world_x = robot_x + self.cam_x * cos_yaw - self.cam_y * sin_yaw
        cam_world_y = robot_y + self.cam_x * sin_yaw + self.cam_y * cos_yaw
        cam_world_z = self.cam_z

        # Translate points to camera origin
        pts = world_points.copy()
        pts[:, 0] -= cam_world_x
        pts[:, 1] -= cam_world_y
        pts[:, 2] -= cam_world_z

        # Rotate to robot body frame (yaw only)
        # Body frame: X forward, Y left, Z up
        body_x = cos_yaw * pts[:, 0] + sin_yaw * pts[:, 1]
        body_y = -sin_yaw * pts[:, 0] + cos_yaw * pts[:, 1]
        body_z = pts[:, 2]

        # Rotate from body to camera frame with pitch
        # Camera: Z forward, X right, Y down
        cam_z = cos_pitch * body_x - sin_pitch * body_z   # forward/depth
        cam_x = -body_y                                     # right
        cam_y = sin_pitch * body_x + cos_pitch * body_z    # down

        # Filter points behind camera
        valid = cam_z > 0.5  # at least 0.5m in front

        # Project to pixels
        pixels = np.zeros((len(world_points), 2), dtype=np.int32)
        if np.any(valid):
            u = self.fx * cam_x[valid] / cam_z[valid] + self.cx
            v = self.fy * cam_y[valid] / cam_z[valid] + self.cy
            pixels[valid, 0] = np.clip(u, -9999, 9999).astype(np.int32)
            pixels[valid, 1] = np.clip(v, -9999, 9999).astype(np.int32)

        return pixels, valid

    def _draw_path_on_image(self, image, path_points, robot_x, robot_y, robot_yaw,
                            color, thickness=3, sample_step=5):
        """Draw a projected path onto the image."""
        if path_points is None or len(path_points) < 2:
            return

        # Sample path to reduce computation
        sampled = path_points[::sample_step]
        pixels, valid = self._world_to_camera_pixels(sampled, robot_x, robot_y, robot_yaw)

        # Draw connected line segments
        prev_pt = None
        for i in range(len(sampled)):
            if not valid[i]:
                prev_pt = None
                continue
            pt = (int(pixels[i, 0]), int(pixels[i, 1]))
            # Only draw if within image bounds (with margin)
            in_bounds = -100 < pt[0] < self.img_w + 100 and -100 < pt[1] < self.img_h + 100
            if prev_pt is not None and in_bounds:
                cv2.line(image, prev_pt, pt, color, thickness, cv2.LINE_AA)
            if in_bounds:
                prev_pt = pt
            else:
                prev_pt = None

    def _overlay_loop(self):
        if self.latest_image is None or self.odom_pose is None:
            return

        image = self.latest_image.copy()
        rx, ry, ryaw = self.odom_pose

        # Draw global path in green
        self._draw_path_on_image(
            image, self.global_path, rx, ry, ryaw,
            color=(0, 255, 0), thickness=3, sample_step=3
        )

        # Draw MPC predicted path in red
        self._draw_path_on_image(
            image, self.mpc_path, rx, ry, ryaw,
            color=(0, 0, 255), thickness=2, sample_step=1
        )

        # Add legend
        cv2.putText(image, 'Global Path', (10, 30),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(image, 'MPC Predicted', (10, 60),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Publish
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_link'
        msg.format = 'jpeg'
        msg.data = bytes(cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])[1])
        self.overlay_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
