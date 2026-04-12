#!/usr/bin/env python3
"""
GPS-based odometry node — replaces the farm-ng filter when it's unavailable.

Fuses:
  - GPS (NavSatFix)  → position in ENU odom frame via UTM
  - Canbus twist     → measured speed + angular rate
  - IMU gyroscope    → heading rate (complementary filter with canbus)

Publishes:
  - /amiga/odom      (nav_msgs/Odometry)
  - /tf              (odom -> base_link)

Usage:
    ros2 run farnav gps_odom --ros-args -p utm_epsg:=EPSG:32616
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

import pyproj


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


class GpsOdomNode(Node):
    def __init__(self):
        super().__init__('gps_odom')

        self.declare_parameter('utm_epsg', 'EPSG:32616')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('gyro_weight', 0.98)

        utm_epsg = self.get_parameter('utm_epsg').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        rate = self.get_parameter('publish_rate_hz').value
        self._gyro_weight = self.get_parameter('gyro_weight').value

        self.transformer = pyproj.Transformer.from_crs(
            'EPSG:4326', utm_epsg, always_xy=False)

        # state
        self._utm_origin = None  # (easting, northing) at first GPS fix
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._yaw = 0.0  # heading in odom frame (radians, ENU)
        self._yaw_initialized = False
        self._vx = 0.0
        self._wz = 0.0
        self._last_time = None
        self._gps_yaw = None  # heading from GPS movement

        # publishers
        self.odom_pub = self.create_publisher(Odometry, '/amiga/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # subscribers
        self.create_subscription(NavSatFix, '/amiga/gps', self._gps_cb, 10)
        self.create_subscription(TwistStamped, '/amiga/twist', self._twist_cb, 10)
        self.create_subscription(Imu, '/amiga/imu', self._imu_cb, 10)

        # publish timer
        self.create_timer(1.0 / rate, self._publish_odom)

        self.get_logger().info(
            f'GPS odom node started (UTM={utm_epsg}, '
            f'gyro_weight={self._gyro_weight})')

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            return

        easting, northing = self.transformer.transform(
            msg.latitude, msg.longitude)

        if self._utm_origin is None:
            self._utm_origin = (easting, northing)
            self.get_logger().info(
                f'GPS origin set: E={easting:.2f}, N={northing:.2f} '
                f'(lat={msg.latitude:.7f}, lon={msg.longitude:.7f})')
            return

        # position in odom frame (ENU: x=east, y=north)
        new_x = easting - self._utm_origin[0]
        new_y = northing - self._utm_origin[1]

        # estimate heading from GPS movement (only when moving)
        dx = new_x - self._odom_x
        dy = new_y - self._odom_y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > 0.05:  # only update heading if moved >5cm
            self._gps_yaw = math.atan2(dy, dx)
            if not self._yaw_initialized:
                self._yaw = self._gps_yaw
                self._yaw_initialized = True
                self.get_logger().info(
                    f'Heading initialized from GPS: {math.degrees(self._yaw):.1f} deg')

        self._odom_x = new_x
        self._odom_y = new_y

    def _twist_cb(self, msg: TwistStamped):
        self._vx = msg.twist.linear.x
        self._wz = msg.twist.angular.z

    def _imu_cb(self, msg: Imu):
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return

        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if dt <= 0 or dt > 0.5:
            return

        # integrate gyro for heading
        gyro_z = msg.angular_velocity.z
        self._yaw += gyro_z * dt

        # complementary filter: blend gyro heading with GPS heading
        if self._gps_yaw is not None and self._yaw_initialized:
            # wrap difference
            diff = self._gps_yaw - self._yaw
            diff = math.atan2(math.sin(diff), math.cos(diff))
            self._yaw += (1.0 - self._gyro_weight) * diff

        # normalize yaw
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

    def _publish_odom(self):
        if self._utm_origin is None:
            return

        now = self.get_clock().now().to_msg()
        q = quat_from_yaw(self._yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.angular.z = self._wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = self._odom_x
        tf.transform.translation.y = self._odom_y
        tf.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = GpsOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
