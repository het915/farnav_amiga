#!/usr/bin/env python3
"""
Logs all FarNav-relevant data to a timestamped text file.

Usage:
    ros2 run farnav data_logger
    ros2 run farnav data_logger --ros-args -p log_dir:=/tmp/farnav_logs -p rate_hz:=2.0
"""
import os
import time
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny, cosy))


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.declare_parameter('log_dir', os.path.expanduser('~/farnav_logs'))
        self.declare_parameter('rate_hz', 2.0)

        log_dir = self.get_parameter('log_dir').value
        rate_hz = self.get_parameter('rate_hz').value

        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(log_dir, f'farnav_{timestamp}.txt')
        self.f = open(self.log_path, 'w')

        # Latest data
        self.odom = None
        self.gps = None
        self.cmd_vel = None
        self.global_path = None
        self.mpc_path = None
        self.enabled = None
        self.path_info = None

        # Subscribers
        self.create_subscription(Odometry, '/amiga/odom', self._odom_cb, 10)
        self.create_subscription(NavSatFix, '/amiga/gps', self._gps_cb, 10)
        self.create_subscription(TwistStamped, '/amiga/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Path, '/farnav/global_path', self._global_path_cb, 10)
        self.create_subscription(Path, '/farnav/mpc_predicted_path', self._mpc_path_cb, 10)
        self.create_subscription(Bool, '/farnav/controller_enable', self._enable_cb, 10)
        self.create_subscription(String, '/farnav/path_info', self._info_cb, 10)

        # Write header
        self._write('=' * 80)
        self._write(f'FarNav Data Log — {timestamp}')
        self._write('=' * 80)

        # Timer for periodic logging
        self.create_timer(1.0 / rate_hz, self._log_tick)
        self.tick = 0

        self.get_logger().info(f'Logging to {self.log_path} at {rate_hz} Hz')

    def _write(self, line):
        self.f.write(line + '\n')
        self.f.flush()

    def _odom_cb(self, msg):
        self.odom = msg

    def _gps_cb(self, msg):
        self.gps = msg

    def _cmd_cb(self, msg):
        self.cmd_vel = msg

    def _global_path_cb(self, msg):
        self.global_path = msg

    def _mpc_path_cb(self, msg):
        self.mpc_path = msg

    def _enable_cb(self, msg):
        self.enabled = msg.data

    def _info_cb(self, msg):
        self.path_info = msg.data

    def _log_tick(self):
        self.tick += 1
        t = time.time()
        self._write(f'\n--- tick {self.tick} | t={t:.3f} ---')

        # GPS
        if self.gps:
            g = self.gps
            cov = g.position_covariance[0]
            self._write(
                f'GPS: lat={g.latitude:.8f} lon={g.longitude:.8f} alt={g.altitude:.2f} '
                f'cov={cov:.6f} status={g.status.status}'
            )

        # Odom
        if self.odom:
            o = self.odom
            x = o.pose.pose.position.x
            y = o.pose.pose.position.y
            yaw = _yaw_from_quat(o.pose.pose.orientation)
            vx = o.twist.twist.linear.x
            wz = o.twist.twist.angular.z
            self._write(
                f'ODOM: x={x:.4f} y={y:.4f} yaw={np.degrees(yaw):.2f}deg '
                f'v={vx:.4f}m/s wz={np.degrees(wz):.2f}deg/s'
            )

        # Controller enabled
        if self.enabled is not None:
            self._write(f'ENABLED: {self.enabled}')

        # Cmd vel
        if self.cmd_vel:
            c = self.cmd_vel
            self._write(
                f'CMD: v={c.twist.linear.x:.4f}m/s omega={c.twist.angular.z:.4f}rad/s'
            )

        # Path info
        if self.path_info:
            self._write(f'PATH_INFO: {self.path_info}')

        # Global path summary
        if self.global_path and len(self.global_path.poses) > 1:
            poses = self.global_path.poses
            sx = poses[0].pose.position.x
            sy = poses[0].pose.position.y
            ex = poses[-1].pose.position.x
            ey = poses[-1].pose.position.y
            arc_len = poses[-1].pose.position.z
            d = np.degrees(np.arctan2(ey - sy, ex - sx))
            self._write(
                f'GLOBAL_PATH: start=({sx:.2f},{sy:.2f}) end=({ex:.2f},{ey:.2f}) '
                f'dir={d:.1f}deg pts={len(poses)} arc_len={arc_len:.2f}m'
            )

        # MPC predicted path
        if self.mpc_path and len(self.mpc_path.poses) > 1:
            poses = self.mpc_path.poses
            sx = poses[0].pose.position.x
            sy = poses[0].pose.position.y
            ex = poses[-1].pose.position.x
            ey = poses[-1].pose.position.y
            d = np.degrees(np.arctan2(ey - sy, ex - sx))
            self._write(
                f'MPC_PRED: start=({sx:.2f},{sy:.2f}) end=({ex:.2f},{ey:.2f}) '
                f'dir={d:.1f}deg pts={len(poses)}'
            )

            # Log all MPC prediction points
            self._write('MPC_PRED_PTS:')
            for i, p in enumerate(poses):
                self._write(f'  [{i}] ({p.pose.position.x:.3f}, {p.pose.position.y:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Log saved to {node.log_path}')
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
