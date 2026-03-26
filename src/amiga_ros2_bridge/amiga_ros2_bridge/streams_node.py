"""Streams node: subscribes to Amiga gRPC services and publishes ROS2 topics."""

import asyncio
import math
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, CompressedImage
from std_msgs.msg import Float32, UInt32
from tf2_ros import TransformBroadcaster

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
from farm_ng.core.uri_pb2 import Uri


class AmigaStreamsNode(Node):
    def __init__(self):
        super().__init__('amiga_streams')

        # Declare parameters
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('canbus_port', 6001)
        self.declare_parameter('oak_port', 6002)
        self.declare_parameter('gps_port', 6003)
        self.declare_parameter('filter_port', 6004)
        self.declare_parameter('enable_canbus', True)
        self.declare_parameter('enable_oak', True)
        self.declare_parameter('enable_gps', True)
        self.declare_parameter('enable_filter', True)
        self.declare_parameter('canbus_every_n', 1)
        self.declare_parameter('oak_every_n', 2)
        self.declare_parameter('gps_every_n', 1)
        self.declare_parameter('filter_every_n', 1)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gps_frame', 'gps_link')
        self.declare_parameter('camera_frame', 'oak_link')

        self._host = self.get_parameter('host').value

        # Publishers
        self.twist_pub = self.create_publisher(TwistStamped, '/amiga/twist', 10)
        self.odom_pub = self.create_publisher(Odometry, '/amiga/odom', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/amiga/gps', 10)
        self.imu_pub = self.create_publisher(Imu, '/amiga/imu', 10)
        self.image_left_pub = self.create_publisher(CompressedImage, '/amiga/oak/left/compressed', 10)
        self.image_rgb_pub = self.create_publisher(CompressedImage, '/amiga/oak/rgb/compressed', 10)
        self.battery_pub = self.create_publisher(Float32, '/amiga/battery', 10)
        self.control_state_pub = self.create_publisher(UInt32, '/amiga/control_state', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Start async event loop in background thread
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(f'Amiga streams node started, connecting to {self._host}')

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._stream_all())

    async def _stream_all(self):
        tasks = []
        if self.get_parameter('enable_canbus').value:
            tasks.append(self._stream_canbus())
        if self.get_parameter('enable_filter').value:
            tasks.append(self._stream_filter())
        if self.get_parameter('enable_gps').value:
            tasks.append(self._stream_gps())
        if self.get_parameter('enable_oak').value:
            tasks.append(self._stream_oak())

        if not tasks:
            self.get_logger().warn('No streams enabled!')
            return

        await asyncio.gather(*tasks, return_exceptions=True)

    def _make_client(self, name: str, port_param: str) -> EventClient:
        port = self.get_parameter(port_param).value
        config = EventServiceConfig(name=name, host=self._host, port=port)
        return EventClient(config)

    async def _stream_canbus(self):
        client = self._make_client('canbus', 'canbus_port')
        every_n = self.get_parameter('canbus_every_n').value
        self.get_logger().info(f'Subscribing to canbus on port {self.get_parameter("canbus_port").value}')

        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(
                    uri=Uri(path="/state", query="service_name=canbus"),
                    every_n=every_n,
                ),
                decode=True,
            ):
                now = self.get_clock().now().to_msg()

                # Publish twist (measured velocity)
                if hasattr(payload, 'amiga_tpdo1'):
                    tpdo = payload.amiga_tpdo1
                    twist_msg = TwistStamped()
                    twist_msg.header.stamp = now
                    twist_msg.header.frame_id = self.get_parameter('base_frame').value
                    twist_msg.twist.linear.x = float(tpdo.measured_speed)
                    twist_msg.twist.angular.z = float(tpdo.measured_angular_rate)
                    self.twist_pub.publish(twist_msg)

                    # Control state
                    state_msg = UInt32()
                    state_msg.data = int(tpdo.control_state)
                    self.control_state_pub.publish(state_msg)

                # Battery
                if hasattr(payload, 'battery_charge_level'):
                    bat_msg = Float32()
                    bat_msg.data = float(payload.battery_charge_level)
                    self.battery_pub.publish(bat_msg)

        except Exception as e:
            self.get_logger().error(f'Canbus stream error: {e}')

    async def _stream_filter(self):
        client = self._make_client('filter', 'filter_port')
        every_n = self.get_parameter('filter_every_n').value
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value
        self.get_logger().info(f'Subscribing to filter on port {self.get_parameter("filter_port").value}')

        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(
                    uri=Uri(path="/state", query="service_name=filter"),
                    every_n=every_n,
                ),
                decode=True,
            ):
                now = self.get_clock().now().to_msg()

                if not hasattr(payload, 'pose'):
                    continue

                pose = payload.pose
                iso = pose.a_from_b

                # Extract quaternion
                quat = iso.rotation.unit_quaternion
                tx = float(iso.translation.x)
                ty = float(iso.translation.y)
                tz = float(iso.translation.z)
                qx = float(quat.imag.x)
                qy = float(quat.imag.y)
                qz = float(quat.imag.z)
                qw = float(quat.real)

                # Publish odometry
                odom_msg = Odometry()
                odom_msg.header.stamp = now
                odom_msg.header.frame_id = odom_frame
                odom_msg.child_frame_id = base_frame
                odom_msg.pose.pose.position.x = tx
                odom_msg.pose.pose.position.y = ty
                odom_msg.pose.pose.position.z = tz
                odom_msg.pose.pose.orientation.x = qx
                odom_msg.pose.pose.orientation.y = qy
                odom_msg.pose.pose.orientation.z = qz
                odom_msg.pose.pose.orientation.w = qw

                # Velocities from tangent
                if hasattr(pose, 'tangent_of_b_in_a'):
                    tang = pose.tangent_of_b_in_a
                    odom_msg.twist.twist.linear.x = float(tang.linear_velocity.x)
                    odom_msg.twist.twist.linear.y = float(tang.linear_velocity.y)
                    odom_msg.twist.twist.linear.z = float(tang.linear_velocity.z)
                    odom_msg.twist.twist.angular.x = float(tang.angular_velocity.x)
                    odom_msg.twist.twist.angular.y = float(tang.angular_velocity.y)
                    odom_msg.twist.twist.angular.z = float(tang.angular_velocity.z)

                self.odom_pub.publish(odom_msg)

                # Broadcast TF: odom -> base_link
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = odom_frame
                t.child_frame_id = base_frame
                t.transform.translation.x = tx
                t.transform.translation.y = ty
                t.transform.translation.z = tz
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Filter stream error: {e}')

    async def _stream_gps(self):
        client = self._make_client('gps', 'gps_port')
        every_n = self.get_parameter('gps_every_n').value
        self.get_logger().info(f'Subscribing to GPS on port {self.get_parameter("gps_port").value}')

        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(
                    uri=Uri(path="/state", query="service_name=gps"),
                    every_n=every_n,
                ),
                decode=True,
            ):
                now = self.get_clock().now().to_msg()

                gps_msg = NavSatFix()
                gps_msg.header.stamp = now
                gps_msg.header.frame_id = self.get_parameter('gps_frame').value

                # farm-ng stores lat/lon in radians
                gps_msg.latitude = float(payload.latitude)
                gps_msg.longitude = float(payload.longitude)
                gps_msg.altitude = float(payload.altitude)

                gps_msg.status.status = (
                    NavSatStatus.STATUS_FIX
                    if payload.status.gnss_fix_ok
                    else NavSatStatus.STATUS_NO_FIX
                )
                gps_msg.status.service = NavSatStatus.SERVICE_GPS

                # Covariance (diagonal, in m^2)
                h_acc = float(payload.horizontal_accuracy)
                v_acc = float(payload.vertical_accuracy)
                gps_msg.position_covariance = [
                    h_acc**2, 0.0, 0.0,
                    0.0, h_acc**2, 0.0,
                    0.0, 0.0, v_acc**2,
                ]
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

                self.gps_pub.publish(gps_msg)

        except Exception as e:
            self.get_logger().error(f'GPS stream error: {e}')

    async def _stream_oak(self):
        client = self._make_client('oak0', 'oak_port')
        every_n = self.get_parameter('oak_every_n').value
        camera_frame = self.get_parameter('camera_frame').value
        self.get_logger().info(f'Subscribing to OAK on port {self.get_parameter("oak_port").value}')

        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(
                    uri=Uri(path="/state", query="service_name=oak0"),
                    every_n=every_n,
                ),
                decode=True,
            ):
                now = self.get_clock().now().to_msg()

                # Left camera image
                if hasattr(payload, 'left') and payload.left.image_data:
                    img_msg = CompressedImage()
                    img_msg.header.stamp = now
                    img_msg.header.frame_id = camera_frame
                    img_msg.format = 'jpeg'
                    img_msg.data = bytes(payload.left.image_data)
                    self.image_left_pub.publish(img_msg)

                # RGB camera image
                if hasattr(payload, 'rgb') and payload.rgb.image_data:
                    img_msg = CompressedImage()
                    img_msg.header.stamp = now
                    img_msg.header.frame_id = camera_frame
                    img_msg.format = 'jpeg'
                    img_msg.data = bytes(payload.rgb.image_data)
                    self.image_rgb_pub.publish(img_msg)

                # IMU data
                if hasattr(payload, 'imu_packets') and payload.imu_packets:
                    for imu_pkt in payload.imu_packets:
                        imu_msg = Imu()
                        imu_msg.header.stamp = now
                        imu_msg.header.frame_id = camera_frame

                        if hasattr(imu_pkt, 'acceleroMeter'):
                            acc = imu_pkt.acceleroMeter
                            imu_msg.linear_acceleration.x = float(acc.x)
                            imu_msg.linear_acceleration.y = float(acc.y)
                            imu_msg.linear_acceleration.z = float(acc.z)

                        if hasattr(imu_pkt, 'gyroscope'):
                            gyro = imu_pkt.gyroscope
                            imu_msg.angular_velocity.x = float(gyro.x)
                            imu_msg.angular_velocity.y = float(gyro.y)
                            imu_msg.angular_velocity.z = float(gyro.z)

                        self.imu_pub.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f'OAK stream error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AmigaStreamsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
