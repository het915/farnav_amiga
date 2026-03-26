"""Combined bridge node: runs both streams and cmd_vel in a single process."""

import asyncio
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, CompressedImage
from std_msgs.msg import Float32, UInt32
from tf2_ros import TransformBroadcaster

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
from farm_ng.core.uri_pb2 import Uri


class AmigaBridgeNode(Node):
    def __init__(self):
        super().__init__('amiga_bridge')

        # Parameters
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('canbus_port', 6001)
        self.declare_parameter('oak_port', 50010)
        self.declare_parameter('gps_port', 50010)
        self.declare_parameter('filter_port', 20001)
        self.declare_parameter('oak_service_name', 'oak/0')
        self.declare_parameter('gps_service_name', 'gps')
        self.declare_parameter('filter_service_name', 'filter')
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
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # --- Publishers ---
        self.twist_pub = self.create_publisher(TwistStamped, '/amiga/twist', 10)
        self.odom_pub = self.create_publisher(Odometry, '/amiga/odom', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/amiga/gps', 10)
        self.imu_pub = self.create_publisher(Imu, '/amiga/imu', 10)
        self.image_left_pub = self.create_publisher(CompressedImage, '/amiga/oak/left/compressed', 10)
        self.image_rgb_pub = self.create_publisher(CompressedImage, '/amiga/oak/rgb/compressed', 10)
        self.battery_pub = self.create_publisher(Float32, '/amiga/battery', 10)
        self.control_state_pub = self.create_publisher(UInt32, '/amiga/control_state', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Subscribers (cmd_vel) ---
        self.create_subscription(TwistStamped, '/amiga/cmd_vel', self._cmd_vel_stamped_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # gRPC client for sending commands
        canbus_config = EventServiceConfig(
            name='canbus', host=self._host, port=self.get_parameter('canbus_port').value
        )
        self._canbus_cmd_client = EventClient(canbus_config)

        # Start async loop in background
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(f'Amiga bridge started -> {self._host}')

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
            tasks.append(self._stream_oak_left())
            tasks.append(self._stream_oak_rgb())
            tasks.append(self._stream_oak_imu())

        if not tasks:
            self.get_logger().warn('No streams enabled!')
            return

        results = await asyncio.gather(*tasks, return_exceptions=True)
        for r in results:
            if isinstance(r, Exception):
                self.get_logger().error(f'Stream task failed: {r}')

    def _make_client(self, name: str, port_param: str) -> EventClient:
        port = self.get_parameter(port_param).value
        config = EventServiceConfig(name=name, host=self._host, port=port)
        return EventClient(config)

    # ---- CMD VEL (ROS2 -> Amiga) ----

    def _send_twist(self, lx: float, ly: float, az: float):
        cmd = Twist2d(linear_velocity_x=lx, linear_velocity_y=ly, angular_velocity=az)
        fut = asyncio.run_coroutine_threadsafe(
            self._canbus_cmd_client.request_reply('/twist', cmd),
            self._loop,
        )
        try:
            fut.result(timeout=1.0)
        except Exception as e:
            self.get_logger().warn(f'Twist cmd failed: {e}')

    def _cmd_vel_stamped_cb(self, msg: TwistStamped):
        self._send_twist(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    def _cmd_vel_cb(self, msg: Twist):
        self._send_twist(msg.linear.x, msg.linear.y, msg.angular.z)

    # ---- CANBUS STREAM ----

    async def _stream_canbus(self):
        client = self._make_client('canbus', 'canbus_port')
        every_n = self.get_parameter('canbus_every_n').value
        self.get_logger().info('Subscribing to canbus...')
        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(uri=Uri(path="/state", query="service_name=canbus"), every_n=every_n),
                decode=True,
            ):
                now = self.get_clock().now().to_msg()
                if hasattr(payload, 'amiga_tpdo1'):
                    tpdo = payload.amiga_tpdo1
                    t = TwistStamped()
                    t.header.stamp = now
                    t.header.frame_id = self._base_frame
                    t.twist.linear.x = float(tpdo.measured_speed)
                    t.twist.angular.z = float(tpdo.measured_angular_rate)
                    self.twist_pub.publish(t)

                    s = UInt32()
                    s.data = int(tpdo.control_state)
                    self.control_state_pub.publish(s)

                if hasattr(payload, 'battery_charge_level'):
                    b = Float32()
                    b.data = float(payload.battery_charge_level)
                    self.battery_pub.publish(b)
        except Exception as e:
            self.get_logger().error(f'Canbus stream error: {e}')

    # ---- FILTER STREAM ----

    async def _stream_filter(self):
        svc_name = self.get_parameter('filter_service_name').value
        client = self._make_client(svc_name, 'filter_port')
        every_n = self.get_parameter('filter_every_n').value
        self.get_logger().info(f'Subscribing to filter (svc={svc_name}, port={self.get_parameter("filter_port").value})...')
        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(uri=Uri(path="/state", query=f"service_name={svc_name}"), every_n=every_n),
                decode=True,
            ):
                if not hasattr(payload, 'pose'):
                    continue
                now = self.get_clock().now().to_msg()
                iso = payload.pose.a_from_b
                q = iso.rotation.unit_quaternion
                tx, ty, tz = float(iso.translation.x), float(iso.translation.y), float(iso.translation.z)
                qx, qy, qz, qw = float(q.imag.x), float(q.imag.y), float(q.imag.z), float(q.real)

                odom = Odometry()
                odom.header.stamp = now
                odom.header.frame_id = self._odom_frame
                odom.child_frame_id = self._base_frame
                odom.pose.pose.position.x = tx
                odom.pose.pose.position.y = ty
                odom.pose.pose.position.z = tz
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw

                if hasattr(payload.pose, 'tangent_of_b_in_a'):
                    tang = payload.pose.tangent_of_b_in_a
                    odom.twist.twist.linear.x = float(tang.linear_velocity.x)
                    odom.twist.twist.linear.y = float(tang.linear_velocity.y)
                    odom.twist.twist.linear.z = float(tang.linear_velocity.z)
                    odom.twist.twist.angular.x = float(tang.angular_velocity.x)
                    odom.twist.twist.angular.y = float(tang.angular_velocity.y)
                    odom.twist.twist.angular.z = float(tang.angular_velocity.z)

                self.odom_pub.publish(odom)

                tf = TransformStamped()
                tf.header.stamp = now
                tf.header.frame_id = self._odom_frame
                tf.child_frame_id = self._base_frame
                tf.transform.translation.x = tx
                tf.transform.translation.y = ty
                tf.transform.translation.z = tz
                tf.transform.rotation.x = qx
                tf.transform.rotation.y = qy
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(tf)
        except Exception as e:
            self.get_logger().error(f'Filter stream error: {e}')

    # ---- GPS STREAM ----

    async def _stream_gps(self):
        svc_name = self.get_parameter('gps_service_name').value
        client = self._make_client(svc_name, 'gps_port')
        every_n = self.get_parameter('gps_every_n').value
        self.get_logger().info(f'Subscribing to GPS (svc={svc_name}, port={self.get_parameter("gps_port").value})...')
        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(uri=Uri(path="/pvt", query=f"service_name={svc_name}"), every_n=every_n),
                decode=True,
            ):
                now = self.get_clock().now().to_msg()
                msg = NavSatFix()
                msg.header.stamp = now
                msg.header.frame_id = self.get_parameter('gps_frame').value
                msg.latitude = float(payload.latitude)
                msg.longitude = float(payload.longitude)
                msg.altitude = float(payload.altitude)
                msg.status.status = (
                    NavSatStatus.STATUS_FIX if payload.status.gnss_fix_ok
                    else NavSatStatus.STATUS_NO_FIX
                )
                msg.status.service = NavSatStatus.SERVICE_GPS
                h = float(payload.horizontal_accuracy)
                v = float(payload.vertical_accuracy)
                msg.position_covariance = [h*h, 0., 0., 0., h*h, 0., 0., 0., v*v]
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                self.gps_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'GPS stream error: {e}')

    # ---- OAK CAMERA STREAMS (separate subscriptions per topic) ----

    async def _stream_oak_left(self):
        svc_name = self.get_parameter('oak_service_name').value
        client = self._make_client(svc_name, 'oak_port')
        every_n = self.get_parameter('oak_every_n').value
        cam_frame = self.get_parameter('camera_frame').value
        self.get_logger().info(f'Subscribing to OAK left (svc={svc_name})...')
        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(uri=Uri(path="/left", query=f"service_name={svc_name}"), every_n=every_n),
                decode=True,
            ):
                now = self.get_clock().now().to_msg()
                if hasattr(payload, 'image_data') and payload.image_data:
                    img = CompressedImage()
                    img.header.stamp = now
                    img.header.frame_id = cam_frame
                    img.format = 'jpeg'
                    img.data = bytes(payload.image_data)
                    self.image_left_pub.publish(img)
        except Exception as e:
            self.get_logger().error(f'OAK left stream error: {e}')

    async def _stream_oak_rgb(self):
        svc_name = self.get_parameter('oak_service_name').value
        client = self._make_client(svc_name, 'oak_port')
        every_n = self.get_parameter('oak_every_n').value
        cam_frame = self.get_parameter('camera_frame').value
        self.get_logger().info(f'Subscribing to OAK rgb (svc={svc_name})...')
        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(uri=Uri(path="/rgb", query=f"service_name={svc_name}"), every_n=every_n),
                decode=True,
            ):
                now = self.get_clock().now().to_msg()
                if hasattr(payload, 'image_data') and payload.image_data:
                    img = CompressedImage()
                    img.header.stamp = now
                    img.header.frame_id = cam_frame
                    img.format = 'jpeg'
                    img.data = bytes(payload.image_data)
                    self.image_rgb_pub.publish(img)
        except Exception as e:
            self.get_logger().error(f'OAK rgb stream error: {e}')

    async def _stream_oak_imu(self):
        svc_name = self.get_parameter('oak_service_name').value
        client = self._make_client(svc_name, 'oak_port')
        every_n = self.get_parameter('oak_every_n').value
        cam_frame = self.get_parameter('camera_frame').value
        self.get_logger().info(f'Subscribing to OAK imu (svc={svc_name})...')
        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(uri=Uri(path="/imu", query=f"service_name={svc_name}"), every_n=every_n),
                decode=True,
            ):
                now = self.get_clock().now().to_msg()
                # OakImuPackets has a 'packets' field
                packets = payload.packets if hasattr(payload, 'packets') else []
                for pkt in packets:
                    imu = Imu()
                    imu.header.stamp = now
                    imu.header.frame_id = cam_frame
                    if hasattr(pkt, 'acceleroMeter'):
                        a = pkt.acceleroMeter
                        imu.linear_acceleration.x = float(a.x)
                        imu.linear_acceleration.y = float(a.y)
                        imu.linear_acceleration.z = float(a.z)
                    if hasattr(pkt, 'gyroscope'):
                        g = pkt.gyroscope
                        imu.angular_velocity.x = float(g.x)
                        imu.angular_velocity.y = float(g.y)
                        imu.angular_velocity.z = float(g.z)
                    self.imu_pub.publish(imu)
        except Exception as e:
            self.get_logger().error(f'OAK IMU stream error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AmigaBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
