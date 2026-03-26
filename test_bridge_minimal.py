"""Minimal test: canbus gRPC -> ROS2 publisher in one script."""
import asyncio
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
from farm_ng.core.uri_pb2 import Uri


class MinimalBridge(Node):
    def __init__(self):
        super().__init__('minimal_bridge')
        self.pub = self.create_publisher(TwistStamped, '/test_twist', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.count = 0
        self.get_logger().info('Minimal bridge started. Publishing /test_twist every 1s...')

        # Also start canbus stream
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_grpc, daemon=True)
        self._thread.start()

    def timer_cb(self):
        """Publish a dummy twist every second to verify ROS2 pubsub works."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(self.count)
        self.pub.publish(msg)
        self.get_logger().info(f'Published dummy twist #{self.count}')
        self.count += 1

    def _run_grpc(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._stream())

    async def _stream(self):
        config = EventServiceConfig(name='canbus', host='172.16.107.128', port=6001)
        client = EventClient(config)
        self.get_logger().info('Connecting to canbus gRPC...')
        try:
            async for event, payload in client.subscribe(
                SubscribeRequest(uri=Uri(path="/state", query="service_name=canbus"), every_n=5),
                decode=True,
            ):
                if hasattr(payload, 'amiga_tpdo1'):
                    tpdo = payload.amiga_tpdo1
                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.twist.linear.x = float(tpdo.measured_speed)
                    msg.twist.angular.z = float(tpdo.measured_angular_rate)
                    self.pub.publish(msg)
                    self.get_logger().info(
                        f'Published from canbus: speed={tpdo.measured_speed:.3f} ang={tpdo.measured_angular_rate:.3f}'
                    )
        except Exception as e:
            self.get_logger().error(f'gRPC error: {e}')


def main():
    rclpy.init()
    node = MinimalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
