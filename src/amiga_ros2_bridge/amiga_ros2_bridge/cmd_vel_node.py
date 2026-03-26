"""Cmd vel node: subscribes to ROS2 /amiga/cmd_vel and forwards to Amiga via gRPC."""

import asyncio
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig


class AmigaCmdVelNode(Node):
    def __init__(self):
        super().__init__('amiga_cmd_vel')

        self.declare_parameter('host', 'localhost')
        self.declare_parameter('canbus_port', 6001)

        self._host = self.get_parameter('host').value
        self._port = self.get_parameter('canbus_port').value

        # Accept both TwistStamped and plain Twist
        self.create_subscription(TwistStamped, '/amiga/cmd_vel', self._cmd_vel_stamped_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # gRPC client
        config = EventServiceConfig(name='canbus', host=self._host, port=self._port)
        self._client = EventClient(config)

        # Async event loop for gRPC calls
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'Amiga cmd_vel node started, forwarding to {self._host}:{self._port}'
        )

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _send_twist(self, linear_x: float, linear_y: float, angular_z: float):
        cmd = Twist2d(
            linear_velocity_x=linear_x,
            linear_velocity_y=linear_y,
            angular_velocity=angular_z,
        )
        future = asyncio.run_coroutine_threadsafe(
            self._client.request_reply('/canbus/amiga_twist2d', cmd),
            self._loop,
        )
        try:
            future.result(timeout=1.0)
        except Exception as e:
            self.get_logger().warn(f'Failed to send twist command: {e}')

    def _cmd_vel_stamped_cb(self, msg: TwistStamped):
        self._send_twist(
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.angular.z,
        )

    def _cmd_vel_cb(self, msg: Twist):
        self._send_twist(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        )


def main(args=None):
    rclpy.init(args=args)
    node = AmigaCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
