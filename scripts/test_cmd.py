"""Test sending a twist command directly to the Amiga via gRPC."""
import asyncio
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig

async def main():
    config = EventServiceConfig(name='canbus', host='172.16.107.128', port=6001)
    client = EventClient(config)

    # Send a very small forward velocity
    cmd = Twist2d(linear_velocity_x=0.1, linear_velocity_y=0.0, angular_velocity=0.0)
    print(f"Sending: linear_x=0.1 m/s, angular=0.0 rad/s")
    try:
        result = await asyncio.wait_for(
            client.request_reply('/canbus/amiga_twist2d', cmd),
            timeout=3.0,
        )
        print(f"Response: {result}")
    except Exception as e:
        print(f"Error: {type(e).__name__}: {e}")

    # Send stop
    cmd = Twist2d(linear_velocity_x=0.0, linear_velocity_y=0.0, angular_velocity=0.0)
    print("Sending stop command...")
    try:
        result = await asyncio.wait_for(
            client.request_reply('/canbus/amiga_twist2d', cmd),
            timeout=3.0,
        )
        print(f"Response: {result}")
    except Exception as e:
        print(f"Error: {type(e).__name__}: {e}")

asyncio.run(main())
