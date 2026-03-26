"""Send sustained twist commands to activate and move the Amiga."""
import asyncio
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
from farm_ng.core.uri_pb2 import Uri

async def main():
    config = EventServiceConfig(name='canbus', host='172.16.107.128', port=6001)
    client = EventClient(config)

    state_names = {0:'BOOT',1:'MANUAL_READY',2:'MANUAL_ACTIVE',3:'CC_ACTIVE',4:'AUTO_READY',5:'AUTO_ACTIVE',6:'ESTOPPED'}

    # Send commands at 10 Hz for 5 seconds (very slow: 0.1 m/s)
    print("Sending sustained forward commands at 0.1 m/s for 5 seconds...")
    print("Press Ctrl+C to stop immediately")
    try:
        for i in range(50):
            cmd = Twist2d(linear_velocity_x=0.1, linear_velocity_y=0.0, angular_velocity=0.0)
            await client.request_reply('/canbus/amiga_twist2d', cmd)

            # Check state every 10 messages
            if i % 10 == 0:
                async for event, payload in client.subscribe(
                    SubscribeRequest(uri=Uri(path='/state', query='service_name=canbus'), every_n=1),
                    decode=True,
                ):
                    tpdo = payload.amiga_tpdo1
                    s = int(tpdo.control_state)
                    print(f'  [{i/10:.0f}s] state={s} ({state_names.get(s)}), speed={tpdo.measured_speed:.3f}')
                    break

            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        pass

    # Stop
    print("Sending stop...")
    cmd = Twist2d(linear_velocity_x=0.0, linear_velocity_y=0.0, angular_velocity=0.0)
    for _ in range(10):
        await client.request_reply('/canbus/amiga_twist2d', cmd)
        await asyncio.sleep(0.1)
    print("Stopped.")

asyncio.run(main())
