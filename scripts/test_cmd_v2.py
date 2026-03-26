"""Test different twist command endpoints."""
import asyncio
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
from farm_ng.core.uri_pb2 import Uri

async def check_state(client):
    state_names = {0:'BOOT',1:'MANUAL_READY',2:'MANUAL_ACTIVE',3:'CC_ACTIVE',4:'AUTO_READY',5:'AUTO_ACTIVE',6:'ESTOPPED'}
    async for event, payload in client.subscribe(
        SubscribeRequest(uri=Uri(path='/state', query='service_name=canbus'), every_n=1),
        decode=True,
    ):
        tpdo = payload.amiga_tpdo1
        s = int(tpdo.control_state)
        print(f'  state={s} ({state_names.get(s)}), speed={tpdo.measured_speed:.3f}')
        return s

async def main():
    config = EventServiceConfig(name='canbus', host='172.16.107.128', port=6001)
    client = EventClient(config)

    cmd = Twist2d(linear_velocity_x=0.1, linear_velocity_y=0.0, angular_velocity=0.0)

    # Try different request paths
    paths = [
        '/twist',
        '/canbus/twist',
        '/canbus/amiga_twist2d',
        '/request/canbus/twist',
    ]

    for path in paths:
        print(f'\nTrying path: {path}')
        try:
            result = await asyncio.wait_for(
                client.request_reply(path, cmd),
                timeout=2.0,
            )
            print(f'  SUCCESS: {result.event.uri.path}')
            await check_state(client)
        except Exception as e:
            print(f'  FAILED: {type(e).__name__}: {str(e)[:150]}')

    # Send stop
    stop = Twist2d(linear_velocity_x=0.0, linear_velocity_y=0.0, angular_velocity=0.0)
    await client.request_reply('/canbus/amiga_twist2d', stop)

asyncio.run(main())
