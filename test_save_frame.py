"""Save one OAK RGB frame to disk."""
import asyncio
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
from farm_ng.core.uri_pb2 import Uri

async def main():
    config = EventServiceConfig(name='oak/0', host='172.16.107.128', port=50010)
    client = EventClient(config)
    async for event, payload in client.subscribe(
        SubscribeRequest(uri=Uri(path="/rgb", query="service_name=oak/0"), every_n=1),
        decode=True,
    ):
        with open('/home/het/lab/amiga_project/oak_test_frame.jpg', 'wb') as f:
            f.write(bytes(payload.image_data))
        print(f"Saved frame: {len(payload.image_data)} bytes -> oak_test_frame.jpg")
        break

asyncio.run(main())
