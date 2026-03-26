"""Debug: check what OAK camera returns."""
import asyncio
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
from farm_ng.core.uri_pb2 import Uri

async def main():
    config = EventServiceConfig(name='oak/0', host='172.16.107.128', port=50010)
    client = EventClient(config)

    print("Connecting to OAK /rgb on 172.16.107.128:50010 ...")
    count = 0
    async for event, payload in client.subscribe(
        SubscribeRequest(uri=Uri(path="/rgb", query="service_name=oak/0"), every_n=5),
        decode=True,
    ):
        print(f"\n--- Frame {count} ---")
        print(f"Event URI: {event.uri.path}")
        print(f"Payload type: {type(payload).__name__}")
        print(f"Payload fields: {[f.name for f in payload.DESCRIPTOR.fields]}")

        if hasattr(payload, 'image_data'):
            data = payload.image_data
            print(f"  image_data length: {len(data)}")
            if len(data) > 4:
                print(f"  first 16 bytes: {bytes(data[:16]).hex()}")
                # Check if JPEG (starts with ff d8 ff)
                if bytes(data[:3]) == b'\xff\xd8\xff':
                    print("  Format: JPEG")
                elif bytes(data[:4]) == b'\x89PNG':
                    print("  Format: PNG")
                else:
                    print(f"  Format: UNKNOWN (magic: {bytes(data[:4])})")
        else:
            print("  NO image_data field")

        # Check all non-empty fields
        for f in payload.DESCRIPTOR.fields:
            val = getattr(payload, f.name, None)
            if val and f.name != 'image_data':
                print(f"  {f.name}: {str(val)[:200]}")

        count += 1
        if count >= 3:
            break

    print("\nDone!")

asyncio.run(main())
