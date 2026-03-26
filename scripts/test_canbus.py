"""Quick debug: connect to Amiga canbus and print raw payloads."""
import asyncio
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
from farm_ng.core.uri_pb2 import Uri

async def main():
    config = EventServiceConfig(name='canbus', host='172.16.107.128', port=6001)
    client = EventClient(config)

    print("Connecting to canbus at 172.16.107.128:6001 ...")
    count = 0
    async for event, payload in client.subscribe(
        SubscribeRequest(uri=Uri(path="/state", query="service_name=canbus"), every_n=1),
        decode=True,
    ):
        print(f"\n--- Message {count} ---")
        print(f"Event URI: {event.uri}")
        print(f"Payload type: {type(payload).__name__}")
        print(f"Payload fields: {[f.name for f in payload.DESCRIPTOR.fields]}")
        if hasattr(payload, 'amiga_tpdo1'):
            tpdo = payload.amiga_tpdo1
            print(f"  measured_speed: {tpdo.measured_speed}")
            print(f"  measured_angular_rate: {tpdo.measured_angular_rate}")
            print(f"  control_state: {tpdo.control_state}")
        else:
            print(f"  (no amiga_tpdo1)")
            # Print first 500 chars of str repr
            print(f"  repr: {str(payload)[:500]}")
        count += 1
        if count >= 5:
            break

    print("\nDone!")

asyncio.run(main())
