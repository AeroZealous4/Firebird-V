import asyncio
from bleak import discover
from bleak.backends.device import BLEDevice

def print_device(d: BLEDevice):
    to_print = f"""Device Name: {d.name}
Address: {d.address}
Metadata: {d.metadata}
Detail: {d.details}
RSSI: {d.rssi}"""
    print(to_print)


async def main():
    devices = await discover()
    for d in devices:
        print_device(d)
        if d.name != "ESP32":
            continue
        data = d.metadata['manufacturer_data']
        data = "".join([chr(value) for value in data[25673]])
        print("Data in Adv: " + data)

#if python version >= 3.7
asyncio.run(main())
# else
#loop = asyncio.get_event_loop()
#loop.run_until_complete(main())
