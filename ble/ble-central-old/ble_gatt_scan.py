import sys
import asyncio
import platform
from bleak import BleakClient


address = (
    "65:84:4D:5B:72:17"  # <--- Change to your device's address here if you are using Windows or Linux
    if platform.system() != "Darwin"
    else "B9EA5233-37EF-4DD6-87A8-2A875E821C46"  # <--- Change to your device's address here if you are using macOS
)


async def main(address):
    async with BleakClient(address) as client:
        services = await client.get_services()
        for i, ser in enumerate(services):
            print(f"Service {i+1}: {ser}")
            for j, char in enumerate(ser.characteristics):
                print(f"> Characteristic {j+1}: {char}")
                print(f">> Properties {char.properties}")
                [print(f">> Descriptor {k+1}: {desc}") for k, desc in enumerate(char.descriptors)]
            sys.stdout.write("\n")


if __name__ == "__main__":
    asyncio.run(main(address))
