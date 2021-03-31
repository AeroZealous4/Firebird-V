import sys
import asyncio
import nest_asyncio
from bleak import BleakClient


WRITE_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
NOTIFY_UUID = "8801f158-f55e-4550-95f6-d260381b99e7"
BLE_ADRRESS = "84:CC:A8:5F:90:D6"

# test_address = "4E:EC:EE:3E:2A:2D"
# TEST_WRITE_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
# TEST_NOTIFY_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"

notify_queue = None
write_queue = None
loop = None


async def forward_request(request):
    await write_queue.put(request)


async def get_response():
    return await notify_queue.get()


async def notify(client, queue):

    async def esp32_response_handler(sender, data):
        esp_response = "{1}".format(sender, data)
        await queue.put(esp_response)

    await client.start_notify(NOTIFY_UUID, esp32_response_handler)
    while True:
        await asyncio.sleep(0.1)
        # await client.stop_notify(NOTIFY_UUID)


async def run(client, queue):
    while True:
        value = bytearray(await queue.get())
        await client.write_gatt_char(WRITE_UUID, value)
        print(f'\n\nForwarded: {value}\n\n')
        # await asyncio.sleep(10.0)


async def main(address):
    async with BleakClient(address) as client:
        # await client.connect()
        global notify_queue, write_queue
        notify_queue = asyncio.Queue(maxsize=1)
        write_queue = asyncio.Queue(maxsize=1)
        await asyncio.gather(notify(client, notify_queue), run(client, write_queue))


def connect_firebird():
    # if sys.version_info[1] > 6:
    #     asyncio.run(main(BLE_ADRRESS))
    # else:
    global loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    nest_asyncio.apply()
    loop.run_until_complete(main(BLE_ADRRESS))
