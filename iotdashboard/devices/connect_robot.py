import sys
import asyncio
import nest_asyncio
from bleak import BleakClient

# test_address = "4E:EC:EE:3E:2A:2D"
# TEST_WRITE_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
# TEST_NOTIFY_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"

class ESP32_BLE:

    notify_queue = None
    write_queue = None
    loop = None

    WRITE_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
    NOTIFY_UUID = "8801f158-f55e-4550-95f6-d260381b99e7"
    BLE_ADRRESS = "84:CC:A8:5F:90:D6"


    async def get_response(self):
        return await self.notify_queue.get()


    async def esp32_response_handler(self, sender, data):
        esp_response = "{1}".format(sender, data)
        await self.notify_queue.put(esp_response)
        print(f'\n\nRobot Response: {esp_response}\n\n')


    async def forward_request(self, request):
        value = bytearray(request)
        await self.client.write_gatt_char(self.WRITE_UUID, value)
        print(f'\n\nForwarded Server Request: {value}\n\n')


    async def connect_firebird(self):
        nest_asyncio.apply()
        self.notify_queue = asyncio.Queue(maxsize=1)
        self.loop = asyncio.get_event_loop()
        asyncio.set_event_loop(self.loop)
        self.client = BleakClient(self.BLE_ADRRESS)
        await self.client.connect()
        await self.client.start_notify(self.NOTIFY_UUID, self.esp32_response_handler)


def test():
    a = ESP32_BLE()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(a.connect_firebird())
    loop.run_until_complete(a.forward_request("scan-7-25".encode('utf-8')))
    print(loop.run_until_complete(a.get_response()))

