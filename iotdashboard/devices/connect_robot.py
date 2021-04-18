import json
import time
import queue
import asyncio
import threading
import nest_asyncio
import channels.layers
from bleak import BleakClient
from django.conf import settings
from asgiref.sync import async_to_sync


# test_address = "4E:EC:EE:3E:2A:2D"
# TEST_WRITE_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
# TEST_NOTIFY_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"

class ESP32_BLE:

    forward_serve_queue = None
    notify_serve_queue = None
    notify_track_queue = None
    notify_debug_queue = None
    loop = None

    WRITE_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
    NOTIFY_SERVE_UUID = "8801f158-f55e-4550-95f6-d260381b99e7"
    NOTIFY_TRACK_UUID = "beb5483f-36e1-4688-b7f5-ea07361b26a8"
    NOTIFY_DEBUG_UUID = "beb54840-36e1-4688-b7f5-ea07361b26a8"
    
    BLE_ADRRESS = "3C:71:BF:4C:81:2A"
    BLE_ADRRESS = "84:CC:A8:5F:90:D6"


    async def get_track_response(self):
        return await self.notify_track_queue.get()

    async def get_serve_response(self):
        return await self.notify_serve_queue.get()








    def poll_ble_debug(self):
        while True:
            self.loop.run_until_complete(self.get_debug_response())
            # self.loop.run_until_complete(self.forward_request())


    def poll_ble_serve(self):
        while True:
            self.loop.run_until_complete(self.forward_request())
            # self.loop.run_until_complete(self.forward_request())


    async def get_debug_response(self):
        debug_msg = await self.notify_debug_queue.get()
        print(f'\n\nDebug Response: {debug_msg}\n\n')


    async def esp32_debug_handler(self, sender, data):
        esp_response = "{1}".format(sender, data)
        await self.notify_debug_queue.put(esp_response)
        print(f'\n\nDebug Response: {esp_response}\n\n')


    async def esp32_track_handler(self, sender, data):
        esp_response = "{1}".format(sender, data)
        print(f'\n\nTrack Response: {esp_response}\n\n')
        await self.notify_track_queue.put(esp_response)


    async def esp32_serve_handler(self, sender, data):
        esp_response = "{1}".format(sender, data)

        # matches = ["sca", "rot", "for", "deb"]

        # if any(x in esp_response for x in matches):
        #     self.notify_track_queue.put(esp_response)
        # else:
            # await self.notify_serve_queue.put(esp_response)

        await self.notify_serve_queue.put(esp_response)
        print(f'\n\nRobot Response: {esp_response}\n\n')


    async def forward_request(self, request):
        value = bytearray(request.encode('utf-8'))
        await self.client.write_gatt_char(self.WRITE_UUID, value)
        print(f'\n\nForwarded Server Request: {value}\n\n')


    # async def forward_request(self):
    #     value = self.forward_serve_queue.get()
    #     await self.client.write_gatt_char(self.WRITE_UUID, value)
    #     print(f'\n\nForwarded Server Request: {value}\n\n')


    async def connect_firebird(self, loop):
        nest_asyncio.apply()
        self.forward_serve_queue = queue.Queue(maxsize=0)
        self.notify_serve_queue = asyncio.Queue(maxsize=1)
        self.notify_track_queue = asyncio.Queue(maxsize=0)
        self.notify_debug_queue = asyncio.Queue(maxsize=0)

        # self.loop = asyncio.get_event_loop()
        self.loop = loop
        asyncio.set_event_loop(self.loop)

        self.client = BleakClient(self.BLE_ADRRESS)
        await self.client.connect()
        await self.client.start_notify(self.NOTIFY_SERVE_UUID, self.esp32_serve_handler)
        await self.client.start_notify(self.NOTIFY_TRACK_UUID, self.esp32_track_handler)
        await self.client.start_notify(self.NOTIFY_DEBUG_UUID, self.esp32_debug_handler)

        # threading.Thread(target=self.poll_ble_debug).start()
        # threading.Thread(target=broadcast_track, args=(self, )).start()
        # await asyncio.gather(self.poll_ble_debug())


# def broadcast_track(esp_response):
def broadcast_track(client):

    while True:
        esp_response = client.loop.run_until_complete(client.get_track_response())
        esp_response = esp_response.replace("bytearray(b'", "").replace("')", "").split('-')
        track_data = dict()

        # duration = 1.5

        if "scanned" in esp_response: 
            track_data = {"scanned": {"plot": esp_response[1], "color": esp_response[-1]}}
            # duration = 2.5

        elif "debris" in esp_response:
            track_data = {"debris": {"current": esp_response[1], "dir": esp_response[-1]}}
            # duration = 1.5

        elif "forward" in esp_response:
            track_data = {"forward": {"current": esp_response[1], "dest": esp_response[-1]}}
            # duration = 1.5

        elif "rotate" in esp_response:
            track_data = {"rotate": {"current": esp_response[1], "face_dir": esp_response[2], "rotate_dir": esp_response[-1]}}
            # duration = 1.5

        if track_data:
            # time.sleep(duration)

            channel_layer = channels.layers.get_channel_layer()
            async_to_sync(channel_layer.group_send)(
                settings.TRACK_GROUP_NAME,
                {
                    "type": 'track_robot',
                    "content": json.dumps(track_data),
                }
            )



def test_one(client):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(client.connect_firebird(loop))

    # threading.Thread(target=broadcast_track, args=(client, )).start()
    broadcast_track(client)

    # while True:
    #     loop.run_until_complete(client.get_debug_response())



def test():
    a = ESP32_BLE()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(a.connect_firebird(loop))
    loop.run_until_complete(a.forward_request("scan-7-25".encode('utf-8')))

    while True:
        loop.run_until_complete(a.get_serve_response())
