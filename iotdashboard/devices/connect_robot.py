import sys
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

    notify_serve_queue = None
    notify_track_queue = None
    loop = None

    WRITE_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
    NOTIFY_SERVE_UUID = "8801f158-f55e-4550-95f6-d260381b99e7"
    NOTIFY_TRACK_UUID = "beb5483f-36e1-4688-b7f5-ea07361b26a8"
    # BLE_ADRRESS = "84:CC:A8:5F:90:D6"
    BLE_ADRRESS = "3C:71:BF:4C:81:2A"


    async def esp32_track_handler(self, sender, data):
        esp_response = "{1}".format(sender, data)
        print(f'\n\nTrack Response: {esp_response}\n\n')
        # threading.Thread(target=broadcast_track, args=(esp_response, )).start()
        self.notify_track_queue.put(esp_response, block=False)


    def get_track_response(self):
        return self.notify_track_queue.get()


    async def get_serve_response(self):
        return await self.notify_serve_queue.get()


    async def esp32_serve_handler(self, sender, data):
        esp_response = "{1}".format(sender, data)
        await self.notify_serve_queue.put(esp_response)
        print(f'\n\nRobot Response: {esp_response}\n\n')


    async def forward_request(self, request):
        value = bytearray(request)
        await self.client.write_gatt_char(self.WRITE_UUID, value)
        print(f'\n\nForwarded Server Request: {value}\n\n')


    async def connect_firebird(self):
        nest_asyncio.apply()
        self.notify_serve_queue = asyncio.Queue(maxsize=1)
        self.notify_track_queue = queue.Queue(maxsize=0)
        self.loop = asyncio.get_event_loop()
        asyncio.set_event_loop(self.loop)
        self.client = BleakClient(self.BLE_ADRRESS)
        await self.client.connect()
        await self.client.start_notify(self.NOTIFY_SERVE_UUID, self.esp32_serve_handler)
        await self.client.start_notify(self.NOTIFY_TRACK_UUID, self.esp32_track_handler)
        threading.Thread(target=broadcast_track, args=(self, )).start()


# def broadcast_track(esp_response):
def broadcast_track(client):

    while True:
        esp_response = client.notify_track_queue.get()
        esp_response = esp_response.replace("bytearray(b'", "").replace("')", "").split('-')
        track_data = dict()

        duration = 1.5

        if "scanned" in esp_response: 
            track_data = {"scanned": {"plot": esp_response[1], "color": esp_response[-1]}}
            duration = 2.5

        elif "debris" in esp_response:
            track_data = {"debris": {"current": esp_response[1], "dir": esp_response[-1]}}
            duration = 1.5

        elif "forward" in esp_response:
            track_data = {"forward": {"current": esp_response[1], "dest": esp_response[-1]}}
            duration = 1.5

        elif "rotate" in esp_response:
            track_data = {"rotate": {"current": esp_response[1], "face_dir": esp_response[2], "rotate_dir": esp_response[-1]}}
            duration = 1.5

        if track_data:
            time.sleep(duration)

            channel_layer = channels.layers.get_channel_layer()
            async_to_sync(channel_layer.group_send)(
                settings.TRACK_GROUP_NAME,
                {
                    "type": 'track_robot',
                    "content": json.dumps(track_data),
                }
            )



def test():
    a = ESP32_BLE()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(a.connect_firebird())
    loop.run_until_complete(a.forward_request("scan-7-25".encode('utf-8')))

    while True:
        loop.run_until_complete(a.get_serve_response())
