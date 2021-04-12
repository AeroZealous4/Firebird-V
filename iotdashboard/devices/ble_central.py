from coapthon.client.helperclient import HelperClient
from asgiref.sync import async_to_sync
from django.conf import settings
import paho.mqtt.client as mqtt
import channels.layers
import threading
import asyncio
import time
import json
import os

import os
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


thingsboard_url = 'thingsboard.e-yantra.org'
ACCESS_TOKEN = 'TcFcRiMFf7WFxHh1TmBS'
DEVICE_ID = 'bfedd360-9906-11eb-950e-efef5c07c810'
subscribe_topic = 'v1/devices/me/rpc/request/+'
coapc = None
mqttc = None
record_copy = dict()

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
#BLE_ADRRESS = "84:CC:A8:5F:90:D6"


async def notify_debug(client):
    def keypress_debug_handler(sender, data):
        print("\n\nDEBUG Response: {1}\n\n".format(sender, data))
    await client.start_notify(NOTIFY_DEBUG_UUID, keypress_debug_handler)
    while True:
        await asyncio.sleep(0.1)


async def notify_track(client):
    def keypress_track_handler(sender, data):
        esp_response = "{1}".format(sender, data)
        print(f'\n\nTRACK Response: {esp_response}\n\n')
        notify_track_queue.put(esp_response)
    await client.start_notify(NOTIFY_TRACK_UUID, keypress_track_handler)
    while True:
        await asyncio.sleep(0.1)


def poll_track():
    while True:
        esp_response = notify_track_queue.get()
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
            broadcast_track(track_data)

        # await asyncio.sleep(0.1)


async def notify_serve(client):
    def keypress_serve_handler(sender, data):
        esp_response = "{1}".format(sender, data)
        print(f'\n\nRobot Response: {esp_response}\n\n')
        notify_serve_queue.put(esp_response)
    await client.start_notify(NOTIFY_SERVE_UUID, keypress_serve_handler)
    while True:
        await asyncio.sleep(0.1)


async def forward_request(client):
    while True:
        value = await forward_serve_queue.get()
        # value = bytearray(b'scan-7-25')
        value = bytearray(value.encode('utf-8'))
        await client.write_gatt_char(WRITE_UUID, value)
        print(f'\n\nForwarded Server Request: {value}\n\n')
        await asyncio.sleep(2.0)


async def main(address):
    global notify_track_queue, notify_serve_queue, forward_serve_queue
    notify_track_queue = queue.Queue(maxsize=0)
    notify_serve_queue = queue.Queue(maxsize=0)
    forward_serve_queue = asyncio.Queue(maxsize=0)

    threading.Thread(target=poll_track).start()

    async with BleakClient(address) as client:
        await asyncio.gather(
            # notify_debug(client),
            notify_track(client),
            # poll_track(),
            notify_serve(client),
            forward_request(client)
        )


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("\n\nConnected with result code " + str(rc) + "\n\n")

    client.subscribe(subscribe_topic)


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global coapc

    request = json.loads(msg.payload)
    print("\n\n", msg.topic + " " + str(msg.payload), "\n\n", request)

    # Update web app with latest server request
    broadcast_serve({'request': request})

    # Forward server request to Firebird
    try:
        send_request(request)
    except Exception as e:
        print("\n\nError with BLE: ", str(e), "\n\n", repr(e), "\n\n")
        response = {"Error": "Error with BLE Device"}
        coapc.post(path=f"api/v1/{ACCESS_TOKEN}/telemetry", payload=str(response))
        broadcast_serve({'response': response})

    # Fetch Firebird response
    if request["method"] == "scan":
        track_info = request["params"]["plot"]
    elif request["method"] == "fetchNearest":
        if "major" in request["params"]["type"]:
            track_info = "red"
        else:
            track_info = "green"
    else:
        track_info = None

    response = fetch_serve_response(request["params"]["id"], request["method"], track_info)

    # response = { 'result': 'ok' }

    # Forward Firebird response back to Server
    coapc.post(path=f"api/v1/{ACCESS_TOKEN}/telemetry", payload=str(response))
    # coapc.post(path=f"api/v1/{ACCESS_TOKEN}/rpc/{rpc_id}", payload=str(response))

    broadcast_serve({'response': response})
    record_copy[msg.payload] = response


def send_request(request):

    esp_req_data = None

    if request["method"] == "fetchNearest" and request["params"]["completeIn"] < 256:
        esp_req_data = "fetch-"
        if "major" in request["params"]["type"]:
            esp_req_data += "major-"
        else:
            esp_req_data += "minor-"

        esp_req_data += str(request["params"]["completeIn"])

    elif request["method"] == "scan" and request["params"]["completeIn"] < 256:
        esp_req_data = request["method"] + '-' + \
        str(request["params"]["plot"]) + '-' + \
        str(request["params"]["completeIn"])

    else:
        esp_req_data = "no-request"

    loop.run_until_complete(forward_serve_queue.put(esp_req_data))


def fetch_serve_response(request_id, method_type, track_info):

    timeTaken = None

    robot_response = {"id": request_id}
    esp_response = notify_serve_queue.get()
    esp_response = esp_response.replace("bytearray(b'", "").replace("')", "")

    while "accepted" == esp_response:
        broadcast_serve({'accept': method_type})
        esp_response = notify_serve_queue.get()
        esp_response = esp_response.replace("bytearray(b'", "").replace("')", "")

    if "major" in esp_response:
        robot_response["type"] = "majorInjury"
        timeTaken = esp_response.split('-')[-1]
        broadcast_track({"scanned": {"color": "red", "plot": track_info}})

    elif "minor" in esp_response:
        robot_response["type"] = "minorInjury"
        timeTaken = esp_response.split('-')[-1]
        broadcast_track({"scanned": {"color": "green", "plot": track_info}})

    elif "no" in esp_response:
        robot_response["type"] = "noInjury"
        timeTaken = esp_response.split('-')[-1]

    elif "fetch" in esp_response:
        robot_response["plot"] = esp_response.split('-')[1]
        timeTaken = esp_response.split('-')[-1]
        broadcast_track({"scanned": {"color": track_info, "plot": robot_response["plot"]}})

    else:
        robot_response["status"] = esp_response
        print(esp_response)

    if timeTaken is not None:
        robot_response["timeTaken"] = timeTaken

    return robot_response


def broadcast_serve(request):
    channel_layer = channels.layers.get_channel_layer()
    async_to_sync(channel_layer.group_send)(
        settings.SERVE_GROUP_NAME,
        {
            "type": 'serve_request',
            "content": json.dumps(request),
        }
    )

    # pass


def broadcast_track(request):
    channel_layer = channels.layers.get_channel_layer()
    async_to_sync(channel_layer.group_send)(
        settings.TRACK_GROUP_NAME,
        {
            "type": 'track_robot',
            "content": json.dumps(request),
        }
    )

    # pass


def setup_mqtt():
    global mqttc
    mqttc = mqtt.Client()
    mqttc.username_pw_set(ACCESS_TOKEN)
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message
    # return mqttc


def setup_coap():
    global coapc
    coapc = HelperClient(server=("13.250.13.141", 5683))
    # return coapc


def connect_server(mqttc):
    mqttc.connect(host=thingsboard_url, port=1883, keepalive=1000)
    mqttc.loop_start()


def disconnect_server():
    global coapc, mqttc
    coapc.stop()
    mqttc.loop_stop()
    mqttc.disconnect()


# if __name__ == "__main__":
def start_iot():
    setup_mqtt()
    setup_coap()
    connect_server(mqttc)

    global loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    nest_asyncio.apply()
    loop.run_until_complete(main(BLE_ADRRESS))
