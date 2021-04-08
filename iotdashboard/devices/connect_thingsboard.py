from coapthon.client.helperclient import HelperClient
from asgiref.sync import async_to_sync
from django.conf import settings
import paho.mqtt.client as mqtt
from . import connect_robot
import channels.layers
import threading
import asyncio
import time
import json
import os


thingsboard_url = 'thingsboard.e-yantra.org'
ACCESS_TOKEN = 'XfZjYAMMwnyGOe48B7YS'
DEVICE_ID = '65e60a20-9773-11eb-950e-efef5c07c810'
subscribe_topic = 'v1/devices/me/rpc/request/+'
coapc = None
ble_client = None
record_copy = dict()


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    client.subscribe(subscribe_topic)

    # threading.Thread(target=test_broadcast_track).start()


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    request = json.loads(msg.payload)
    print(msg.topic + " " + str(msg.payload), "\n\n", request)

    # Forward server request to Firebird
    send_request(request)

    # Update web app with latest server request
    broadcast_serve({'request': request})

    rpc_id = os.path.split(msg.topic)[-1]

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
    global coapc
    coapc.post(path=f"api/v1/{ACCESS_TOKEN}/telemetry", payload=str(response))
    # coapc.post(path=f"api/v1/{ACCESS_TOKEN}/rpc/{rpc_id}", payload=str(response))

    broadcast_serve({'response': response})
    record_copy[msg.payload] = response

    # response = coapc.post(path=f"api/v1/{ACCESS_TOKEN}/rpc/{rpc_id}", payload=str(payload))
    # print(response.code, '\n\n')
    # coapc.stop()


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

    ble_client.loop.run_until_complete(ble_client.forward_request(esp_req_data.encode('utf-8')))


def fetch_serve_response(request_id, method_type, track_info):

    timeTaken = None
    # loop = asyncio.get_event_loop()
    # loop = asyncio.new_event_loop()
    # asyncio.set_event_loop(loop)

    robot_response = {"id": request_id}
    esp_response = ble_client.loop.run_until_complete(ble_client.get_serve_response())
    esp_response = esp_response.replace("bytearray(b'", "").replace("')", "")

    while "accepted" == esp_response:
        broadcast_serve({'accept': method_type})
        esp_response = ble_client.loop.run_until_complete(ble_client.get_serve_response())
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

    # time.sleep(6)

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


def broadcast_track(request):
    channel_layer = channels.layers.get_channel_layer()
    async_to_sync(channel_layer.group_send)(
        settings.TRACK_GROUP_NAME,
        {
            "type": 'track_robot',
            "content": json.dumps(request),
        }
    )


def setup_mqtt():
    mqttc = mqtt.Client()
    mqttc.username_pw_set(ACCESS_TOKEN)
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message

    return mqttc


def setup_coap():
    global coapc
    coapc = HelperClient(server=("13.250.13.141", 5683))
    return coapc


def setup_ble():
    global ble_client
    ble_client = connect_robot.ESP32_BLE()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_client.connect_firebird())


def connect_server(mqttc):
    setup_ble()
    mqttc.connect(host=thingsboard_url, port=1883, keepalive=60)
    mqttc.loop_start()

    # mqttc.loop_forever()


def disconnect_server(mqttc, coapc):
    coapc.stop()
    mqttc.loop_stop()
    mqttc.disconnect()


def test_broadcast_track():
    duration = 1.5

    broadcast_track({"forward": {"current": 77, "dest": 77}})
    time.sleep(duration)
    broadcast_track({"forward": {"current": 77, "dest": 68}})
    time.sleep(duration)
    broadcast_track({"scanned": {"plot": 14, "color": "red"}})
    time.sleep(duration)
    broadcast_track({"rotate": {"current": 68, "face_dir": 'n', "rotate_dir": 'a'}})
    time.sleep(duration)
    broadcast_track({"scanned": {"plot": 15, "color": "green"}})
    time.sleep(duration)
    broadcast_track({"rotate": {"current": 68, "face_dir": 's', "rotate_dir": 'a'}})
    time.sleep(duration)
    broadcast_track({"forward": {"current": 68, "dest": 59}})
    time.sleep(duration)
    broadcast_track({"rotate": {"current": 59, "face_dir": 'n', "rotate_dir": 'r'}})
    time.sleep(duration)
    broadcast_track({"debris": {"current": 59, "dir": 'e'}})
    time.sleep(duration)
    broadcast_track({"rotate": {"current": 59, "face_dir": 'e', "rotate_dir": 'a'}})
    time.sleep(duration)
    broadcast_track({"forward": {"current": 59, "dest": 58}})
    time.sleep(duration)
    broadcast_track({"forward": {"current": 58, "dest": 57}})
