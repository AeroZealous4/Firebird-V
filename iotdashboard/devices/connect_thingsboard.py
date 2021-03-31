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
ACCESS_TOKEN = 'pxvRs3iucxEGEU0R4ik5'
DEVICE_ID = 'a9e821a0-8caf-11eb-950e-efef5c07c810'
subscribe_topic = 'v1/devices/me/rpc/request/+'
coapc = None
ble_thread = None
record_copy = dict()


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    client.subscribe(subscribe_topic)


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    request = msg.payload
    print(msg.topic + " " + str(msg.payload), "\n\n", json.loads(request))

    # Forward server request to Firebird
    send_request(json.loads(request))

    # Update web app with latest server request
    broadcast_ticks({'request': json.loads(request)})

    rpc_id = os.path.split(msg.topic)[-1]

    # Fetch Firebird response
    response = fetch_response(rpc_id, json.loads(request)["method"])

    # response = { 'result': 'ok' }

    # Forward Firebird response back to Server
    coapc.post(path=f"api/v1/{ACCESS_TOKEN}/rpc/{rpc_id}", payload=str(response))

    broadcast_ticks({'response': response})
    record_copy[request] = response

    # response = coapc.post(path=f"api/v1/{ACCESS_TOKEN}/rpc/{rpc_id}", payload=str(payload))
    # print(response.code, '\n\n')
    # coapc.stop()


def send_request(request):

    esp_req_data = None

    if request["method"] == "fetchNearest":
        esp_req_data = "fetch-"
        if "major" in request["params"]["type"]:
            esp_req_data += "major-"
        else:
            esp_req_data += "minor-"

        esp_req_data += str(request["params"]["completeIn"])

    elif request["method"] == "scan":
        esp_req_data = request["method"] + '-'
        + str(request["params"]["plot"]) + '-'
        + str(request["params"]["completeIn"])

    else:
        esp_req_data = "no-request"

    # loop = asyncio.get_event_loop()
    # loop = asyncio.new_event_loop()
    # asyncio.set_event_loop(loop)
    connect_robot.loop.run_until_complete(connect_robot.forward_request(esp_req_data.encode('utf-8')))


def fetch_response(request_id, method_type):

    timeTaken = None
    # loop = asyncio.get_event_loop()
    # loop = asyncio.new_event_loop()
    # asyncio.set_event_loop(loop)

    robot_response = {"id": request_id}
    esp_response = connect_robot.loop.run_until_complete(connect_robot.get_response())
    esp_response = esp_response.replace("bytearray(b'", "").replace("')", "")

    if "accepted" == esp_response:
        broadcast_ticks({'accept': method_type})
        esp_response = connect_robot.loop.run_until_complete(connect_robot.get_response())

    if "major" in esp_response:
        robot_response["type"] = "majorInjury"
        timeTaken = esp_response.split('-')[-1]

    elif "minor" in esp_response:
        robot_response["type"] = "minorInjury"
        timeTaken = esp_response.split('-')[-1]

    elif "no" in esp_response:
        robot_response["type"] = "noInjury"
        timeTaken = esp_response.split('-')[-1]

    elif "fetch" in esp_response:
        robot_response["plot"] = esp_response.split('-')[1]
        timeTaken = esp_response.split('-')[-1]

    else:
        robot_response["status"] = esp_response
        print(esp_response)

    if timeTaken is not None:
        robot_response["timeTaken"] = timeTaken

    return robot_response


def broadcast_ticks(request):
    channel_layer = channels.layers.get_channel_layer()
    async_to_sync(channel_layer.group_send)(
        settings.TICKS_GROUP_NAME,
        {
            "type": 'new_ticks',
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
    global ble_thread
    # ble_thread = threading.Thread(target=connect_robot.connect_firebird)
    # ble_thread.start()
    connect_robot.connect_firebird()


def connect_server(mqttc):
    setup_ble()
    mqttc.connect(host=thingsboard_url, port=1883, keepalive=60)
    mqttc.loop_start()

    # mqttc.loop_forever()


def disconnect_server(mqttc, coapc):
    coapc.stop()
    mqttc.loop_stop()
    mqttc.disconnect()
