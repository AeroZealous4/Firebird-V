from coapthon.client.helperclient import HelperClient
from asgiref.sync import async_to_sync
from django.conf import settings
import paho.mqtt.client as mqtt
import channels.layers
import json
import os


thingsboard_url = 'thingsboard.e-yantra.org'
ACCESS_TOKEN = 'pxvRs3iucxEGEU0R4ik5'
DEVICE_ID = 'a9e821a0-8caf-11eb-950e-efef5c07c810'
subscribe_topic = 'v1/devices/me/rpc/request/+'
coapc = None
record_copy = dict()


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    client.subscribe(subscribe_topic)


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
	request = msg.payload
	print(msg.topic + " " + str(msg.payload), "\n\n", json.loads(request))

	broadcast_ticks(json.loads(msg.payload))

	# record_copy[request.payload] = response

	payload = { 'result': 'ok' }
	rpc_id = os.path.split(msg.topic)[-1]
	coapc.post(path=f"api/v1/{ACCESS_TOKEN}/rpc/{rpc_id}", payload=str(payload))

	# response = coapc.post(path=f"api/v1/{ACCESS_TOKEN}/rpc/{rpc_id}", payload=str(payload))
	# print(response.code, '\n\n')
	# coapc.stop()


def broadcast_ticks(request):
    print("BITCH")
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


def connect_server(mqttc):
	mqttc.connect(host=thingsboard_url, port=1883, keepalive=60)
	mqttc.loop_start()
	# mqttc.loop_forever()


def disconnect_server(mqttc, coapc):
	coapc.stop()
	mqttc.loop_stop()
	mqttc.disconnect()
