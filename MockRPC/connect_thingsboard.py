from coapthon.client.helperclient import HelperClient
import paho.mqtt.client as mqtt
import json
import os

thingsboard_url = 'thingsboard.e-yantra.org'
ACCESS_TOKEN = 'pxvRs3iucxEGEU0R4ik5'
DEVICE_ID = 'a9e821a0-8caf-11eb-950e-efef5c07c810'
subscribe_topic = 'v1/devices/me/rpc/request/+'


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    client.subscribe(subscribe_topic)


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
	print(msg.topic + " " + str(msg.payload), "\n\n", json.loads(msg.payload))

	payload = { 'result': 'ok' }
	rpc_id = os.path.split(msg.topic)[-1]
	response = coapc.post(path=f"api/v1/{ACCESS_TOKEN}/rpc/{rpc_id}", payload=str(payload))

	print(response.code, '\n\n')
	coapc.stop()


mqttc = mqtt.Client()
mqttc.username_pw_set(ACCESS_TOKEN)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

coapc = HelperClient(server=("13.250.13.141", 5683))
mqttc.connect(host=thingsboard_url, port=1883, keepalive=60)
mqttc.loop_forever()

# mqttc.loop_stop()
# mqttc.disconnect()