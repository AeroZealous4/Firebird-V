from django.conf import settings
from asgiref.sync import async_to_sync
from channels.consumer import AsyncConsumer
# from . import connect_thingsboard
from . import ble_central
import nest_asyncio
import threading


class TicksSyncConsumer(AsyncConsumer):

    mqttc = None
    coapc = None


    async def websocket_connect(self, event):

        subscribe_type = self.scope['url_route']['kwargs']['type']

        if subscribe_type == "serve":
            await self.send({
                'type': 'websocket.accept'
            })

            # Join serve group
            await self.channel_layer.group_add(
                settings.SERVE_GROUP_NAME,
                self.channel_name
            )

            if self.mqttc is None:
                nest_asyncio.apply()
                # self.mqttc = connect_thingsboard.setup_mqtt()
                # self.coapc = connect_thingsboard.setup_coap()
                # connect_thingsboard.connect_server(self.mqttc)
                self.mqttc = True
                threading.Thread(target=ble_central.start_iot).start()

        elif subscribe_type == "track":
            await self.send({
                'type': 'websocket.accept'
            })

            # Join track group
            await self.channel_layer.group_add(
                settings.TRACK_GROUP_NAME,
                self.channel_name
            )            


    async def websocket_disconnect(self, event):
        # Leave serve group
        await self.channel_layer.group_discard(
            settings.SERVE_GROUP_NAME,
            self.channel_name
        )

        # Leave track group
        await self.channel_layer.group_discard(
            settings.TRACK_GROUP_NAME,
            self.channel_name
        )

        # connect_thingsboard.disconnect_server(self.mqttc, self.coapc)
        ble_central.disconnect_server()


    async def serve_request(self, event):
        await self.send({
            'type': 'websocket.send',
            'text': event['content'],
        })


    async def track_robot(self, event):
        await self.send({
            'type': 'websocket.send',
            'text': event['content'],
        })
