from django.conf import settings
from asgiref.sync import async_to_sync
from channels.consumer import AsyncConsumer
from . import connect_thingsboard
import nest_asyncio


class TicksSyncConsumer(AsyncConsumer):

    mqttc = None
    coapc = None


    async def websocket_connect(self, event):
        await self.send({
            'type': 'websocket.accept'
        })

        # Join ticks group
        await self.channel_layer.group_add(
            settings.TICKS_GROUP_NAME,
            self.channel_name
        )

        if self.mqttc is None:
            nest_asyncio.apply()
            self.mqttc = connect_thingsboard.setup_mqtt()
            self.coapc = connect_thingsboard.setup_coap()
            connect_thingsboard.connect_server(self.mqttc)


    async def websocket_disconnect(self, event):
        # Leave ticks group
        await self.channel_layer.group_discard(
            settings.TICKS_GROUP_NAME,
            self.channel_name
        )

        connect_thingsboard.disconnect_server(self.mqttc, self.coapc)


    async def new_ticks(self, event):
        await self.send({
            'type': 'websocket.send',
            'text': event['content'],
        })
