from django.conf import settings
from asgiref.sync import async_to_sync
from channels.consumer import SyncConsumer
from . import connect_thingsboard


class TicksSyncConsumer(SyncConsumer):

    mqttc = None
    coapc = None


    def websocket_connect(self, event):
        self.send({
            'type': 'websocket.accept'
        })

        # Join ticks group
        async_to_sync(self.channel_layer.group_add)(
            settings.TICKS_GROUP_NAME,
            self.channel_name
        )

        if self.mqttc is None:
            self.mqttc = connect_thingsboard.setup_mqtt()
            self.coapc = connect_thingsboard.setup_coap()
            connect_thingsboard.connect_server(self.mqttc)


    def websocket_disconnect(self, event):
        # Leave ticks group
        async_to_sync(self.channel_layer.group_discard)(
            settings.TICKS_GROUP_NAME,
            self.channel_name
        )

        connect_thingsboard.disconnect_server(self.mqttc, self.coapc)


    def new_ticks(self, event):
        self.send({
            'type': 'websocket.send',
            'text': event['content'],
        })
