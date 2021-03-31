from django.urls import path
from . import consumers

websocket_url = [path("ws/ticks/", consumers.TicksSyncConsumer.as_asgi()),]
