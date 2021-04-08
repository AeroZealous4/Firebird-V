from django.urls import path
from . import consumers

websocket_url = [path("ws/ticks/<str:type>", consumers.TicksSyncConsumer.as_asgi()),]
