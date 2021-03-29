from django.urls import path
# from channels.routing import ProtocolTypeRouter, URLRouter
from . import consumers

websocket_url = [path("ws/ticks/", consumers.TicksSyncConsumer.as_asgi()),]

# application = ProtocolTypeRouter({
#     "websocket": URLRouter([
#         path("ws/ticks/", consumers.TicksSyncConsumer),
#     ]),
# })