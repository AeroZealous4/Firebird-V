import os
from channels.auth import AuthMiddlewareStack
import django
from channels.http import AsgiHandler
from channels.routing import ProtocolTypeRouter, URLRouter
import devices.routing

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'iotdashboard.settings')
django.setup()

application = ProtocolTypeRouter({
  "http": AsgiHandler(),
  "websocket": URLRouter(
            devices.routing.websocket_url
        )
})