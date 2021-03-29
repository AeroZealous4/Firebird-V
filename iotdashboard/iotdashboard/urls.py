from django.urls import path, include
from django.contrib import admin
from django.conf.urls.i18n import i18n_patterns
from django.conf import settings
from django.conf.urls.static import static
from django.views.static import serve
# from rest_framework import routers
from devices import views as devices
# from datas import views as datas
# from channels.routing import ProtocolTypeRouter, URLRouter
# from devices import consumers

# application = ProtocolTypeRouter({
#     "websocket": URLRouter([
#         path("ws/ticks/", consumers.TicksSyncConsumer),
#     ]),
# })


# router = routers.DefaultRouter()

urlpatterns = i18n_patterns(
    # backoffice panels index page
    path('', devices.index, name='index'),

    # data query
    # path('datas/chart/ajax/<str:id>/', datas.data_chart_ajax, name='data_chart_ajax'),

    # django admin page
    path('admin/', admin.site.urls),
)

# urlpatterns += [
#     # REST framework
#     # path('api/', include(router.urls)),
#     # path('api-auth/', include('rest_framework.urls', namespace='rest_framework')),

#     # path('api/datas/', datas.DataList.as_view(), name='api_data'),
#     # path('api/datas/<int:pk>/', datas.DataDetail.as_view(), name='api_data_detail'),
# ]

urlpatterns += [
    path('media/<str:path>/', serve, {'document_root': settings.MEDIA_ROOT, }),
    path('static/<str:path>/', serve, {'document_root': settings.STATIC_ROOT, }),
]

urlpatterns += static(settings.STATIC_URL, document_root=settings.STATIC_ROOT)
urlpatterns += static(settings.MEDIA_URL, document_root=settings.MEDIA_ROOT)
