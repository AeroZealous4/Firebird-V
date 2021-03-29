from django.contrib.auth import authenticate, login
from django.shortcuts import render
import json
from asgiref.sync import async_to_sync
import channels.layers
from django.conf import settings


def index(request):
    """
    :param request:
    :return:
    """
    server_request = True

    # auto login for test users
    # admin:Aa1234567890
    user = authenticate(username='admin', password='Aa1234567890')
    login(request, user)
    return render(request, "back/index.html", locals())
