from django.contrib import admin

from datas.models import Data


class DataAdmin(admin.ModelAdmin):
    """
    """
    list_display = ('id', 'device', 'remote_address', 'pub_date')


admin.site.register(Data, DataAdmin)
