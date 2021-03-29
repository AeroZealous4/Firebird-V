from rest_framework import serializers
from .models import Data


class DataSerializer(serializers.ModelSerializer):
    """
    """
    api_key = serializers.HiddenField(default=None)

    class Meta:
        model = Data
        fields = ['id',
                  'device',
                  'field_1',
                  'field_2',
                  'field_3',
                  'field_4',
                  'field_5',
                  'field_6',
                  'field_7',
                  'field_8',
                  'field_9',
                  'field_10',
                  'remote_address',
                  'api_key']
