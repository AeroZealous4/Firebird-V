from django.forms import ModelForm

from devices.models import Device


class DeviceForm(ModelForm):
    required_css_class = 'required'

    class Meta:
        model = Device
        fields = [
            'name',
            'description',
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
            'enable',
        ]

    def __init__(self, *args, **kwargs):
        super(DeviceForm, self).__init__(*args, **kwargs)
        for i in self.fields:
            if i not in ['enable']:
                self.fields[i].widget.attrs['class'] = 'form-control'
