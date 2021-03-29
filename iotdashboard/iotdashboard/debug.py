from __future__ import unicode_literals

from iotdashboard.settings import DEBUG


def debug(*val):
    """
    :param val:
    :return:
    """
    if DEBUG:
        print(str(val).encode('utf-8'))
    return 0
