'''
from bluepy.btle import Scanner, DefaultDelegate

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr)
        elif isNewData:
            print("Received new data from", dev.addr)

scanner = Scanner().withDelegate(ScanDelegate())
devices = scanner.scan(10.0)

for dev in devices:
    print("Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi))
    for (adtype, desc, value) in dev.getScanData():
        print("  %s = %s" % (desc, value))
'''

import sys
import binascii
import struct
import time
from bluepy.btle import UUID, Peripheral

led_service_uuid = UUID(0xA000)
led_char_uuid = UUID(0xA001)

if len(sys.argv) != 2:
  print("Fatal, must pass device address:", sys.argv[0], "<device address="">")
  quit()

p = Peripheral(sys.argv[1], "random")
LedService=p.getServiceByUUID(led_service_uuid)

try:
    ch = LedService.getCharacteristics(led_char_uuid)[0]
    while 1:
      ch.write(struct.pack('<B', 0x00));
      print("Led2 on")
      time.sleep(1)
      ch.write(struct.pack('<B', 0x01));
      print("Led2 off")
      time.sleep(1)
finally:
    p.disconnect()
