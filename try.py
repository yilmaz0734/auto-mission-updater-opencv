import time
from dronekit import connect

start= time.time()
connection_string="/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_3E0040001151303437363830-if00"

iha=connect(connection_string,wait_ready=True,timeout=100,baud=115200)
print("..........................")
@iha.on_attribute('attitude')
def attitude_listener(self, name, msg):
    print('%s attribute is: %s'%(name, msg))
while True:
    end = time.time()
    attitude_listener