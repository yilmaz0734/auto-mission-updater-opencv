import time
from dronekit import connect

start = time.time()
connection_string="/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_3E0040001151303437363830-if00"

iha=connect(connection_string,wait_ready=True,timeout=100,baud=115200)
attitude = iha.attitude
print("..........................")
@iha.on_attribute('attitude')
def attitude_listener(self, name, msg):
   global attitude
   attitude = msg
time.sleep(5)
print("Attitude: %s" % attitude)
while True:
    time.sleep(0.5)
    end = time.time()
    if end-start > 15:
        break
    print("attitude: %s" % iha.attitude)