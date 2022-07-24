from dronekit import connect,VehicleMode,mavutil,LocationGlobal,Command,LocationGlobalRelative
import time
from matplotlib.pyplot import get
import numpy as np
import math
import cv2
import sys
from dronekit_mission import *

log_file = open("log.txt","w")
sys.stdout = log_file

connection_string="/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_3E0040001151303437363830-if00"

print("Connecting to İHA...")
iha=connect(connection_string,wait_ready=True,timeout=100,baud=115200)
print("-----------------------------------------------------")
print("Connected to İHA!")
print("Mode: %s" % iha.mode)
print("Attitude: %s" % iha.attitude)
print("Velocity: %s" % iha.velocity)
print("Global Location (relative altitude) %s" % iha.location.global_relative_frame)
print("-----------------------------------------------------")
time.sleep(5)
if iha.mode == "AUTO":
    cmds = iha.commands
    cmds.download()
    cmds.wait_ready()
    print("Commands have been downloaded!")
    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)
    print("Waypoints : %s" % missionlist)
    
liste = []
for i in range(10):
    liste.append(get_the_angle(iha))
road_angle = sum(liste)/len(liste)
print("Road angle: %s" % road_angle)

video = cv2.VideoCapture(0)

if (video.isOpened() == False):
    print("Error reading video file")
frame_width = int(video.get(3))
frame_height = int(video.get(4))
fps = video.get(cv2.CAP_PROP_FPS)
size = (frame_width, frame_height)
# is stored in 'output.avi' file.
result = cv2.VideoWriter('output.avi',
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         1, size)

start = time.time()
count = 1
telemetry_count = 1
while(True):
    _, imageFrame = webcam.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    red_lower,red_upper = np.array([136, 87, 111], np.uint8) , np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    kernal = np.ones((5, 5), "uint8")
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                              mask = red_mask)
    concatted = cv2.vconcat([imageFrame, red_mask])
    result.write(concatted)
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)

    end = time.time()
    if (end - start)%2 == 0:
        print("-----------------------------------------------------")
        print("New telemetry data has been downloaded, telemetry number: %s" % telemetry_count)
        print("Mode: %s" % iha.mode)
        print("Attitude: %s" % iha.attitude)
        print("Velocity: %s" % iha.velocity)
        print("Global Location (relative altitude) %s" % iha.location.global_relative_frame)
        print("Distance from home %s" % get_distance_meters(iha.home_location, iha.location.global_relative_frame))
        print("-----------------------------------------------------")
        telemetry_count+=1
    if (end-start)>60:
        break
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break


print("Video recording has been finished!")
print("Last saved video is stored in 'output.avi' file.")
print("Flight time: %s" % (end-start))
print("Total distance taken: %s" % get_distance_meters(iha.home_location, iha.location.global_relative_frame))

# When everything done, release
# the video capture and video
# write objects
video.release()
result.release()

# Closes all the frames
cv2.destroyAllWindows()

sys.stdout = old_stdout
log_file.close()