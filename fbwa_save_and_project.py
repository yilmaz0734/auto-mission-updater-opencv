from dronekit import connect,VehicleMode,mavutil,LocationGlobal,Command,LocationGlobalRelative
import time
import numpy as np
import math
import cv2
import sys
from plane_functions import *

old_stdout = sys.stdout
log_file = open("/home/pi/Desktop/auto-mission-updater-opencv/log.txt","w")
sys.stdout = log_file

connection_string="/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_3E0040001151303437363830-if00"

print("Connecting to İHA...")
iha=connect(connection_string,wait_ready=True,timeout=100,baud=115200)
print("..........................")
print("Connected to İHA!")
print("Mode: %s" % iha.mode)
print("Attitude: %s" % iha.attitude)
print("Velocity: %s" % iha.velocity)
print("Global Location (relative altitude) %s" % iha.location.global_relative_frame)
print("..........................\n")

road_angle_list = []

for i in range(10):
    road_angle_list.append(iha.attitude.yaw)
    time.sleep(0.5)

road_angle = sum(road_angle_list)/len(road_angle_list)
print("Road angle: %s" % road_angle)

video = cv2.VideoCapture(0)

if (video.isOpened() == False):
    print("Error reading video file")
video.set(3,1920)
video.set(4,1080)

start = time.time()
count = 1
telemetry_count = 1

saved_coordinates = []

while(True):
    _, imageFrame = video.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    red_lower,red_upper = np.array([136, 87, 111], np.uint8) , np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    kernal = np.ones((5, 5), "uint8")
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                              mask = red_mask)
    rgb = cv2.cvtColor(red_mask,cv2.COLOR_GRAY2BGR)

    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    count+=1
    end = time.time()
    if round(end - start,6)%1 == 0:
        telemetry_sender(iha,telemetry_count)
        telemetry_count+=1
    if (end-start)>150:
        break
    length = len(saved_coordinates)
    for pic, contour in enumerate(contours[:1]):
        area = cv2.contourArea(contour)
        if(area > 1000):
            print("Red carpet found, the location will be saved.")
            pitch,roll,yaw = iha.attitude.pitch,iha.attitude.roll,iha.attitude.yaw
            location = iha.location.global_relative_frame
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w/2
            cy = y + h/2
            radius = w/2
            coordinates = (int(x+w/2), int(y+h/2))
            center_point = (int(imageFrame.shape[1])//2,int(imageFrame.shape[0])//2)
            xdist , ydist = coordinates[0] - center_point[0],coordinates[1] - center_point[1]
            red_carpet_loc=save_and_plan(pitch,roll,yaw,location,xdist,ydist,radius)
            saved_coordinates.append(red_carpet_loc)
            
    if len(saved_coordinates)!=0 and len(saved_coordinates)-length==0:
        break
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
lats,lons,alts=[],[],[]
for i in saved_coordinates:
    lats.append(i.lat)
    lons.append(i.lon)
    alts.append(i.alt)
act_lat = sum(lats)/len(lats)
act_lon = sum(lons)/len(lons)
act_alt = sum(alts)/len(alts)
print("Actual location: %s" % (act_lat,act_lon,act_alt))
red_zone_location = LocationGlobalRelative(act_lat,act_lon,act_alt)

start_sec = time.time()
telemetry_count_sec = 1
while True:
    end_sec = time.time()
    if round(end_sec-start_sec,6)%0.5==0:
        fall_time = math.sqrt(2*iha.location.global_relative_frame.alt/9.8)
        veast,vnorth,vz = iha.velocity.y,iha.velocity.x,iha.velocity.z
        vx = vnorth*math.sin(iha.yaw)+veast*math.cos(iha.yaw)
        vy = vnorth*math.cos(iha.yaw)-veast*math.sin(iha.yaw)
        if get_distance_meters(iha.location.global_relative_frame,red_zone_location)<=vy*fall_time:
            pwm = 2000
        else:
            pwm = 1000

        set_servo(iha,12,pwm)
        if pwm == 2000:
            print("Ball is falling!")
            break
    if round(end - start,6)%1 == 0:
        telemetry_sender(iha,telemetry_count_sec)
        telemetry_count_sec+=1
    if (end_sec-start_sec)>150:
        break
video.release()

# Closes all the frames
cv2.destroyAllWindows()

sys.stdout = old_stdout
log_file.close()

