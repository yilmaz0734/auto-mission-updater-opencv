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
#print("Global Location (relative altitude) %s" % iha.location.global_relative_frame)
print("..........................\n")

'''road_angle_list = []

for i in range(10):
    road_angle_list.append(iha.attitude.yaw)
    time.sleep(0.5)

road_angle = sum(road_angle_list)/len(road_angle_list)
print("Road angle: %s" % road_angle)'''

video = cv2.VideoCapture(0)

if (video.isOpened() == False):
    print("Error reading video file")
video.set(3,1920)
video.set(4,1080)

start = time.time()
count = 1
telemetry_count = 1

saved_coordinates = []
run_once = 0
while(True):
    
    _, imageFrame = video.read()
    end = time.time()
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
    if (end-start)>200:
        break
    length = len(saved_coordinates)
    pwm = 1000
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
            pwm = 2000
            '''xreal,yreal=(0.8*xdist/radius),(0.8*ydist/radius)
            dist_target = math.sqrt(xreal**2 + yreal**2)
            cv2.circle(imageFrame,(x+w//2,y+h//2),radius,(0,255,255),3)
            cv2.putText(imageFrame,str(xreal)+" "+str(yreal)+" "+str(dist_target),((center_point[0]),(center_point[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.line(imageFrame,((center_point[0]),(center_point[1])),coordinates,(0,255,0),2)'''
    print(pwm)
    if pwm == 2000:
        set_servo(iha,11,pwm)
        run_once = 0
    elif pwm ==1000 and run_once==0:
        set_servo(iha,11,pwm)
        telemetry_sender(iha,telemetry_count)
        telemetry_count+=1
        run_once = 1
        
        
    
    #cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)

sys.stdout = old_stdout
log_file.close()
  
        
   
    