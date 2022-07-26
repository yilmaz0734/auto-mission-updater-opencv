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
send_or_not = 1
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
    if (end-start)>120:
        break
    for pic, contour in enumerate(contours[:1]):
        area = cv2.contourArea(contour)
        if(area > 500):
            pitch,roll,yaw = iha.attitude.pitch,iha.attitude.roll,iha.attitude.yaw
            location = iha.location.global_relative_frame
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w/2
            cy = y + h/2
            radius = w/2
            coordinates = (int(x+w/2), int(y+h/2))
            center_point = (int(imageFrame.shape[1])//2,int(imageFrame.shape[0])//2)
            xdist , ydist = coordinates[0] - center_point[0],coordinates[1] - center_point[1]
            xreal,yreal=(0.8*xdist/radius),(0.8*ydist/radius)
            dist_target = math.sqrt(xreal**2 + yreal**2)
            cv2.circle(imageFrame,(x+w//2,y+h//2),radius,(0,255,255),3)
            cv2.putText(imageFrame,str(xreal)+" "+str(yreal)+" "+str(dist_target),((center_point[0]),(center_point[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.line(imageFrame,((center_point[0]),(center_point[1])),coordinates,(0,255,0),2)
            red_carpet_loc=save_and_plan(pitch,roll,yaw,location,xdist,ydist,radius)
            print("Red carpet location: %s" % red_carpet_loc)
            saved_coordinates.append(red_carpet_loc)
            send_or_not = 1
    cv2.imwrite("/home/pi/Desktop/auto-mission-updater-opencv/frames_mask/frame{}.jpg".format(count),rgb)
    cv2.imwrite("/home/pi/Desktop/auto-mission-updater-opencv/frames/frame{}.jpg".format(count),imageFrame)
    if send_or_not == 1:
        telemetry_count+=1
        if telemetry_count%5<2 and telemetry_count%5>0:
            telemetry_sender(iha,telemetry_count)
        send_or_not=0
        
    

endlast = time.time()
print("Video recording has been finished!")
print("Flight notes: ")
print("********************************************************")
print("Flight time: %s" % (endlast-start))
print("Total distance taken: %s" % get_distance_meters(iha.home_location, iha.location.global_relative_frame))
print("Home location: %s" % iha.home_location)
if len(saved_coordinates)==0:
    print("No red carpet found!")
else:
    print("Red carpet found!")
    print("Coordinates: ")
    for i in saved_coordinates:
        print("lat: {}, lon: {}, alt: {}".format(i.lat,i.lon,i.alt))
lats,lons,alts=[],[],[]
for i in saved_coordinates:
    lats.append(i.lat)
    lons.append(i.lon)
    alts.append(i.alt)
act_lat = sum(lats)/len(lats)
act_lon = sum(lons)/len(lons)
act_alt = sum(alts)/len(alts)
print("Actual location: {} {} {}".format(act_lat,act_lon,act_alt))
print("********************************************************")
# When everything done, release
# the video capture and video
# write objects
video.release()

# Closes all the frames
cv2.destroyAllWindows()

sys.stdout = old_stdout
log_file.close()