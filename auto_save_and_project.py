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
time.sleep(90)
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

attitude = iha.attitude
velocity = iha.velocity
global_location = iha.location.global_relative_frame

@iha.on_attribute('attitude')
def attitude_listener(self, name, msg):
    global attitude
    attitude = msg
@iha.on_attribute('location.global_relative_frame')
def location_listener(self, name, msg):
    global global_location
    global_location = msg
@iha.on_attribute('velocity')
def velocity_listener(self, name, msg):
    global velocity
    velocity = msg

road_angle_set=set()
start_first = time.time()
print("Road angle is being saved...")
while True:
    road_angle_set.add(attitude.yaw)
    if time.time()-start_first > 10:
        break
road_angle = sum(road_angle_set)/len(road_angle_set)
print("Road angle is: %s" % road_angle)


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

    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    count+=1
    if (end-start)>300:
        break
    length = len(saved_coordinates)
    for pic, contour in enumerate(contours[:1]):
        area = cv2.contourArea(contour)
        if(area > 20000):
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w/2
            cy = y + h/2
            radius = w/2
            coordinates = (int(x+w/2), int(y+h/2))
            center_point = (int(imageFrame.shape[1])//2,int(imageFrame.shape[0])//2)
            xdist , ydist = coordinates[0] - center_point[0],coordinates[1] - center_point[1]
            pixel_distance = math.sqrt(xdist**2 + ydist**2)
            xreal,yreal=(1.25*xdist/radius),(1.25*ydist/radius)
            real_distance = math.sqrt(xreal**2 + yreal**2)
            east_d = math.cos(-attitude.yaw)*xreal - math.sin(-attitude.yaw)*yreal
            north_d = math.sin(-attitude.yaw)*xreal + math.cos(-attitude.yaw)*yreal
            dist_target = math.sqrt(east_d**2+north_d**2)
            red_carpet_loc = get_location_meters(global_location,north_d,east_d)
            saved_coordinates.append(red_carpet_loc)
            saved_coordinates.append(global_location)
            info = """
x_d = {} y_d = {} p_d = {} \n
x_r = {} y_r = {} r_d = {} \n
pitch = {} yaw = {} roll = {} \n
east_t = {} north_t = {} t_d = {} \n
time = {} \n
p_l = {} \n
c_l = {} \n
pixel_num = {}
            """.format(round(xdist,2),round(ydist,2),round(pixel_distance,2),
            round(xreal,2),round(yreal,2),round(real_distance,2),round(attitude.pitch,2),round(attitude.yaw,2),round(attitude.roll,2),
            round(east_d,2),round(north_d,2),round(dist_target,2),
            round(time.time()-start,2),
            [global_location.lat,global_location.lon,global_location.alt],
            [red_carpet_loc.lat,red_carpet_loc.lon,red_carpet_loc.alt],
            area)
            y0, dy = 7, 15
            for i, line in enumerate(info.split('\n')):
                y = y0 + i*dy
                cv2.putText(imageFrame, line, (0,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
            print("Red zone found! \n"+info + "\n -----------------------------------------------------")
            cv2.circle(imageFrame,(x+w//2,y+h//2),int(radius),(0,255,255),3)
            cv2.putText(imageFrame,str(east_d)+" "+str(north_d)+" "+str(dist_target),((center_point[0]),(center_point[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.line(imageFrame,((center_point[0]),(center_point[1])),coordinates,(0,255,0),2)
    cv2.imwrite("/home/pi/Desktop/auto-mission-updater-opencv/frames_auto/frame{}.jpg".format(count),imageFrame)
            
    if len(saved_coordinates)!=0 and len(saved_coordinates)-length==0:
        print("Target has been found")
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


#check if the mode of İHA is "AUTO"
if iha.mode!="AUTO":
    print("IHA mode is not AUTO, please change it to AUTO")
    sys.exit()
else:
    print("Vehicle mode is AUTO, mission is being started...")

time.sleep(15)
while True:
    print("After ten seconds, red carpet checking is started.")
    if check_if_near(iha,red_zone_location,road_angle):
        print("Circle has been completed, there are 30 meters to the red carpet.")
        mission_updater(iha,red_zone_location,attitude.yaw)
        break
start_sec = time.time()
telemetry_count_sec = 1
run_once = 0
print("Mission started, please wait...")
while True:
    pwm = 2000
    end_sec = time.time()
    if get_distance_meters(iha.location.global_relative_frame,red_zone_location)<=6:
        pwm = 1000
    if pwm == 1000:
        set_servo(iha,11,pwm)
        run_once = 0
    elif pwm ==2000 and run_once==0:
        set_servo(iha,11,pwm)
        telemetry_sender(iha,telemetry_count)
        telemetry_count+=1
        run_once = 1

        
'''print("Second part of the mission is in progress...")
if iha.mode!="AUTO":
    print("Vehicle mode is not AUTO, switching to AUTO mode...")
    switch_mode(iha,"AUTO")
else:
    print("Vehicle mode is AUTO, mission is being started...")

time.sleep(10)
while True:
    print("After ten seconds, red carpet checking is started.")
    if check_if_near(iha,red_carpet_loc):
        print("Circle has been completed, there are 30 meters to the red carpet.")
        mission_updater(iha,red_carpet_loc)
        break
while True:
    if projectile_handler(iha,red_carpet_loc,ball_area,ball_mass):
        set_servo(iha,14,2000)
        break

'''
sys.stdout = old_stdout
log_file.close()