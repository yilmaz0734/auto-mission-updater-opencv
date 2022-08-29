from dronekit import connect,VehicleMode,mavutil,LocationGlobal,Command,LocationGlobalRelative
import time
import numpy as np
import math
import cv2
import sys
from plane_functions import *

'''old_stdout = sys.stdout
log_file = open("/home/pi/Desktop/auto-mission-updater-opencv/log.txt","w")
sys.stdout = log_file'''

connection_string="/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_420042001351313132383631-if00"

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
@iha.on_attribute('attitude')
def attitude_listener(self, name, msg):
    global attitude
    attitude = self.attitude

global_location = iha.location.global_relative_frame
@iha.on_attribute('location.global_relative_frame')
def location_listener(self, name, msg):
    global global_location
    global_location = self.location.global_relative_frame

velocity = iha.velocity
@iha.on_attribute('velocity')
def velocity_listener(self, name, msg):
    global velocity
    if velocity[1] == round(self.velocity[1],1) or velocity[0] == round(self.velocity[0],1):
        return
    velocity = self.velocity

video = cv2.VideoCapture(0)
if (video.isOpened() == False):
    print("Error reading video file")
video.set(3,1920)
video.set(4,1080)

print("Sleeping started!")
while True:
    time.sleep(1)
    if global_location.alt > 5.0 :
        print("Altitude target reached, altitude is: %s" % global_location.alt)
        break
start = time.time()
count = 1
run_once,repeater = 0,0
saved_coordinates,frames_list = [],[]

while(True):
    _, imageFrame = video.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    #red_lower,red_upper = np.array([155,100,200]),np.array([180,255,255])
    red_lower,red_upper = np.array([136, 87, 160], np.uint8) , np.array([180, 255, 255], np.uint8)
    #red_lower,red_upper = np.array([136, 87, 111], np.uint8) , np.array([180, 255, 255], np.uint8)
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
    length = len(saved_coordinates)
    for pic, contour in enumerate(contours[:1]):
        area = cv2.contourArea(contour)
        if (area > 25000) and (area<500000):
            repeater += 1
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w/2
            cy = y + h/2
            radius = w/2
            coordinates = (int(x+w/2), int(y+h/2))
            center_point = (int(imageFrame.shape[1])//2,int(imageFrame.shape[0])//2)
            xdist , ydist = coordinates[0] - center_point[0],center_point[1] - coordinates[1]
            pixel_distance = math.sqrt(xdist**2 + ydist**2)
            xreal,yreal=(1.25*xdist/radius),(1.25*ydist/radius)
            real_distance = math.sqrt(xreal**2 + yreal**2)
            east_d = math.cos(-attitude.yaw)*xreal - math.sin(-attitude.yaw)*yreal
            north_d = math.sin(-attitude.yaw)*xreal + math.cos(-attitude.yaw)*yreal
            dist_target = math.sqrt(east_d**2+north_d**2)
            red_carpet_loc = get_location_meters(global_location,north_d,east_d)
            saved_coordinates.append(red_carpet_loc)
            frames_list.append(count)
            info = """
x_d = {} y_d = {} p_d = {} \n
x_r = {} y_r = {} r_d = {} \n
pitch = {} yaw = {} roll = {} \n
east_t = {} north_t = {} t_d = {} \n
p_l = {} \n
c_l = {} \n
pixel_num = {}
            """.format(round(xdist,2),round(ydist,2),round(pixel_distance,2),
            round(xreal,2),round(yreal,2),round(real_distance,2),round(attitude.pitch,2),round(attitude.yaw,2),round(attitude.roll,2),
            round(east_d,2),round(north_d,2),round(dist_target,2),
            [global_location.lat,global_location.lon,global_location.alt],
            [red_carpet_loc.lat,red_carpet_loc.lon,red_carpet_loc.alt],
            area)
            print("Red carpet seen")
            y0, dy = 7, 15
            for i, line in enumerate(info.split('\n')):
                ky = y0 + i*dy
                cv2.putText(imageFrame, line, (0,ky), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
            cv2.circle(imageFrame,(x+w//2,y+h//2),int(radius),(0,255,255),3)
            cv2.putText(imageFrame,str(east_d)+" "+str(north_d)+" "+str(dist_target),
            ((center_point[0]),(center_point[1])), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, 
            (0, 255, 0), 2)
            cv2.line(imageFrame,((center_point[0]),(center_point[1])),coordinates,(0,255,0),2)
        else:
            repeater = 0
    cv2.imwrite("/home/pi/Desktop/auto-mission-updater-opencv/frames_auto/frame{}.jpg".format(count),imageFrame)
            
    if repeater>=2:
        print("Target has been found")
        break
    if time.time()-start>=35:
        break
    '''if next_waypoint==6:
        print("Time is up, target not found")
        break'''
if len(saved_coordinates) == 0:
    #red_zone_location = LocationGlobalRelative(38.7899983,30.4832843,20)
    red_zone_location = LocationGlobalRelative(38.8014737,30.4654822,20)
else:
    print("Saved locations:")
    for i in range(len(saved_coordinates)):
        print("Coordinate: {} {} {} (frame {})".format(saved_coordinates[i].lat,saved_coordinates[i].lon,saved_coordinates[i].alt,frames_list[i]))
    latr = sum([i.lat for i in saved_coordinates])/len(saved_coordinates)
    lonr = sum([i.lon for i in saved_coordinates])/len(saved_coordinates)
    altr = sum([i.alt for i in saved_coordinates])/len(saved_coordinates)
    print("Actual location: {} {} {}".format(latr,lonr,20))
    red_zone_location = LocationGlobalRelative(latr,lonr,20)

mission_updater_new(iha,red_zone_location)

starter = time.time()
while True:
    time.sleep(1)
    globed = global_location
    if time.time()-starter >= 5.0:
        break

telemetry_count = 1
run_once = 0
print("Mission started, please wait...")

closed = 982
opened = 1728

started_ball = time.time()

while True:
    time.sleep(0.1)
    vy = math.sqrt(velocity[1]*velocity[1]+velocity[0]*velocity[0])
    fall_time = math.sqrt(2*np.abs(global_location.alt)/9.98)  
    pwm = closed
    if get_distance_meters(global_location,red_zone_location)<=vy*fall_time+2.5:
        print("vy: {} fall_time: {} range_finder_height = {}".format(vy,fall_time,global_location.alt))
        print("Target has been reached!")
        print("There are {} meters to the target, plane is in {} {} and red zone is in {} {}".format(get_distance_meters(global_location,red_zone_location),
                                                                            global_location.lat,global_location.lon,
                                                                            red_zone_location.lat,red_zone_location.lon))
        pwm = opened
    if pwm == opened:
        set_servo(iha,11,pwm)
        telemetry_count += 1
        run_once = 0
    elif pwm ==closed and run_once==0:
        set_servo(iha,11,pwm)
        run_once = 1
    if telemetry_count>=3:
        break
    if time.time()-started_ball >= 20.0:
        set_servo(iha,11,opened)
        break

start_to_second = time.time()
while True:
    holder_loc = global_location
    if time.time()-start_to_second>=5:
        break

telemetry_saved = telemetry_count
run_once_2 = 0

closed_t = 2006
opened_t = 1220

started_ball_two = time.time()

while True:
    time.sleep(0.1)
    vy = math.sqrt(velocity[1]*velocity[1]+velocity[0]*velocity[0])
    fall_time = math.sqrt(2*np.abs(global_location.alt)/9.98)   
    pwm = closed_t
    if get_distance_meters(global_location,red_zone_location)<=vy*fall_time+2.5:
        print("vy: {} fall_time: {} range_finder_height = {}".format(vy,fall_time,global_location.alt))
        print("Target has been reached!")
        print("There are {} meters to the target, plane is in {} {} and red zone is in {} {}".format(get_distance_meters(global_location,red_zone_location),
                                                              global_location.lat,global_location.lon,
                                                              red_zone_location.lat,red_zone_location.lon))
        pwm = opened_t
    if pwm == opened_t:
        set_servo(iha,12,pwm)
        run_once_2 = 0
        telemetry_count+=1
    elif pwm ==closed_t and run_once_2==0:
        set_servo(iha,12,pwm)
        run_once_2 = 1
    if telemetry_count-telemetry_saved>=3:
        break
    if time.time()-started_ball_two >= 20.0:
        set_servo(iha,12,opened_t)
        break
'''sys.stdout = old_stdout
log_file.close()'''