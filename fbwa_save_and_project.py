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
time.sleep(60)
if (video.isOpened() == False):
    print("Error reading video file")
video.set(3,1920)
video.set(4,1080)

start = time.time()
count = 1
run_once = 0
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

    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    end = time.time()
    count+=1
    if (end-start)>200:
        break
    length = len(saved_coordinates)
    for pic, contour in enumerate(contours[:1]):
        area = cv2.contourArea(contour)
        if(area > 1000):
            pitch,roll,yaw = iha.attitude.pitch,iha.attitude.roll,iha.attitude.yaw
            location = iha.location.global_relative_frame
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
            east_d = math.cos(yaw)*xreal - math.sin(yaw)*yreal
            north_d = math.sin(yaw)*xreal + math.cos(yaw)*yreal
            dist_target = math.sqrt(east_d**2+north_d**2)
            red_carpet_loc = get_location_meters(coordinates,north_d,east_d)
            info = """
x_d = {} y_d = {} p_d = {} \n
x_r = {} y_r = {} r_d = {} \n
pitch = {} yaw = {} roll = {} \n
east_t = {} north_t = {} t_d = {} \n
time = {} \n
p_l = {} \n
c_l = {} \n
            """.format(round(xdist,2),round(ydist,2),round(pixel_distance,2),
            round(xreal,2),round(yreal,2),round(real_distance,2),round(pitch,2),round(yaw,2),round(roll,2),
            round(east_d,2),round(north_d,2),round(dist_target,2),
            round(time.time()-start,2),
            [location.lat,location.lon,location.alt],
            [red_carpet_loc.lat,red_carpet_loc.lon,red_carpet_loc.alt])
            cv2.circle(imageFrame,(x+w//2,y+h//2),radius,(0,255,255),3)
            #cv2.putText(imageFrame,str(xtarget)+" "+str(ytarget)+" "+str(dist_target),((center_point[0]),(center_point[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.line(imageFrame,((center_point[0]),(center_point[1])),coordinates,(0,255,0),2)
            y0, dy = 7, 15
            for i, line in enumerate(info.split('\n')):
                y = y0 + i*dy
                cv2.putText(imageFrame, line, (0,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
            print("Red zone found! \n"+info + "\n -----------------------------------------------------")
            saved_coordinates.append(red_carpet_loc)
    cv2.imwrite("/home/pi/Desktop/auto-mission-updater-opencv/framesfbwa/frame{}.jpg".format(count),imageFrame)
            
    if len(saved_coordinates)!=0 and len(saved_coordinates)-length==0:
        print("First loop has ended!")
        break
if len(saved_coordinates)==0:
    print("No red carpet found!")
else:
    print("Red carpet found {} times!".format(len(saved_coordinates)))
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

print("Actual location: %s" % (act_lat,act_lon,act_alt))
red_zone_location = LocationGlobalRelative(act_lat,act_lon,act_alt)

time.sleep(10)
start_sec = time.time()
while True:
    pwm = 1000
    end_sec = time.time()
    if get_distance_meters(iha.location.global_relative_frame,red_zone_location)<=4:
        pwm = 2000
    if pwm == 2000:
        set_servo(iha,11,pwm)
        run_once = 0
    elif pwm == 1000 and run_once == 0:
        set_servo(iha,11,pwm)
        print("Servo worked!")
        run_once = 1
    if (end_sec-start_sec)>200:
        break
video.release()

endlast = time.time()
print("Video recording has been finished!")
print("Flight notes: ")
print("********************************************************")
print("Flight time: %s" % (endlast-start))
print("Total distance taken: %s" % get_distance_meters(iha.home_location, iha.location.global_relative_frame))
print("Home location: %s" % iha.home_location)
# Closes all the frames
cv2.destroyAllWindows()

sys.stdout = old_stdout
log_file.close()

