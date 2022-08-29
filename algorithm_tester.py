import time
import numpy as np
import cv2
import math

video = cv2.VideoCapture(0)
if (video.isOpened() == False):
    print("Error reading video file")
video.set(3,1920)
video.set(4,1080)

print("Sleeping started!")
alt = 0
while True:
    alt+=1
    time.sleep(1)
    if alt > 5.0 :
        print("Altitude target reached, altitude is: %s" % alt)
        break

start = time.time()
count = 1
run_once,repeater = 0,0
saved_coordinates,frames_list = [],[]

pitch,roll,yaw = 0,0,0
lat,lon,alt = 15,15,15
next_waypoint = 0.1
while(True):
    next_waypoint += 0.1
    _, imageFrame = video.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    #red_lower = np.array([155,100,200])
    #red_upper = np.array([180,255,255])
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
    length = len(saved_coordinates)
    for pic, contour in enumerate(contours[:1]):
        area = cv2.contourArea(contour)
        if (area > 30000) and (area<500000):
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
            east_d = math.cos(yaw)*xreal - math.sin(yaw)*yreal
            north_d = math.sin(yaw)*xreal + math.cos(yaw)*yreal
            dist_target = math.sqrt(east_d**2+north_d**2)
            red_carpet_loc = dist_target
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
            round(xreal,2),round(yreal,2),round(real_distance,2),round(pitch,2),round(yaw,2),round(roll,2),
            round(east_d,2),round(north_d,2),round(dist_target,2),
            [lat,lon,alt],
            [lat,lon,alt],
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
    cv2.imwrite("frames_auto/frame{}.jpg".format(count),imageFrame)
    cv2.imshow("Frame",imageFrame)
    if repeater>=3:
        print("Target has been found")
        break
    print(next_waypoint)
    if next_waypoint>=6.0:
        print("Time is up, target not found")
        break

if len(saved_coordinates) == 0:
    red_zone_location = 50,50,50
else:
    print("Saved locations:")
    for i in range(len(saved_coordinates)):
        print("Coordinate: {} {} {} (frame {})".format(saved_coordinates[i],saved_coordinates[i],saved_coordinates[i],frames_list[i]))
    latr = sum([i for i in saved_coordinates])/len(saved_coordinates)
    lonr = sum([i for i in saved_coordinates])/len(saved_coordinates)
    altr = sum([i for i in saved_coordinates])/len(saved_coordinates)
    print("Actual location: {} {} {}".format(latr,lonr,10))
    red_zone_location = (latr,lonr,altr)

starter = time.time()
while True:
    time.sleep(1)
    if time.time()-starter >= 10.0:
        break

telemetry_count = 1
run_once = 0
print("Mission started, please wait...")

closed = 1813
opened = 1109

started_ball = time.time()
vyi = 5
while True:
    time.sleep(0.1)
    vyi += 5
    fall_time = math.sqrt(2*np.abs(alt)/9.98)   
    pwm = closed
    if 300<=vyi*fall_time+3:
        print("vy: {} fall_time: {} range_finder_height = {}".format(vyi,fall_time,alt))
        print("Target has been reached!")
        pwm = opened
    if pwm == opened:
        print("Opened")
        print("telemetry count: {}".format(telemetry_count))
        run_once = 0
        telemetry_count+=1
    elif pwm ==closed and run_once==0:
        print("Closed")
        run_once = 1
    if telemetry_count>=3:
        break
    if time.time()-started_ball >= 5.0:
        print("Opened because of timeout")
        break

start_to_second = time.time()
while True:
    time.sleep(1)
    print(time.time()-start_to_second)
    if time.time()-start_to_second>=10:
        break

telemetry_saved = telemetry_count
run_once_2 = 0
vy = 5
closed_t = 1109
opened_t = 1813
print("ball two started")
alt = 25
started_ball_two = time.time()
velocity = 5
while True:
    time.sleep(0.1)
    vy += 5
    fall_time = math.sqrt(2*np.abs(alt)/9.98)   
    pwm = closed_t
    if 300<=vy*fall_time+3:
        print("vy: {} fall_time: {} range_finder_height = {}".format(vy,fall_time,alt))
        print("Target has been reached!")
        pwm = opened_t
    if pwm == opened_t:
        print("Opened")
        run_once_2 = 0
        telemetry_count+=1
    elif pwm ==closed_t and run_once_2==0:
        print("Closed")
        run_once_2 = 1
    if telemetry_count-telemetry_saved>=3:
        break
    if time.time()-started_ball_two >= 25.0:
        print("Opened because of timeout")
        break