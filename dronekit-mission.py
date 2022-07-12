from dronekit import connect,VehicleMode,mavutil,LocationGlobal
import time
import numpy as np
import math
import cv2
connection_string="/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_3E0040001151303437363830-if00"

print("Connecting to İHA...")
iha=connect(connection_string,wait_ready=True,timeout=100,baud=115200)

print("Attitude: %s" % iha.attitude)
print("Velocity: %s" % iha.velocity)
print("Mode: %s" % iha.mode)
print("Global Location (relative altitude) %s" % iha.location.global_relative_frame)

red_carpet_loc = None

def get_location_meters(original_location,dNorth,dEast):
    earth_r = 6378137.0
    dLat = dNorth / earth_r
    dLon = dEast / (earth_r*math.cos(math.pi*original_location.lat/180))
    
    newLat = original_location.lat + (dLat * 180 / math.pi)
    newLon = original_location.lon + (dLon * 180 /math.pi)
    return LocationGlobal(newLat,newLon,original_location.alt)

def get_distance_metres(aLoc1,aLoc2):
    dlat = aLoc2.lat - aLoc1.lat
    dlon = aLoc2.lon - aLoc1.lon
    return math.sqrt((dlat**2)+(dlon**2))*1.113195e5

def download_mission(vehicle,x,y):
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    
def save_and_plan(vehicle,xdist,ydist):
    roll,yaw,pitch=vehicle.attitude.roll,vehicle.attitude.yaw,vehicle.attitude.pitch
    xreal,yreal=(4*xdist/radius)/math.cos(roll),(4*ydist/radius)/math.cos(pitch)
    try:
        xtarget,ytarget=round(math.sqrt(xreal**2+yreal**2)*math.sin(yaw+math.atan(xreal/yreal)),2),round(math.sqrt(xreal**2+yreal**2)*math.cos(yaw+math.atan(xreal/yreal)),2)
    except:
        xtarget,ytarget=round(math.sqrt(xreal**2+yreal**2)*math.sin(yaw+math.pi/2),2),round(math.sqrt(xreal**2+yreal**2)*math.cos(yaw+math.pi/2),2)
    dist_target=round(math.sqrt(xtarget**2+y**2),2)
    print("Saved coordinates are {} {}".format(x,y))
    red_carpet_loc = get_location_meters(vehicle.location.global_relative_frame,xtarget,ytarget)
    return red_carpet_loc
    
def switch_mode(vehicle,mode):
    print("Vehicle mode is being switched to {}...".format(mode))
    vehicle.mode = VehicleMode(mode)
    while not vehicle.mode.name==mode:
        print("Waiting for mode change")
        time.sleep(0.5)
    print("New mode: %s" % iha.mode)

def check_if_near(vehicle,carpet):
    if (vehicle.location.global_relative_frame.east - carpet.east)*1.113195e5 < 2 and (vehicle.location_global_relative_frame.north - carpet.north)*1.113195e5 < 20 :
        return True
    else:
        return False
    
def waypoint_creator(vehicle,point,cmds):
    nextpoint=vehicle.commands.next
    print(" Distance to waypoint (&s): %s" % (nextpoint,distance_to_current_waypoint()))
    
    
webcam = cv2.VideoCapture(0)
while True:
    _, imageFrame = webcam.read()

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
    for pic, contour in enumerate(contours[:1]):
        break_out_flag = False
        area = cv2.contourArea(contour)
        if(area > 1000):
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w/2
            cy = y + h/2
            radius = w/2
            coordinates = (int(x+w/2), int(y+h/2))
            center_point = (int(imageFrame.shape[1])//2,int(imageFrame.shape[0])//2)
            xdist , ydist = coordinates[0] - center_point[0],coordinates[1] - center_point[1]
            red_carpet_loc=save_and_plan(iha,xdist,ydist)
            '''cv2.circle(imageFrame,(x+w//2,y+h//2),radius,(0,255,255),0)
            cv2.putText(imageFrame,str(xtarget)+" "+str(ytarget)+" "+str(dist_target),((center_point[0]),(center_point[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.line(imageFrame,((center_point[0]),(center_point[1])),coordinates,(0,255,0),2)'''
            break_out_flag = True
            break
    if break_out_flag:
       break     
        
    '''cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break'''

switch_to_guided(iha)

