from dronekit import connect,VehicleMode,mavutil,LocationGlobal,Command,LocationGlobalRelative
import time
import numpy as np
import math
import cv2
import sys

ball_area=0.1
ball_mass=0.15
old_stdout = sys.stdout
log_file = open("log.txt","w")
sys.stdout = log_file

connection_string="/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_3E0040001151303437363830-if00"

print("Connecting to İHA...")
iha=connect(connection_string,wait_ready=True,timeout=100,baud=115200)

print("Connected to İHA!")
print("Mode: %s" % iha.mode)
print("Attitude: %s" % iha.attitude)
print("Velocity: %s" % iha.velocity)
print("Global Location (relative altitude) %s" % iha.location.global_relative_frame)

red_carpet_loc = None

def set_servo(vehicle, servo_number, pwm_value):
	pwm_value_int = int(pwm_value)
	msg = vehicle.message_factory.command_long_encode(
		0, 0, 
		mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
		0,
		servo_number,
		pwm_value_int,
		0,0,0,0,0
		)
	vehicle.send_mavlink(msg)

def get_location_meters(original_location,dNorth,dEast):
    earth_r = 6378137.0
    dLat = dNorth / earth_r
    dLon = dEast / (earth_r*math.cos(math.pi*original_location.lat/180))
    
    newLat = original_location.lat + (dLat * 180 / math.pi)
    newLon = original_location.lon + (dLon * 180 /math.pi)
    return LocationGlobalRelative(newLat,newLon,original_location.alt)

def get_distance_meters(aLoc1,aLoc2):
    dlat = aLoc2.lat - aLoc1.lat
    dlon = aLoc2.lon - aLoc1.lon
    return math.sqrt((dlat**2)+(dlon**2))*1.113195e5

def mission_updater(vehicle,carpet):
    # generate waypoints for the carpet
    print("Generating waypoints for the carpet...")
    waypoints = []
    for i in range(-20,1):
        if i%2==0:
            waypoints.append(get_location_meters(carpet,i,0))
    #create commands list with waypoints
    commands = []
    for wp in waypoints:
        commands.append(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,wp.lat,wp.lon,wp.alt))
    print("Commands list has been generated!")
    #download the commands
    print("Now, waiting for the vehicle to send the commands...")
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    print("Commands have been downloaded!")
    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)
    nextwaypoint=vehicle.commands.next
    if nextwaypoint==0:
        return None
    #insert the command
    print("The noisy waypoints are being deleted...")
    for i in range(nextwaypoint,nextwaypoint+15):
        lat,lon,alt=missionlist[i].x,missionlist[i].y,missionlist[i].z
        waypointlocation=LocationGlobalRelative(lat,lon,alt)
        lat_cur,lon_cur,alt_cur=missionlist[nextwaypoint-1].x,missionlist[nextwaypoint-1].y,missionlist[nextwaypoint-1].z
        waypointcurrent=LocationGlobalRelative(lat_cur,lon_cur,alt_cur)
        waypointtocurrent=get_distance_meters(waypointlocation,waypointcurrent)
        carpettocurrent=get_distance_meters(waypointcurrent,carpet)
        if waypointtocurrent-carpettocurrent<10.0:
            print("The waypoint with coordinates {},{},{} is being deleted...".format(lat,lon,alt))
            missionlist.pop(i)       
    print("The noisy waypoints are deleted!")
    print("The new waypoints are being inserted...")
    for new_command in range(len(commands)):
        print("New waypoint with coordinates {},{},{} is being inserted...".format(commands[new_command].x,commands[new_command].y,commands[new_command].z))
        missionlist.insert(nextwaypoint-1+new_command,commands[new_command])
    print("The new waypoints are inserted!")
    print("Now, upload is starting...")
    #upload the new list
    cmds.clear()
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()
    print("Mission update has been finished!")
    
def projectile_handler(vehicle,carpet,ball_area,ball_mass):
    # get the ground and air velocities
    print("Getting the ground and air velocities...")
    ground_velocity = vehicle.velocity
    vx,vy,vz = ground_velocity.x,ground_velocity.y,ground_velocity.z
    air_velocity = vehicle.airspeed
    wx,wy,wz = ground_velocity.x-air_velocity.x,ground_velocity.y-air_velocity.y,ground_velocity.z-air_velocity.z
    # get the coordinates of the vehicle
    print("Getting the coordinates of the vehicle...")
    vehicle_location = vehicle.location.global_relative_frame
    lat,lon,alt = vehicle_location.lat,vehicle_location.lon,vehicle_location.alt
    # calculate flight time with altitude information
    print("Calculating flight time with altitude information...")
    g = 9.80665
    gwx = (1.225 - 9.8*(10**(-5))*alt)*0.5*(wx**2)*ball_area/ball_mass
    gwy = (1.225 - 9.8*(10**(-5))*alt)*0.5*(wy**2)*ball_area/ball_mass
    gwz = (1.225 - 9.8*(10**(-5))*alt)*0.5*(wz**2)*ball_area/ball_mass
    t = vz + math.sqrt((vz**2)+2*alt*(g+gwz))
    # calculate the coordinates of the ball
    print("Calculating the latitude difference...")
    x = vx*t + gwx*(t**2)*0.5
    y = vy*t + gwy*(t**2)*0.5
    distance = math.sqrt(x**2+y**2)
    if distance>=get_distance_meters(carpet,vehicle_location):
        print("The projection conditions has been met, the ball is in the carpet!")
        return True
    else:
        return False

def save_and_plan(vehicle,xdist,ydist):
    print("Saving process has been started...")
    roll,yaw,pitch=vehicle.attitude.roll,vehicle.attitude.yaw,vehicle.attitude.pitch
    xreal,yreal=(4*xdist/radius)/math.cos(roll),(4*ydist/radius)/math.cos(pitch)
    try:
        xtarget,ytarget=round(math.sqrt(xreal**2+yreal**2)*math.sin(yaw+math.atan(xreal/yreal)),2),round(math.sqrt(xreal**2+yreal**2)*math.cos(yaw+math.atan(xreal/yreal)),2)
    except:
        xtarget,ytarget=round(math.sqrt(xreal**2+yreal**2)*math.sin(yaw+math.pi/2),2),round(math.sqrt(xreal**2+yreal**2)*math.cos(yaw+math.pi/2),2)
    dist_target=round(math.sqrt(xtarget**2+y**2),2)
    print("Saving process has been finished!, distance to target is {} m".format(dist_target))
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
    if (vehicle.location.global_relative_frame.east - carpet.east)*1.113195e5 < 15 and (vehicle.location_global_relative_frame.north - carpet.north)*1.113195e5 < 30 :
        return True
    else:
        return False
    
webcam = cv2.VideoCapture(0)
print("Webcam initialized, search for red carpet is started.")
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
            print("Red carpet found, the location will be saved.")
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

#check if the mode of İHA is "AUTO"
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
        set_servo(iha,13,2000)
        break

    
print("Second part of the mission is in progress...")
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


sys.stdout = old_stdout
log_file.close()