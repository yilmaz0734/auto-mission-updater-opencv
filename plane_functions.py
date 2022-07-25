import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, mavutil
import time

def get_distance_meters(aLoc1,aLoc2):
    dlat = aLoc2.lat - aLoc1.lat
    dlon = aLoc2.lon - aLoc1.lon
    return math.sqrt((dlat**2)+(dlon**2))*1.113195e5

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

def get_location_meters_for_road(original_location,dx,dy,road_angle):
    earth_r = 6378137.0
    try:
        dNorth = math.sin(math.atan(dy/dx)-road_angle)*math.sqrt(dx**2+dy**2)
        dEast = math.cos(math.atan(dy/dx)-road_angle)*math.sqrt(dx**2+dy**2)
    except:
        dNorth = math.sin(math.pi/2-road_angle)*math.sqrt(dx**2+dy**2)
        dEast = math.cos(math.pi/2-road_angle)*math.sqrt(dx**2+dy**2)
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
    for i in [10,0,-10]:
        waypoints.append(get_location_meters_for_road(carpet,0,i))
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
    for i in range(nextwaypoint,nextwaypoint+5):
        lat,lon,alt=missionlist[i].x,missionlist[i].y,missionlist[i].z
        waypointlocation=LocationGlobalRelative(lat,lon,alt)
        lat_cur,lon_cur,alt_cur=missionlist[nextwaypoint-1].x,missionlist[nextwaypoint-1].y,missionlist[nextwaypoint-1].z
        waypointcurrent=LocationGlobalRelative(lat_cur,lon_cur,alt_cur)
        waypointtocurrent=get_distance_meters(waypointlocation,waypointcurrent)
        carpettocurrent=get_distance_meters(waypointcurrent,carpet)
        if waypointtocurrent-carpettocurrent<30.0:
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
    
def projectile_handler(vehicle,carpet,ball_area,ball_mass,road_angle):
    # get the ground and air velocities
    print("Getting the ground and air velocities...")
    ground_velocity = vehicle.velocity
    veast,vnorth,vz = ground_velocity.y,ground_velocity.x,ground_velocity.z
    vx = vnorth*math.sin(road_angle)+veast*math.cos(road_angle)
    vy = vnorth*math.cos(road_angle)-veast*math.sin(road_angle)
    air_velocity = vehicle.airspeed
    weast,wnorth,wz = air_velocity.y,air_velocity.x,air_velocity.z
    wx = wnorth*math.sin(road_angle)+weast*math.cos(road_angle)
    wy = wnorth*math.cos(road_angle)-weast*math.sin(road_angle)
    
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

def save_and_plan(pitch,roll,yaw,location,xdist,ydist,radius):
    print("Saving process has been started...")
    xreal,yreal=(4*xdist/radius)/math.cos(roll),(4*ydist/radius)/math.cos(pitch)
    try:
        xtarget,ytarget=round(math.sqrt(xreal**2+yreal**2)*math.cos(-yaw+math.atan(yreal/xreal)),2),round(math.sqrt(xreal**2+yreal**2)*math.sin(-yaw+math.atan(yreal/xreal)),2)
    except:
        xtarget,ytarget=round(math.sqrt(xreal**2+yreal**2)*math.cos(-yaw+math.pi/2),2),round(math.sqrt(xreal**2+yreal**2)*math.sin(-yaw+math.pi/2),2)
    dist_target=round(math.sqrt(xtarget**2+ytarget**2),2)
    print("Saving process has been finished!, distance to target is {} m".format(dist_target))
    red_carpet_loc = get_location_meters(location,xtarget,ytarget)
    return red_carpet_loc

def telemetry_sender(vehicle,telemetry_count):
    print("-----------------------------------------------------")
    print("Telemetry number: %s" % telemetry_count)
    print("Mode: %s" % vehicle.mode)
    print("Attitude: %s" % vehicle.attitude)
    print("Velocity: %s" % vehicle.velocity)
    print("Global Location (relative altitude) %s" % vehicle.location.global_relative_frame)
    print("Distance from home %s" % get_distance_meters(vehicle.home_location, vehicle.location.global_relative_frame))
    print("-----------------------------------------------------")

def switch_mode(vehicle,mode):
    print("Vehicle mode is being switched to {}...".format(mode))
    vehicle.mode = VehicleMode(mode)
    while not vehicle.mode.name==mode:
        print("Waiting for mode change")
        time.sleep(0.5)
    print("New mode: %s" % vehicle.mode)

def check_if_near(vehicle,carpet,road_angle):
    eas_dist_meters = (vehicle.location.global_relative_frame.east - carpet.east)*1.113195e5
    north_dist_meters = (vehicle.location_global_relative_frame.north - carpet.north)*1.113195e5
    xdist = north_dist_meters*math.sin(road_angle)+eas_dist_meters*math.cos(road_angle)
    ydist = north_dist_meters*math.cos(road_angle)-eas_dist_meters*math.sin(road_angle)
    if math.abs(xdist)<3 and math.abs(ydist)<20:
        return True
    else:
        return False