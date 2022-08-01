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
    dEast = math.cos(-road_angle)*dx - math.sin(-road_angle)*dy
    dNorth = math.sin(-road_angle)*dx + math.cos(-road_angle)*dy
    dLat = dNorth / earth_r
    dLon = dEast / (earth_r*math.cos(math.pi*original_location.lat/180))
    newLat = original_location.lat + (dLat * 180 / math.pi)
    newLon = original_location.lon + (dLon * 180 /math.pi)
    return LocationGlobalRelative(newLat,newLon,original_location.alt)

def get_distance_meters(aLoc1,aLoc2):
    dlat = aLoc2.lat - aLoc1.lat
    dlon = aLoc2.lon - aLoc1.lon
    return math.sqrt((dlat**2)+(dlon**2))*1.113195e5
    
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

def telemetry_sender(vehicle,telemetry_count):
    print("-----------------------------------------------------")
    print("Telemetry number: %s" % telemetry_count)
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
    xdist = math.cos(-road_angle)*eas_dist_meters - math.sin(-road_angle)*north_dist_meters
    ydist = math.sin(-road_angle)*eas_dist_meters + math.cos(-road_angle)*north_dist_meters
    if math.abs(xdist)<3 and math.abs(ydist)<20:
        return True
    else:
        return False
def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """	
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    return math.atan(off_x/off_y)
    '''bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;'''
    
def mission_updater_new(vehicle,carpet):
    # generate waypoints for the carpet
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    print("Commands have been downloaded!")
    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)
    lat_cur,lon_cur,alt_cur=missionlist[3].x,missionlist[3].y,missionlist[3].z 
    waypointcurrent=LocationGlobalRelative(lat_cur,lon_cur,alt_cur)
    angle = get_bearing(waypointcurrent,carpet)
    waypoints = []
    for i in [0,10]:
        waypoints.append(get_location_meters_for_road(carpet,0,i,angle))
    commands = []
    for wp in waypoints:
        commands.append(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,wp.lat,wp.lon,wp.alt))
    for i in [4,5]:
        missionlist[i] = commands[i-4]
    print("The new waypoints are inserted!")
    print("Now, upload is starting...")
    #upload the new list
    cmds.clear()
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()
    print("Mission update has been finished!")