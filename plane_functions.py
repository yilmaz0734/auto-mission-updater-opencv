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
    lat_cur,lon_cur,alt_cur=missionlist[2].x,missionlist[2].y,10
    waypointcurrent=LocationGlobalRelative(lat_cur,lon_cur,alt_cur)
    angle = get_bearing(waypointcurrent,carpet)
    waypoints = []
    for i in [0,40]:
        waypoints.append(get_location_meters_for_road(carpet,0,i,angle))
    commands = []
    for wp in waypoints:
        commands.append(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,wp.lat,wp.lon,wp.alt))
    for i in [3,4]:
        missionlist[i] = commands[i-3]
    print("The new waypoints are inserted!")
    print("Now, upload is starting...")
    #upload the new list
    cmds.clear()
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()
    print("Mission update has been finished!")