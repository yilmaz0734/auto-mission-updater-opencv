'''start_for_search = time.time()
while True:
    if time.time() - start_for_search > 15:
        print("Target not found")
        break
    yaw_holwer = attitude.yaw

while True:
    print("After ten seconds, red carpet checking is started.")
    if check_if_near(iha,red_zone_location,road_angle):
        print("Circle has been completed, there are 30 meters to the red carpet.")
        current_yawer = {}
        start_for_yawer = time.time()
        while True:
            current_yawer.add(attitude.yaw)
            if time.time()-start_for_yawer > 1.0:
                break
        current_yaw_average = sum(current_yawer)/len(current_yawer)
        mission_updater(iha,red_zone_location,current_yaw_average)
        break'''