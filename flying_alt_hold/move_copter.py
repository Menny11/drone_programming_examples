"""
Usage: Start Mission Planner -> SITL. select hex copter model and add starting parameters
(Extra command line):

--home=50.450739,30.461242,0,0

press on copter icon to start sitl simulation.
Execute this script:

python move_copter.py --addr tcp:127.0.0.1:5762

"""

import time, math
from dronekit import LocationGlobalRelative

from connect_copter import connect_drone
from manage_copter import *
from utils import *

target_altitude = 100
target_coords = [50.443326, 30.448078]
target_lat = target_coords[0]
target_lon = target_coords[1]
target_yaw = 350
drone_speed = 10.0

drone = connect_drone()

slowing_distance = 3
def get_thrust(altitude):
    if altitude < slowing_distance or target_altitude - altitude <= slowing_distance:
        return 700 # for slow start and end
    return 1000

def takeoff(altitude):
    print("Start to takeoff")
    thrust = 700
    while True:
        alt = drone.location.global_relative_frame.alt
        thrust = get_thrust(alt)
        print("alt: %f, thrust: %f"%(alt, thrust))

        if alt > altitude - 1.0:
            print("Drone reached target altitude!")
            set_manual_control(drone, 0, 0, 0, 500) # to discard thrust
            break

        set_manual_control(drone, 0, 0, 0, thrust)
        time.sleep(0.5)

def delay(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        set_manual_control(drone, 0, 0, 0, 500) # to keep the altitude
        time.sleep(0.1)

def yaw_diff(curr_yaw, target_yaw):
    diff = abs(target_yaw - curr_yaw)
    if diff > 180:
        diff = 360 - diff
    return diff


# ************ FLYING **************

arm_and_set_mode(drone, "ALT_HOLD")
takeoff(target_altitude)

target_pos = LocationGlobalRelative(target_lat, target_lon, target_altitude)
timescale = 0.5
axis_max_value = 1000
distance_leveling = 2
distance_slowdown = 30
thrust = 500

while True:
    pos = drone.location.global_relative_frame
    dist = distance(pos, target_pos)

    print("Current distance: %f"%dist)
    if dist <= distance_leveling:
        print("Reached destination!")
        set_manual_control(drone=drone, thrust=thrust)
        break

    if dist <= distance_slowdown:
        axis_max_value = 200

    vx, vy = direction(pos, target_pos) # normalized vector in the direction of the target
    print("vx: %f, vy: %f"%(vx, vy))

    yaw = drone.attitude.yaw;
    vx, vy = rotate(yaw, vx, vy)
    print("rotated, vx: %f, vy: %f"%(vx, vy))

    vx, vy = scale_axis(axis_max_value, vx, vy)
    print("scaled, vx: %f, vy: %f"%(vx, vy))

    set_manual_control(drone=drone, pitch=vx, roll=vy, thrust=thrust)
    time.sleep(timescale)

delay(3)

print("Setting yaw..")
yaw = 100
if target_yaw > 180:
    yaw = -100 # to rotate counterclockwise

stop_yaw_diff = 1
slowdown_yaw_diff = 20
slowdown_yaw = 50
while True:
    curr_yaw = (drone.attitude.yaw * 180/math.pi + 360)%360;
    print("Current yaw: %f, target yaw: %f"%(curr_yaw, target_yaw))
    diff = yaw_diff(curr_yaw, target_yaw)
    if diff <= stop_yaw_diff:
        print("Reached yaw!")
        set_manual_control(drone=drone, yaw=0, thrust=thrust)
        break

    if diff <= slowdown_yaw_diff:
        if yaw > 0:
            yaw = slowdown_yaw
        else:
            yaw = -slowdown_yaw

    set_manual_control(drone=drone, yaw=yaw, thrust=thrust)
    time.sleep(0.2)

delay(5)

drone.close()




