import time
from dronekit import VehicleMode
from pymavlink import mavutil


def change_mode(drone, mode_str):
    while not drone.mode == VehicleMode(mode_str):
        drone.mode = VehicleMode(mode_str)
        time.sleep(0.5)

def arm_and_set_mode(drone, mode_str=None):
    while not drone.is_armable:
        print("Waiting for drone to be ready to arm..")
        time.sleep(1)

    if mode_str != None:
        change_mode(drone, mode_str)

    drone.armed = True
    while not drone.armed:
        time.sleep(1)

def set_manual_control(drone, pitch=0, roll=0, yaw=0, thrust=0):
    # send manual control params
    # values normilized in -1000 to 1000 range
    print("Sending pitch:%f, roll:%f, yaw:%f, thrust:%f"%(pitch, roll, yaw, thrust))
    msg = drone.message_factory.manual_control_encode(
        1, # target system
        pitch,
        roll,
        thrust,
        yaw,
        0) # bitfield for other joystick buttons
    drone.send_mavlink(msg)
    drone.flush()

# ***************** FUNCTIONS FOR GUIDED PILOTING *********************

def guided_takeoff(drone, altitude):
    print("Start to takeoff")
    drone.simple_takeoff(altitude);
    while True:
        alt = drone.location.global_relative_frame.alt
        if alt > altitude - 1.0:
            print("Drone reached target altitude!")
            break

        time.sleep(1)

def set_guided_velocity(drone, vx, vy, vz):
    # use in the guided mode of the drone
    msg = drone.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, #-- move in global coords
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)


    drone.send_mavlink(msg)
    drone.flush()

def set_guided_yaw(drone, yaw_degr, is_relative=False):

    relative = 0 # absolute
    if is_relative:
        relative = 1 #yaw relative to direction of movement

    msg = drone.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,
        yaw_degr,    # yaw in degrees
        0,          # yaw speed deg/s
        1,          # direction -1 ccw, 1 cw
        relative,
        0, 0, 0)

    drone.send_mavlink(msg)
    drone.flush()