import argparse, math, time, socket

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException

def connect_drone():
    parser = argparse.ArgumentParser(description="commands",
                                     usage="python connect_copter.py --addr tcp:127.0.0.1:5762",)
    parser.add_argument("--addr")
    args = parser.parse_args()

    addr_str = args.addr
    print("The parsed address to connect: {0}".format(addr_str))
    if not addr_str:
        addr_str = "tcp:127.0.0.1:5762"
        print("Missing input connection address, setting it to: {0}".format(addr_str))

    print("Connecting..")
    # drone = connect(addr_str, wait_ready=True)
    drone = connect(addr_str)
    return drone

if __name__ == "__main__":
    drone = connect_drone()

