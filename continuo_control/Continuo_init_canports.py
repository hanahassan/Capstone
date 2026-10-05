import os
import time

def init_canports():

    'Bring up network devices for each CAN channel at bitrate 1 Mbps.' 

    for can in ["can0", "can1", "can2", "can3"]:
        os.system(f"sudo ip link set {can} down")
        time.sleep(0.1)
        os.system(f"sudo ip link set {can} up type can bitrate 1000000")
        time.sleep(0.1)

if __name__ == '__main__':
    init_canports()