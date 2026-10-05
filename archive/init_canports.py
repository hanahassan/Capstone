import os
import time

def init_canports():
    'Bring up network devices for each CAN channel at bitrate 1 Mbps.'   
    
    print('Bringing up CAN0...')
    os.system("sudo ip link set can0 up type can bitrate 1000000")
    time.sleep(0.1) 

    print('Bringing up CAN1...')
    os.system("sudo ip link set can1 up type can bitrate 1000000")
    time.sleep(0.1) 

    print('Bringing up CAN2...')
    os.system("sudo ip link set can2 up type can bitrate 1000000")
    time.sleep(0.1) 

    print('Bringing up CAN3...')
    os.system("sudo ip link set can3 up type can bitrate 1000000")
    time.sleep(0.1)

if __name__ == '__main__':
    init_canports()