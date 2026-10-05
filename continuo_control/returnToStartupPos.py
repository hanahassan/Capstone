import os
import time
import math
import myactuator_rmd_py as rmd
from Continuo_init_canports import init_canports
from Continuo_init_motors import init_motors


## Define variables:
error_no = 0

motor_IDs = list(range(1, 7)) + list(range(10, 18))

## Pose joint angles (relative to starting, in degrees).
# Legs down.
pose_down = {
    1: 0,
    2: 0,
    3: 66.5,
    4: -66.5,
    5: -107.1,
    6: 107.1,
    10: 0,
    11: 0,
    12: 50.1,
    13: -50.1,
    14: -84,
    15: 84,
    16: 81.3,
    17: -81.3
}


proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Confirm ContinuO in startup pose and suspended (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

init_canports()

## Setup CAN channels and motors.
motor = {}

# Front right leg.
can_fr = rmd.CanDriver("can0")
fr_shoulder = motor[1] = rmd.ActuatorInterface(can_fr, 1)
fr_hip = motor[3] = rmd.ActuatorInterface(can_fr, 3)
fr_knee = motor[5] = rmd.ActuatorInterface(can_fr, 5)
print("Front right leg motor ID's set.")

# Front left leg.
can_fl = rmd.CanDriver("can1")
fl_shoulder = motor[2] = rmd.ActuatorInterface(can_fl, 2)
fl_hip = motor[4] = rmd.ActuatorInterface(can_fl, 4)
fl_knee = motor[6] = rmd.ActuatorInterface(can_fl, 6)
print("Front left leg motor ID's set.")

# Hind right leg.
can_hr = rmd.CanDriver("can2")
hr_shoulder = motor[11] = rmd.ActuatorInterface(can_hr, 11)
hr_hip = motor[13] = rmd.ActuatorInterface(can_hr, 13)
hr_knee = motor[15] = rmd.ActuatorInterface(can_hr, 15)
hr_ankle = motor[17] = rmd.ActuatorInterface(can_hr, 17)
print("Hind right leg motor ID's set.")

# Hind left leg.
can_hl = rmd.CanDriver("can3")
hl_shoulder = motor[10] = rmd.ActuatorInterface(can_hl, 10)
hl_hip = motor[12] = rmd.ActuatorInterface(can_hl, 12)
hl_knee = motor[14] = rmd.ActuatorInterface(can_hl, 14)
hl_ankle = motor[16] = rmd.ActuatorInterface(can_hl, 16)
print("Hind left leg motor ID's set.")

startup_pos = {}
pos_accel = rmd.actuator_state.AccelerationType(0)
pos_decel = rmd.actuator_state.AccelerationType(1)
for id in motor_IDs:
    # Read actuator startup positions for later calculaltions of offset.
    startup_pos[id] = motor[id].getMultiTurnAngle()
    # Set position planning accelerations to 500 for single commands.
    try:
        motor[id].setAcceleration(500, pos_accel)
    except rmd.can.BusError:
        error_no += 1
        print("Bus Error setting acceleration, motor ID " + str(id) + ": ") 
    try:
        motor[id].setAcceleration(500, pos_decel)
    except rmd.can.BusError:
        error_no += 1
        print("Bus Error setting acceleration, motor ID " + str(id) + ": ")

# Legs down pose.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id] + pose_down[id], 100)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

