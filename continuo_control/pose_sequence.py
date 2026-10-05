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
    3: 0,
    4: 0,
    5: 0,
    6: 0,
    10: 0,
    11: 0,
    12: 0,
    13: 0,
    14: 0,
    15: 0,
    16: 0,
    17: 0
}
# Legs more down.
pose_moredown = {
    1: 0,
    2: 0,
    3: 20,
    4: -20,
    5: -30,
    6: 30,
    10: 0,
    11: 0,
    12: 20,
    13: -20,
    14: -20,
    15: 20,
    16: -60,
    17: 60
}
# Legs to right.
pose_right = {
    1: 0,
    2: 0,
    3: 0,
    4: 0,
    5: 0,
    6: 0,
    10: 0,
    11: 0,
    12: 0,
    13: 0,
    14: 0,
    15: 0,
    16: 0,
    17: 0
}
# Legs to both sides.
pose_sides = {
    1: 60,
    2: -60,
    3: -45,
    4: 45,
    5: 90,
    6: -90,
    10: 50,
    11: -50,
    12: -30,
    13: 30,
    14: 45,
    15: -45,
    16: -60,
    17: 60
}
# Forward and back.
pose_stretch = {
    1: 0,
    2: 0,
    3: -175,
    4: 175,
    5: 175,
    6: -175,
    10: 0,
    11: 0,
    12: -175,
    13: 175,
    14: 175,
    15: -175,
    16: -175,
    17: 175
}
# Legs up.
pose_up = {
    1: 0,
    2: 0,
    3: 45,
    4: -45,
    5: -90,
    6: 90,
    10: 0,
    11: 0,
    12: 30,
    13: -30,
    14: -45,
    15: 45,
    16: 60,
    17: -60
}
# Reach up.
pose_reach = {
    1: 0,
    2: 0,
    3: -230,
    4: 230,
    5: 175,
    6: -175,
    10: 0,
    11: 0,
    12: -30,
    13: 30,
    14: 45,
    15: -45,
    16: -60,
    17: 60
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
    # Set position planning accelerations to 5000 for single commands.
    try:
        motor[id].setAcceleration(0, pos_accel)
    except rmd.can.BusError:
        error_no += 1
        print("Bus Error setting acceleration, motor ID " + str(id) + ": ") 
    try:
        motor[id].setAcceleration(0, pos_decel)
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

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Legs down more? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Legs more down pose.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id] + pose_moredown[id], 100)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Legs right? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Legs right pose.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id] + pose_right[id], 50)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Return to starting pose? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Return to starting pose.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id], 50)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Legs to both sides? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Legs to both sides.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id] + pose_sides[id], 50)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Return to starting pose? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Return to starting pose.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id], 150)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Stretch? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Front right forward and hind left back.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id] + pose_stretch[id], 50)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Return to starting pose? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Return to starting pose.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id], 50)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Legs up? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Legs up.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id] + pose_up[id], 50)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Return to starting pose? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Return to starting pose.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id], 50)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Reach up? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Reach up.
for id in [1, 2, 3, 4, 10, 11, 12, 13, 14, 15, 16, 17]:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id] + pose_reach[id], 50)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")
# Wait for upper arm link to be in position.
time.sleep(4)
for id in [5, 6]:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id] + pose_reach[id], 50)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Return to starting pose? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Return to starting pose.
for id in motor_IDs:
    flag = None
    while flag == None:
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id], 50)
            flag = 1
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: " + str(error_no) + "\n")