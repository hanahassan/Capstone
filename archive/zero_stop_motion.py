import os
import time
import math
import myactuator_rmd_py as rmd
from init_canports import init_canports

# Motion control gains, 12-bit int 0-4095.
kp = 82 # converts from 0-4095 to 0-500 (82 = 10).
kd = 819 # converts from 0-4095 to 0-5 (819 = 1).

init_canports()

## Setup CAN channels and motors.
# Dict for single-motor command objects.
motor = {}
# Dict for motion-mode command objects.
motion_motor = {}

# Front right leg.
can_fr = rmd.CanDriver("can0")
fr_shoulder = motor[1] = rmd.ActuatorInterface(can_fr, 1)
fr_hip = motor[3] = rmd.ActuatorInterface(can_fr, 3)
fr_knee = motor[5] = rmd.ActuatorInterface(can_fr, 5)

motion_can_fr = rmd.MotionCanDriver("can0")
motion_motor[1] = rmd.MotionActuatorInterface(motion_can_fr, 1)
motion_motor[3] = rmd.MotionActuatorInterface(motion_can_fr, 3)
motion_motor[5] = rmd.MotionActuatorInterface(motion_can_fr, 5)

print("Front right leg motor ID's set.")

# Front left leg.
can_fl = rmd.CanDriver("can1")
fl_shoulder = motor[2] = rmd.ActuatorInterface(can_fl, 2)
fl_hip = motor[4] = rmd.ActuatorInterface(can_fl, 4)
fl_knee = motor[6] = rmd.ActuatorInterface(can_fl, 6)

motion_can_fl = rmd.MotionCanDriver("can1")
motion_motor[2] = rmd.MotionActuatorInterface(motion_can_fl, 2)
motion_motor[4] = rmd.MotionActuatorInterface(motion_can_fl, 4)
motion_motor[6] = rmd.MotionActuatorInterface(motion_can_fl, 6)

print("Front left leg motor ID's set.")

# Hind right leg.
can_hr = rmd.CanDriver("can2")
hr_shoulder = motor[11] = rmd.ActuatorInterface(can_hr, 11)
hr_hip = motor[13] = rmd.ActuatorInterface(can_hr, 13)
hr_knee = motor[15] = rmd.ActuatorInterface(can_hr, 15)
hr_ankle = motor[17] = rmd.ActuatorInterface(can_hr, 17)

motion_can_hr = rmd.MotionCanDriver("can2")
motion_motor[11] = rmd.MotionActuatorInterface(motion_can_hr, 11)
motion_motor[13] = rmd.MotionActuatorInterface(motion_can_hr, 13)
motion_motor[15] = rmd.MotionActuatorInterface(motion_can_hr, 15)
motion_motor[17] = rmd.MotionActuatorInterface(motion_can_hr, 17)

print("Hind right leg motor ID's set.")

# Hind left leg.
can_hl = rmd.CanDriver("can3")
hl_shoulder = motor[10] = rmd.ActuatorInterface(can_hl, 10)
hl_hip = motor[12] = rmd.ActuatorInterface(can_hl, 12)
hl_knee = motor[14] = rmd.ActuatorInterface(can_hl, 14)
hl_ankle = motor[16] = rmd.ActuatorInterface(can_hl, 16)

motion_can_hl = rmd.MotionCanDriver("can3")
motion_motor[10] = rmd.MotionActuatorInterface(motion_can_hl, 10)
motion_motor[12] = rmd.MotionActuatorInterface(motion_can_hl, 12)
motion_motor[14] = rmd.MotionActuatorInterface(motion_can_hl, 14)
motion_motor[16] = rmd.MotionActuatorInterface(motion_can_hl, 16)

print("Hind left leg motor ID's set.")

motor_IDs = list(range(1, 7)) + list(range(10, 18))
# motor_IDs = list(range(1, 3)) + list(range(4, 7)) + list(range(10, 18))

for id in motor_IDs:
    # Reset encoder zero and reset motor.
    while True:
        try:
            motor[id].setCurrentPositionAsEncoderZero()
            break
        except rmd.can.BusError as exc:
            print(exc, "zeroing:", id)
    while True:
        try:
            motor[id].reset()
            break
        except rmd.can.BusError as exc:
            print(exc, "resetting:", id)

# Wait after reset.
time.sleep(1)

startup_pos = {}
startup_pos_rad = {}

for id in motor_IDs:
    while True:
        try:
            startup_pos[id] = motor[id].getMultiTurnAngle()
            break
        except rmd.can.BusError as exc:
            print(exc, "reading position:", id)
    startup_pos_rad[id] = math.radians(startup_pos[id])

    while True:
        try:
            motion_motor[id].motionModeControl(startup_pos_rad[id], 0, kp, kd, 0)
            break
        except rmd.can.BusError as exc:
            print(exc, "setting position:", id)