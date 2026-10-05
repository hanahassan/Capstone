import os
import time
import math
import myactuator_rmd_py as rmd
from init_canports import init_canports

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

motor_IDs = list(range(1, 7)) + list(range(10, 18))
# motor_IDs = [11, 13, 15, 17]

error_no = 0

# Read actuator startup positions for later calculaltions of zero offset, and set position planning accelerations to zero for dynamic tracking.
startup_pos = {}
pos_accel = rmd.actuator_state.AccelerationType(0)
pos_decel = rmd.actuator_state.AccelerationType(1)
for id in motor_IDs:
        startup_pos[id] = motor[id].getMultiTurnAngle()
        motor[id].setAcceleration(0, pos_accel)
        motor[id].setAcceleration(0, pos_decel)


# maximum joint position from start of each motor (id) in degrees.
max_angle = {
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
    16: -100,
    17: 0
}

freq = 0.5 # Hz
t = 0
dt = 0.1

def sin_pos(t, id):
    return startup_pos[id] + max_angle[id]/2 * (1 - math.cos(2 * math.pi * freq * t))

# Sinusoidal path following until KeyboardInterrupt (^C).
try:
    while True:
        for id in motor_IDs:
            try:
                motor[id].sendPositionAbsoluteSetpoint(sin_pos(t, id), 500)
            except rmd.can.BusError:
                error_no += 1
                print("Bus Error: " + str(error_no) + " Motor ID =" + str(id) + "\n" )
        time.sleep(dt)
        t += dt
except KeyboardInterrupt:
    pass

time.sleep(2)

# Reset accelerations and return to startup positions for shutdown.
for id in motor_IDs:
    try:
        motor[id].setAcceleration(10000, pos_accel)
    except rmd.ProtocolException as exc:
        print(exc)
    except rmd.can.BusError:
        error_no += 1
        print("Bus Error resetting acceleration, motor ID " + str(id) + ": " + str(error_no) + "\n")
    try:
        motor[id].setAcceleration(10000, pos_decel)
    except rmd.ProtocolException as exc:
        print(exc)
    except rmd.can.BusError:
        error_no += 1
        print("Bus Error resetting deceleration, motor ID " + str(id) + ": " + str(error_no) + "\n")
    try:
        motor[id].sendPositionAbsoluteSetpoint(startup_pos[id], 50)
    except rmd.ProtocolException as exc:
        print(exc)
    except rmd.can.BusError:
        error_no += 1
        print("Bus Error returning to startup pos. motor ID" + str(id) + ": " + str(error_no) + "\n")