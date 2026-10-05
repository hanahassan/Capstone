import os
import time
import math
import myactuator_rmd_py as rmd
from init_canports import init_canports

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Confirm ContinuO in startup pose on floor (y/n):\n')
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

startup_pos = {}
pos_accel = rmd.actuator_state.AccelerationType(0)
pos_decel = rmd.actuator_state.AccelerationType(1)
for id in motor_IDs:
        # Read actuator startup positions for later calculaltions of zero offset.
        startup_pos[id] = motor[id].getMultiTurnAngle()
        # Set position planning accelerations to 10000 for single commands.
        motor[id].setAcceleration(10000, pos_accel)
        motor[id].setAcceleration(10000, pos_decel)
        # All motors hold position.
        motor[id].stopMotor()

motor[3].sendPositionAbsoluteSetpoint(startup_pos[3] + 48.5, 50)
motor[4].sendPositionAbsoluteSetpoint(startup_pos[4] - 48.5, 50)
motor[14].sendPositionAbsoluteSetpoint(startup_pos[14] - 23, 50)
motor[15].sendPositionAbsoluteSetpoint(startup_pos[15] + 23, 50)

time.sleep(1)

motor[5].sendPositionAbsoluteSetpoint(startup_pos[5] + 10, 50)
motor[6].sendPositionAbsoluteSetpoint(startup_pos[6] - 10, 50)
motor[16].sendPositionAbsoluteSetpoint(startup_pos[16] - 30, 50)
motor[17].sendPositionAbsoluteSetpoint(startup_pos[17] + 30, 50)

time.sleep(1)

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Confirm ContinuO feet in position (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

squat_pos = {}

for id in motor_IDs:
        squat_pos[id] = motor[id].getMultiTurnAngle()
        # Set position planning accelerations to 0 for dynamic tracking.
        motor[id].setAcceleration(0, pos_accel)
        motor[id].setAcceleration(0, pos_decel)

# maximum joint position from start of each motor (id) in degrees.
max_angle = {
    1: 0,
    2: 0,
    3: -90,
    4: 90,
    5: 75,
    6: -75,
    10: 0,
    11: 0,
    12: -30,
    13: 30,
    14: 70,
    15: -70,
    16: -40,
    17: 40
}

freq = 0.5 # Hz
t = 0
dt = 0.002

def sin_pos(t, id):
    return squat_pos[id] + max_angle[id]/2 * (1 - math.cos(2 * math.pi * freq * t))

# Sinusoidal path following until KeyboardInterrupt (^C).
try:
    while t < 0.5/freq:
        for id in motor_IDs:
            try:
                motor[id].sendPositionAbsoluteSetpoint(sin_pos(t, id), 500)
            except rmd.can.BusError:
                error_no += 1
                print("Bus Error: " + str(error_no) + "\n")
        time.sleep(dt)
        t += dt
except KeyboardInterrupt:
    pass

time.sleep(2)

# # Reset accelerations and return to startup positions for shutdown.
# for id in motor_IDs:
#     try:
#         motor[id].setAcceleration(10000, pos_accel)
#     except rmd.ProtocolException as exc:
#         print(exc)
#     except rmd.can.BusError:
#         error_no += 1
#         print("Bus Error resetting acceleration, motor ID " + str(id) + ": " + str(error_no) + "\n")
#     try:
#         motor[id].setAcceleration(10000, pos_decel)
#     except rmd.ProtocolException as exc:
#         print(exc)
#     except rmd.can.BusError:
#         error_no += 1
#         print("Bus Error resetting deceleration, motor ID " + str(id) + ": " + str(error_no) + "\n")
#     try:
#         motor[id].sendPositionAbsoluteSetpoint(startup_pos[id], 50)
#     except rmd.ProtocolException as exc:
#         print(exc)
#     except rmd.can.BusError:
#         error_no += 1
#         print("Bus Error returning to startup pos. motor ID" + str(id) + ": " + str(error_no) + "\n")

# 1: 0.15000009536743164
# 2: -0.1400012969970703
# 3: 48.6300048828125
# 4: -52.8699951171875
# 5: 13.850006103515625
# 6: -14.85003662109375
# 10: 0.0
# 11: -0.10000228881835938
# 12: -0.06999969482421875
# 13: -0.010000228881835938
# 14: -23.00994873046875
# 15: 24.010009765625
# 16: -30.479995727539062
# 17: 31.8900146484375