import os
import time
import math
import myactuator_rmd_py as rmd
from init_canports import init_canports

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

# Read actuator startup positions for later calculaltions of zero offset.
startup_pos = {}
for id in motor_IDs:
    startup_pos[id] = motor[id].getMultiTurnAngle()

t = 0
dt = 0.01

def sin_vel(t):
    v_max = 50 # degree/s
    freq = 0.5 # Hz
    return v_max * math.sin(2 * math.pi * freq * t)

# print(time.time())

while t <= 10:
    fr_knee.sendVelocitySetpoint(sin_vel(t))
    fr_hip.sendVelocitySetpoint(sin_vel(t))
    
    fl_knee.sendVelocitySetpoint(sin_vel(t))
    fl_hip.sendVelocitySetpoint(sin_vel(t))

    time.sleep(dt)
    t += dt
    

fr_knee.stopMotor()
fr_hip.stopMotor()

fl_knee.stopMotor()
fl_hip.stopMotor()

# print(time.time())

# filename = 'Data_joint_positions_0.01_comb.txt'
# filename1 = 'Data_joint_velocities_0.01_comb.txt'

# def read_line_as_array(filename, line_number):
#     try:
#         with open(filename, 'r') as file:
#             current_line = 0
#             for line in file:
#                 current_line += 1
#                 if current_line == line_number:
#                     # Convert the line into an array of doubles
#                     return [float(x) for x in line.split()]
#             # If the desired line doesn't exist, return an empty array
#             return []
#     except FileNotFoundError:
#         print(f"File '{filename}' not found.")
#         return []
    
# def get_action(t):
#   line_number = t
# #   desired_position = read_line_as_array(filename, line_number)
# #   desired_velocity = read_line_as_array(filename1, line_number)
#   print(desired_position)
#   print(desired_velocity)

# #for i in range(3600):   ##3960
# for i in range(12968):   ##3960
#     action = get_action(i+1)
#     # robot.step(action)
#     time.sleep(0.01)
