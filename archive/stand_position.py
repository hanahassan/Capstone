import os
import time
import math
import myactuator_rmd_py as rmd
from init_canports import init_canports

## Define variables.

file_pos = 'Data_joint_positions_standing_2ms.txt'

motor_IDs = list(range(1, 7)) + list(range(10, 18))
# motor_IDs = [1, 3, 5]

error_no = 0

t = 0
dt = 0.002

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Confirm ContinuO in startup pose and suspended (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

init_canports()

## Setup CAN channels and motors.
# Single-motor commands
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

def read_line_as_array(filename, line_number):
    try:
        with open(filename, 'r') as file:
            current_line = 0
            for line in file:
                current_line += 1
                if current_line == line_number:
                    # Convert the line into an array of doubles
                    return [float(x) for x in line.split()]
            # If the desired line doesn't exist, return an empty array
            return []
    except FileNotFoundError:
        print(f"File '{filename}' not found.")
        return []
    
def model_to_motor_pos(motor_id, model_pos):
    '''Convert from model to motor position and direction (degrees).'''
    if motor_id == 1 or motor_id == 2:
        return round(-math.degrees(model_pos), 3)
    elif motor_id == 3:
        return round(math.degrees(model_pos) - (96.790), 3)
    elif motor_id == 4:
        return round(-math.degrees(model_pos) + (96.720), 3)
    elif motor_id == 5:
        return round(math.degrees(model_pos) - (-193.090), 3)
    elif motor_id == 6:
        return round(-math.degrees(model_pos) + (-193.290), 3)
    elif motor_id == 10 or motor_id == 11:
        return round(math.degrees(model_pos), 3)
    elif motor_id == 12:
        return round(-math.degrees(model_pos) + (-80.650), 3)
    elif motor_id == 13:
        return round(math.degrees(model_pos) - (-81.890), 3)
    elif motor_id == 14:
        return round(-math.degrees(model_pos) + (182.540), 3)
    elif motor_id == 15:
        return round(math.degrees(model_pos) - (182.940), 3)
    elif motor_id == 16:
        return round(-math.degrees(model_pos) + (-199.990), 3)
    elif motor_id == 17:
        return round(math.degrees(model_pos) - (-200.610), 3)

# Read initial position of model files and convert to motor ID dictionary.
ready_pos = {}
model_ready_pos = read_line_as_array(file_pos, 1)

# Front right leg.
ready_pos[1] = model_to_motor_pos(1, model_ready_pos[0])
ready_pos[3] = model_to_motor_pos(3, model_ready_pos[1])
ready_pos[5] = model_to_motor_pos(5, model_ready_pos[2])

# Front left leg.
ready_pos[2] = model_to_motor_pos(2, model_ready_pos[3])
ready_pos[4] = model_to_motor_pos(4, model_ready_pos[4])
ready_pos[6] = model_to_motor_pos(6, model_ready_pos[5])

# Hind right leg.
ready_pos[11] = model_to_motor_pos(11, model_ready_pos[6])
ready_pos[13] = model_to_motor_pos(13, model_ready_pos[7])
ready_pos[15] = model_to_motor_pos(15, model_ready_pos[8])
ready_pos[17] = model_to_motor_pos(17, model_ready_pos[9])

# Hind left leg.
ready_pos[10] = model_to_motor_pos(10, model_ready_pos[10])
ready_pos[12] = model_to_motor_pos(12, model_ready_pos[11])
ready_pos[14] = model_to_motor_pos(14, model_ready_pos[12])
ready_pos[16] = model_to_motor_pos(16, model_ready_pos[13])

startup_pos = {}
pos_accel = rmd.actuator_state.AccelerationType(0)
pos_decel = rmd.actuator_state.AccelerationType(1)
for id in motor_IDs:
        # Read actuator startup positions for later calculations of offset (degrees).
        startup_pos[id] = motor[id].getMultiTurnAngle()
        # Set position planning accelerations to 5000 for single commands.
        motor[id].setAcceleration(5000, pos_accel)
        motor[id].setAcceleration(5000, pos_decel)
        # Assume initial pose from file "ready pose".
        flag = None
        while flag == None:
            try:
                motor[id].sendPositionAbsoluteSetpoint(startup_pos[id] + ready_pos[id], 50)
                flag = 1
            except rmd.can.BusError:
                error_no += 1
                print("Bus Error: " + str(error_no) + "\n")

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Confirm ContinuO feet in position (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

for id in motor_IDs:
    # Set position planning accelerations to 0 for dynamic tracking.
    motor[id].setAcceleration(0, pos_accel)
    motor[id].setAcceleration(0, pos_decel) 

desired_pos = {}

try:
    while t < 1356:
        line_number = t+1
        model_desired_pos = read_line_as_array(file_pos, line_number)

        # Front right leg.
        desired_pos[1] = model_to_motor_pos(1, model_desired_pos[0])
        desired_pos[3] = model_to_motor_pos(3, model_desired_pos[1])
        desired_pos[5] = model_to_motor_pos(5, model_desired_pos[2])

        # Front left leg.
        desired_pos[2] = model_to_motor_pos(2, model_desired_pos[3])
        desired_pos[4] = model_to_motor_pos(4, model_desired_pos[4])
        desired_pos[6] = model_to_motor_pos(6, model_desired_pos[5])

        # Hind right leg.
        desired_pos[11] = model_to_motor_pos(11, model_desired_pos[6])
        desired_pos[13] = model_to_motor_pos(13, model_desired_pos[7])
        desired_pos[15] = model_to_motor_pos(15, model_desired_pos[8])
        desired_pos[17] = model_to_motor_pos(17, model_desired_pos[9])

        # Hind left leg.
        desired_pos[10] = model_to_motor_pos(10, model_desired_pos[10])
        desired_pos[12] = model_to_motor_pos(12, model_desired_pos[11])
        desired_pos[14] = model_to_motor_pos(14, model_desired_pos[12])
        desired_pos[16] = model_to_motor_pos(16, model_desired_pos[13])

        print("line " + str(line_number) + ":\n", 
            "p_des:", desired_pos)

        # for id in motor_IDs:
        for id in motor_IDs:
            try:
                motor[id].sendPositionAbsoluteSetpoint(startup_pos[id] + desired_pos[id], 500)
            except rmd.can.BusError:
                error_no += 1
                print("Bus Error: ", error_no)

        t += 1
        time.sleep(dt)
except KeyboardInterrupt:
    pass

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Return to starting pose? (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

for id in motor_IDs:
        # Set position planning accelerations to 5000 for single commands.
        try:
            motor[id].setAcceleration(5000, pos_accel)
        except rmd.ProtocolException as exc:
            print(exc)
        try:
            motor[id].setAcceleration(5000, pos_decel)
        except rmd.ProtocolException as exc:
            print(exc)
        # Assume startup pose.
        try:
            motor[id].sendPositionAbsoluteSetpoint(startup_pos[id], 100)
        except rmd.can.BusError:
            error_no += 1
            print("Bus Error: ", error_no)
        except rmd.ProtocolException as exc:
            print(exc)

