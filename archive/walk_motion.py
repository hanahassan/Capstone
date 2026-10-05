import os
import time
import math
import myactuator_rmd_py as rmd
from init_canports import init_canports

## Define variables
# motor_IDs = list(range(1, 7)) + list(range(10, 18))
# motor_IDs = [5]
motor_IDs = [11, 13, 15, 17]

t = 0
timestep = 0.012

# Motion control gains, 12-bit int 0-4095.
kp = 50 # converts from 0-4095 to 0-500 (82 = 10).
# for repeated commands:
kd = 819 # converts from 0-4095 to 0-5 (819 = 1).
# for individual commands:
kd2 = 1400

file_pos = 'Data_joint_positions_walk_0.002.txt'
file_vel = 'Data_joint_velocities_walk_0.002.txt'

bus_error_no = 0
step_error_no = 0

v2_reduction = 9
v3_reduction = 6.2

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Confirm ContinuO in startup pose on flat surface (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

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
    '''Convert from model to motor position and direction (radians).'''
    if motor_id == 1 or motor_id == 2:
        return -model_pos
    elif motor_id == 3:
        return model_pos - math.radians(96.790)
    elif motor_id == 4:
        return -model_pos + math.radians(96.720)
    elif motor_id == 5:
        return model_pos - math.radians(-193.090)
    elif motor_id == 6:
        return -model_pos + math.radians(-193.290)
    elif motor_id == 10 or motor_id == 11:
        return model_pos
    elif motor_id == 12:
        return -model_pos + math.radians(-80.650)
    elif motor_id == 13:
        return model_pos - math.radians(-81.890)
    elif motor_id == 14:
        return -model_pos + math.radians(182.540)
    elif motor_id == 15:
        return model_pos - math.radians(182.940)
    elif motor_id == 16:
        return -model_pos + math.radians(-199.990)
    elif motor_id == 17:
        return model_pos - math.radians(-200.610)
    
# def model_to_motor_vel(motor_id, model):
#     if motor_id in [1, 2, 4, 6, 12, 14, 16]:
#         return -model
#     else: 
#         return model 

# Read initial position of model files and convert to motor ID dictionary (radians).
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

# for id in motor_IDs:
#         # Reset encoder zero and reset motor to avoid out of range errors of motion control.
#         motor[id].setCurrentPositionAsEncoderZero()
#         motor[id].reset()

# # Wait after reset.
# time.sleep(0.2)

# Dicts for joint positions in pose flat on ground in degrees and radians.
startup_pos = {}
startup_pos_rad = {}

# pos_accel = rmd.actuator_state.AccelerationType(0)
# pos_decel = rmd.actuator_state.AccelerationType(1)

# for id in motor_IDs:
#     # Reset encoder zero and reset motor.
#     motor[id].setCurrentPositionAsEncoderZero()
#     motor[id].reset()
# # Wait after reset.
# time.sleep(1)

for id in motor_IDs:
    # Read actuator startup positions for later calculations of offset -- should be ~0.
    startup_pos[id] = motor[id].getMultiTurnAngle()
    startup_pos_rad[id] = math.radians(startup_pos[id])

    # # Set position planning accelerations to 10000 for single commands.
    # motor[id].setAcceleration(10000, pos_accel)
    # motor[id].setAcceleration(10000, pos_decel)

    # # Assume initial pose from file.
    # motion_motor[id].motionModeControl(round(startup_pos_rad[id] + ready_pos[id], 3), 0.5, kp, kd, 0)

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Confirm ContinuO is suspended, ready to assume ready pose (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

## Assume initial pose from file, groups of motors at a time.
proceed = None
while proceed != 'c' and proceed != 'C':
    # for id in [1, 2, 10, 11]:
    for id in [11]:
        try:
            motion_motor[id].motionModeControl(round(startup_pos_rad[id] + ready_pos[id], 3), 0, kp, kd2, 0)
        except rmd.can.BusError:
            bus_error_no += 1
            print("Bus Error: ", bus_error_no)
    proceed = input('(C)ontinue, (E)xit, or retry (any other)?:\n')
    if proceed == 'e' or proceed == 'E':
        print('Terminating program.')
        exit()
proceed = None
while proceed != 'c' and proceed != 'C':
    # for id in [3, 4, 12, 13]:
    for id in [13]:
        try:
            motion_motor[id].motionModeControl(round(startup_pos_rad[id] + ready_pos[id], 3), 0, kp, kd2, 0)
        except rmd.can.BusError:
            bus_error_no += 1
            print("Bus Error: ", bus_error_no)
    proceed = input('(C)ontinue, (E)xit, or retry (any other)?:\n')
    if proceed == 'e' or proceed == 'E':
        print('Terminating program.')
        exit()
proceed = None
while proceed != 'c' and proceed != 'C':
    # for id in [5, 6, 14, 15]:
    for id in [15]:
        try:
            motion_motor[id].motionModeControl(round(startup_pos_rad[id] + ready_pos[id], 3), 0, kp, kd2, 0)
        except rmd.can.BusError:
            bus_error_no += 1
            print("Bus Error: ", bus_error_no)
    proceed = input('(C)ontinue, (E)xit, or retry (any other)?:\n')
    if proceed == 'e' or proceed == 'E':
        print('Terminating program.')
        exit()
proceed = None
while proceed != 'c' and proceed != 'C':
    # for id in [16, 17]:
    for id in [17]:
        try:
            motion_motor[id].motionModeControl(round(startup_pos_rad[id] + ready_pos[id], 3), 0, kp, kd2, 0)
        except rmd.can.BusError:
            bus_error_no += 1
            print("Bus Error: ", bus_error_no)
    proceed = input('(C)ontinue, (E)xit, or retry (any other)?:\n')
    if proceed == 'e' or proceed == 'E':
        print('Terminating program.')
        exit()

proceed = None
while proceed != 'y' and proceed != 'Y':
    proceed = input('Confirm ContinuO feet in position (y/n):\n')
    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

desired_pos = {}
desired_vel = {}

while t <= 3600:
    line_number = t+1
    model_desired_pos = read_line_as_array(file_pos, line_number)
    model_desired_vel = read_line_as_array(file_vel, line_number)

    # Front right leg.
    desired_pos[1] = model_to_motor_pos(1, model_desired_pos[0])
    desired_pos[3] = model_to_motor_pos(3, model_desired_pos[1])
    desired_pos[5] = model_to_motor_pos(5, model_desired_pos[2])
    # Desired velocity is motor-side of reducer (i.e., before reduction). Negatives to transform direction where motor orientation opposite model.
    desired_vel[1] = -model_desired_vel[0] * v2_reduction 
    desired_vel[3] = model_desired_vel[1] * v3_reduction
    desired_vel[5] = model_desired_vel[2] * v3_reduction 

    # Front left leg.
    desired_pos[2] = model_to_motor_pos(2, model_desired_pos[3])
    desired_pos[4] = model_to_motor_pos(4, model_desired_pos[4])
    desired_pos[6] = model_to_motor_pos(6, model_desired_pos[5])
    desired_vel[2] = -model_desired_vel[3] * v2_reduction
    desired_vel[4] = -model_desired_vel[4] * v3_reduction
    desired_vel[6] = -model_desired_vel[5] * v3_reduction

    # Hind right leg.
    desired_pos[11] = model_to_motor_pos(11, model_desired_pos[6])
    desired_pos[13] = model_to_motor_pos(13, model_desired_pos[7])
    desired_pos[15] = model_to_motor_pos(15, model_desired_pos[8])
    desired_pos[17] = model_to_motor_pos(17, model_desired_pos[9])
    desired_vel[11] = model_desired_vel[6] * v2_reduction
    desired_vel[13] = model_desired_vel[7] * v2_reduction
    desired_vel[15] = model_desired_vel[8] * v3_reduction
    desired_vel[17] = model_desired_vel[9] * v3_reduction

    # Hind left leg.
    desired_pos[10] = model_to_motor_pos(10, model_desired_pos[10])
    desired_pos[12] = model_to_motor_pos(12, model_desired_pos[11])
    desired_pos[14] = model_to_motor_pos(14, model_desired_pos[12])
    desired_pos[16] = model_to_motor_pos(16, model_desired_pos[13])
    desired_vel[10] = model_desired_vel[10] * v2_reduction
    desired_vel[12] = -model_desired_vel[11] * v2_reduction
    desired_vel[14] = -model_desired_vel[12] * v3_reduction
    desired_vel[16] = -model_desired_vel[13] * v3_reduction

    p_d = {}
    # for id in desired_pos:
    for id in motor_IDs:
        p_d[id] = round(startup_pos_rad[id] + desired_pos[id], 3)
    # print("line " + str(line_number) + ":\n", 
    #       "p_des:", p_d, "\n", 
    #       "v_des:", desired_vel)

    t1 = time.perf_counter()
    for id in motor_IDs:
    # for id in [5]:
        try:
            motion_motor[id].motionModeControl(round(startup_pos_rad[id] + desired_pos[id], 3), desired_vel[id], kp, kd, 0)
        except rmd.can.BusError:
            bus_error_no += 1
            print("Bus Error: ", bus_error_no)

    t2 = time.perf_counter()
    dt = t2 - t1
    print("exectution dt:",dt,"s")

    if dt <= timestep:
        time.sleep(timestep - dt)
    elif dt > timestep:
        step_error_no += 1
        print("timestep error:",step_error_no)

    # t += 1
    t += 6
    # time.sleep(dt)

for id in motor_IDs:
    # Assume startup pose.
    try:
        motion_motor[id].motionModeControl(round(startup_pos_rad[id], 3), 0, kp, kd2, 0)    
    except rmd.can.BusError:
        bus_error_no += 1
        print("Bus Error: ", bus_error_no)