import os
import time
import math
import myactuator_rmd_py as rmd
from Continuo_init_canports import init_canports
from Continuo_init_motors import init_motors

#---------------------------
#   VARIABLES
#---------------------------

bus_error_no = 0
step_err_no = 0

max_angle = {
    1: 80,
    2: -80,
    3: -50,
    4: 50,
    5: 150,
    6: -150,
    10: 30,
    11: -30,
    12: 60,
    13: -60,
    14: -60,
    15: 60,
    16: -130,
    17: 130
}

legs = {
    "front_right": [5, 3, 1],
    "front_left": [6, 4, 2],
    "hind_right": [17, 15, 13, 11],
    "hind_left": [16, 14, 12, 10],
}

active_legs = ["front_right"]

freq = 0.3
t = 0
infoMaxTime = 0.1
inter_motor_delay = 0.02

#---------------------------
#   FUNCTIONS
#---------------------------

def retry_call(label, func, *args, max_attempts=2, delay=0.001):
    for attempt in range(1, max_attempts + 1):
        try:
            return func(*args)

        except rmd.can.BusError:
            print(f"{label}: BusError attempt {attempt}/{max_attempts}")
            time.sleep(delay)

        except rmd.ProtocolException as exc:
            print(f"{label}: ProtocolException attempt {attempt}/{max_attempts}: {exc}")
            time.sleep(delay)

    raise RuntimeError(f"{label}: failed after {max_attempts} attempts")


def sin_pos(t, motor_id):
    return (
        startup_pos[motor_id]
        + max_angle[motor_id] / 2
        * (1 - math.cos(2 * math.pi * freq * t))
    )


def read_startup_position(motor, motor_id):
    return motor[motor_id].getMultiTurnAngle()


def set_motor_acceleration(motor, motor_id, value, accel_type):
    return motor[motor_id].setAcceleration(value, accel_type)


def send_sinus_position(motor, motor_id, t, speed):
    target = sin_pos(t, motor_id)
    return motor[motor_id].sendPositionAbsoluteSetpoint(target, speed)


def return_to_startup_position(motor, motor_id, speed):
    return motor[motor_id].sendPositionAbsoluteSetpoint(
        startup_pos[motor_id],
        speed
    )

#---------------------------
#   MAIN PROGRAM
#---------------------------

# User confirmation

proceed = None

while proceed != 'y' and proceed != 'Y':
    proceed = input('Confirm ContinuO in startup pose and suspended (y/n):\n')

    if proceed == 'n' or proceed == 'N':
        print('Terminating program.')
        exit()

# Initialization

init_canports()
motor, motor_IDs, can = init_motors()
startup_pos = {}

pos_accel = rmd.actuator_state.AccelerationType(0)
pos_decel = rmd.actuator_state.AccelerationType(1)


for leg_name, leg_motor_ids in legs.items():
    for motor_id in leg_motor_ids:
        if leg_name not in active_legs:
            continue

        time.sleep(inter_motor_delay)

        print("Reading motor", motor_id)

        startup_pos[motor_id] = retry_call(
            f"Read startup position motor {motor_id}",
            read_startup_position,
            motor,
            motor_id
        )

        print("OK", motor_id, startup_pos[motor_id])

        retry_call(
            f"Set acceleration motor {motor_id}",
            set_motor_acceleration,
            motor,
            motor_id,
            0,
            pos_accel
        )

        retry_call(
            f"Set deceleration motor {motor_id}",
            set_motor_acceleration,
            motor,
            motor_id,
            0,
            pos_decel
        )

# MAIN Loop

try:
    while t < 10:
        t1 = time.perf_counter()

        for leg_name, leg_motor_ids in legs.items():
            for motor_id in leg_motor_ids:
                if leg_name not in active_legs:
                    continue
                
                time.sleep(inter_motor_delay)

                retry_call(
                    f"Send position {leg_name} motor {motor_id}",
                    send_sinus_position,
                    motor,
                    motor_id,
                    t,
                    500
                )

                
        t2 = time.perf_counter()
        dt = t2 - t1

        t += dt 

        if dt >= infoMaxTime:
            step_err_no += 1
            print("execution longer than intended timestep:", dt, step_err_no)

            

except KeyboardInterrupt:
    pass

finally:
    time.sleep(2)

    for leg_name, leg_motor_ids in legs.items():
        for motor_id in leg_motor_ids:
            if leg_name not in active_legs:
                continue

            time.sleep(inter_motor_delay)

            retry_call(
                f"Reset acceleration motor {motor_id}",
                set_motor_acceleration,
                motor,
                motor_id,
                10000,
                pos_accel
            )

            retry_call(
                f"Reset deceleration motor {motor_id}",
                set_motor_acceleration,
                motor,
                motor_id,
                10000,
                pos_decel
            )

            retry_call(
                f"Return motor {motor_id} to startup position",
                return_to_startup_position,
                motor,
                motor_id,
                50
            )