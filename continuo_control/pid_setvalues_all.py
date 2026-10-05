import os
import time
import csv
import myactuator_rmd_py as rmd
from Continuo_init_canports import init_canports
from Continuo_init_motors import init_motors

#---------------------------
#   PARAMETERS
#---------------------------


# Gains format:
# rmd.actuator_state.Gains(
#     current_kp,
#     current_ki,
#     speed_kp,
#     speed_ki,
#     position_kp,
#     position_ki
# )

motor_gains = {
    # Front right leg
    1:  rmd.actuator_state.Gains(50, 50, 100, 4, 20, 0),
    3:  rmd.actuator_state.Gains(100, 100, 90, 50, 40, 0),
    5:  rmd.actuator_state.Gains(100, 100, 110, 25, 75, 0),

    # Front left leg
    2:  rmd.actuator_state.Gains(50, 50, 100, 4, 20, 0),
    4:  rmd.actuator_state.Gains(100, 100, 90, 50, 40, 0),
    6:  rmd.actuator_state.Gains(100, 100, 110, 25, 75, 0),

    # Hind right leg
    11: rmd.actuator_state.Gains(50, 50, 120, 6, 15, 0),
    13: rmd.actuator_state.Gains(50, 50, 105, 2, 10, 0),
    15: rmd.actuator_state.Gains(100, 100, 85, 95, 40, 0),
    17: rmd.actuator_state.Gains(100, 100, 90, 20, 75, 0),

    # Hind left leg
    10: rmd.actuator_state.Gains(50, 50, 120, 6, 15, 0),
    12: rmd.actuator_state.Gains(50, 50, 105, 2, 10, 0),
    14: rmd.actuator_state.Gains(100, 100, 85, 95, 40, 0),
    16: rmd.actuator_state.Gains(100, 100, 90, 20, 75, 0),
}


#---------------------------
#   FUNCTIONS DEFINITION
#---------------------------

# This function is used to send commands via the CAN bus

def retry_call(label, func, max_attempts=5, delay=0.05):
    for attempt in range(1, max_attempts + 1):
        try:
            return func()
        except rmd.can.BusError:
            print(f"{label}: BusError attempt {attempt}/{max_attempts}")
            time.sleep(delay)
        except rmd.ProtocolException as exc:
            print(f"{label}: ProtocolException attempt {attempt}/{max_attempts}: {exc}")
            time.sleep(delay)

    raise RuntimeError(f"{label}: failed after {max_attempts} attempts")



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


# Set new gains

for motor_id in motor_IDs:

    if motor_id not in motor_gains:
        raise RuntimeError(f"No gains defined for motor {motor_id}")

    gains = motor_gains[motor_id]

    retry_call(
        f"Write controller gains motor {motor_id}",
        lambda motor_id=motor_id, gains=gains:
            motor[motor_id].setControllerGains(gains, True)
    )

    print(f"Gains updated for motor {motor_id}")