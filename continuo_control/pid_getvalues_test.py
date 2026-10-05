import os
import time
import csv
import myactuator_rmd_py as rmd
from Continuo_init_canports import init_canports
from Continuo_init_motors import init_motors

#---------------------------
#   PARAMETERS
#---------------------------

# Name of the CSV file

output_file = "continuo_motor_pid_gains.csv"

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


# This function is used to extract PI values from the status of the motor

def extract_pid_gains(pid_gains):
    return {
        "current_kp": pid_gains.current.kp,
        "current_ki": pid_gains.current.ki,
        "speed_kp": pid_gains.speed.kp,
        "speed_ki": pid_gains.speed.ki,
        "position_kp": pid_gains.position.kp,
        "position_ki": pid_gains.position.ki,
    }

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

rows = []

fieldnames = [
    "motor_id",
    "status",
    "current_kp",
    "current_ki",
    "speed_kp",
    "speed_ki",
    "position_kp",
    "position_ki",
]

# Read PI gains of every motor

for motor_id in motor_IDs:
    print(f"Reading gains motor {motor_id}")

    try:
        pid_gains = retry_call(
            f"Read motor gains {motor_id}",
            lambda motor_id=motor_id: motor[motor_id].getControllerGains()
        )

        gains = extract_pid_gains(pid_gains)

        row = {
            "motor_id": motor_id,
            "status": "OK",
            **gains
        }

        print(f"Motor {motor_id} OK: {pid_gains}")

    except Exception as exc:
        row = {
            "motor_id": motor_id,
            "status": f"ERROR: {exc}",
            "current_kp": "",
            "current_ki": "",
            "speed_kp": "",
            "speed_ki": "",
            "position_kp": "",
            "position_ki": "",
        }

        print(f"Motor {motor_id} ERROR: {exc}")

    rows.append(row)


# Create the CSV file with all gains

with open(output_file, mode="w", newline="") as file:
    writer = csv.DictWriter(file, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)

print(f"Gains exported to {output_file}")
