import os
import time
import csv
import matplotlib.pyplot as plt
import myactuator_rmd_py as rmd
from Continuo_init_canports import init_canports
from Continuo_init_motors import init_motors

#---------------------------
#   PARAMETERS
#---------------------------

# Motor targert ID (CAN ID)

target_id = 16

# New PI gains

#gains = rmd.actuator_state.Gains(100, 100, 180, 40, 30, 0)  # <- CHANGE GAINS !

# Step input parameters

position_step = 30.0
max_speed = 300
step_duration = 5.0
sample_time = 0.0001
pre_step_duration = 1.0

# CSV file name

output_file = "position_step_response_motor_16.csv"


#---------------------------
#   FUNCTIONS
#---------------------------

# This function is used to send commands via the CAN bus

def retry_call(label, func, max_attempts=10, delay=0.05):
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


# This function is used to extract the motor position from the status of the motor

def get_motor_position(motor, motor_id):
    status = retry_call(
        f"Read motor status {motor_id}",
        lambda: motor[motor_id].getMotorStatus2()
    )
    return status.shaft_angle

# This function is used to send the motor position using the retry_call function

def send_position(motor, motor_id, position, speed):
    retry_call(
        f"Send position setpoint motor {motor_id}",
        lambda: motor[motor_id].sendPositionAbsoluteSetpoint(position, speed)
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

# Register the startup position

startup_pos = retry_call(
    f"Read startup position motor {target_id}",
    lambda: motor[target_id].getMultiTurnAngle()
)

target_position = startup_pos + position_step
print("OK", target_id, startup_pos)

# Acceleration configuration

pos_accel = rmd.actuator_state.AccelerationType(0)
pos_decel = rmd.actuator_state.AccelerationType(1)
motor[target_id].setAcceleration(2000, pos_accel)
motor[target_id].setAcceleration(2000, pos_decel)

# Set PI gains

# retry_call(
#     f"Write controller gains motor {target_id}",
#     lambda: motor[target_id].setControllerGains(gains, True)
# )

#---------------------------
#   PLOT CONFIGURATION
#---------------------------

time_data = []
position_ref_data = []
position_meas_data = []

plt.ion()

plt.rcParams.update({
    'font.size': 16,
    'axes.titlesize': 20,
    'axes.labelsize': 18,
    'xtick.labelsize': 16,
    'ytick.labelsize': 16,
    'legend.fontsize': 16
})

fig, ax = plt.subplots(figsize=(10, 6))

line_ref, = ax.plot([], [], linewidth=2, label="Position setpoint")
line_meas, = ax.plot([], [], linewidth=2, label="Measured position")

ax.set_title(f"Position step response - Motor {target_id}", fontsize=30)
ax.set_xlabel("Time [s]", fontsize=20)
ax.set_ylabel("Position [deg]", fontsize=20)

ax.grid(True)
ax.minorticks_on()
ax.grid(which='major', linestyle='-', linewidth=1.0)
ax.grid(which='minor', linestyle='--', linewidth=0.5)
ax.legend()

fig.tight_layout()

start_time = time.perf_counter()

# Measurement

last_position_ref = None
plot_update_every = 1000

ax.set_xlim(0, pre_step_duration + step_duration)
ax.set_ylim(
    min(startup_pos, target_position) - 5,
    max(startup_pos, target_position) + 5
)

try:
    while True:
        loop_start = time.perf_counter()
        t = loop_start - start_time

        if t >= (pre_step_duration + step_duration):
            break

        if t < pre_step_duration:
            position_ref = startup_pos
        else:
            position_ref = target_position

        if position_ref != last_position_ref:
            send_position(motor, target_id, position_ref, max_speed)
            last_position_ref = position_ref

        measured_position = get_motor_position(motor, target_id)

        time_data.append(t)
        position_ref_data.append(position_ref)
        position_meas_data.append(measured_position)

        if len(time_data) % plot_update_every == 0:
            line_ref.set_data(time_data, position_ref_data)
            line_meas.set_data(time_data, position_meas_data)

            plt.pause(0.001)

        dt = time.perf_counter() - loop_start

        if dt < sample_time:
            time.sleep(sample_time - dt)

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:

    # Motor stop

    retry_call(
        "Stop target motor",
        lambda: motor[target_id].stopMotor()
    )

    # Create the CSV file with all gains
    
    with open(output_file, mode="w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            "time_s",
            "position_setpoint",
            "position_measured"
        ])

        for row in zip(
            time_data,
            position_ref_data,
            position_meas_data
        ):
            writer.writerow(row)

    print("Position setpoint reset.")
    print(f"Data exported to {output_file}")

    time.sleep(1)

    # Motor Reset

    retry_call(
        f"Reset acceleration motor {target_id}",
        lambda: motor[target_id].setAcceleration(10000, pos_accel)
    )

    retry_call(
        f"Reset deceleration motor {target_id}",
        lambda: motor[target_id].setAcceleration(10000, pos_decel)
    )

    retry_call(
        f"Return motor {target_id} to startup position",
        lambda: motor[target_id].sendPositionAbsoluteSetpoint(startup_pos, 50)
    )

    # Plot reset

    plt.ioff()
    plt.show()