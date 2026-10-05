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

target_id = 14

# New PI gains

gains = rmd.actuator_state.Gains(100,100,170,190,20,0)

# Step input parameters

speed_step = 50        
step_duration = 0.5        
sample_time = 0.001          
pre_step_duration = 1.0     

# CSV file name

output_file = "speed_step_response_motor_17.csv"



#---------------------------
#   FUNCTIONS
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

# This function is used to extract the motor speed from the status of the motor

def get_motor_speed(motor, motor_id):
    status = retry_call(
        f"Read motor status {motor_id}",
        lambda: motor[motor_id].getMotorStatus2()
    )
    return status.shaft_speed

# This function is used to send the motor speed using the retry_call function

def send_speed(motor, motor_id, speed):
    retry_call(
        f"Send speed setpoint motor {motor_id}",
        lambda: motor[motor_id].sendVelocitySetpoint(speed)
    )

#---------------------------
#   MAIN PROGRAM
#---------------------------

#User confirmation

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

startup_pos= retry_call(
        f"Read startup position motor {target_id}",
        lambda: motor[target_id].getMultiTurnAngle()
    )

print("OK", target_id, startup_pos)
    
# Acceleration configuration

pos_accel = rmd.actuator_state.AccelerationType(0)
pos_decel = rmd.actuator_state.AccelerationType(1)

motor[target_id].setAcceleration(0, pos_accel)
motor[target_id].setAcceleration(0, pos_decel)

# Set PI gains

retry_call(
    f"Write controller gains motor {target_id}",
    lambda: motor[target_id].setControllerGains(gains, True)
)


#Plot configuration

time_data = []
speed_ref_data = []
speed_meas_data = []

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

line_ref, = ax.plot([], [], linewidth=2, label="Speed setpoint")
line_meas, = ax.plot([], [], linewidth=2, label="Measured speed")

ax.set_title(f"Speed step response - Motor {target_id}", fontsize=30)
ax.set_xlabel("Time [s]", fontsize=20)
ax.set_ylabel("Speed", fontsize=20)

ax.grid(True)
ax.minorticks_on()

ax.grid(which='major', linestyle='-', linewidth=1.0)
ax.grid(which='minor', linestyle='--', linewidth=0.5)
ax.legend()


fig.tight_layout()

start_time = time.perf_counter()

# Measurement

last_ref = None

try:

    while True:

        loop_start = time.perf_counter()

        t = loop_start - start_time

        if t >= (pre_step_duration + step_duration):
            break

        if t < pre_step_duration:
            current_ref = 0
        else:
            current_ref = speed_step

        if current_ref != last_ref:
            send_speed(motor, target_id, current_ref)
            last_ref = current_ref

        read_start = time.perf_counter()
        measured_speed = get_motor_speed(motor, target_id)
        read_dt = time.perf_counter() - read_start
        print("CAN read time:", read_dt)


        time_data.append(t)
        speed_ref_data.append(current_ref)
        speed_meas_data.append(measured_speed)

        if len(time_data) % 20 == 0:

            line_ref.set_data(time_data, speed_ref_data)
            line_meas.set_data(time_data, speed_meas_data)

            ax.relim()
            ax.autoscale_view()

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

    with open(output_file, mode="w", newline="") as file:

        writer = csv.writer(file)

        writer.writerow([
            "time_s",
            "speed_setpoint",
            "speed_measured"
        ])

        for row in zip(
            time_data,
            speed_ref_data,
            speed_meas_data
        ):
            writer.writerow(row)

    print("Speed setpoint reset to zero.")
    print(f"Data exported to {output_file}")

    time.sleep(1)

    # Motor Reset

    retry_call(
        "Reset acceleration motor 17",
        lambda: motor[target_id].setAcceleration(10000, pos_accel)
    )

    retry_call(
        "Reset deceleration motor 17",
        lambda: motor[target_id].setAcceleration(10000, pos_decel)
    )

    retry_call(
        "Return motor 17 to startup position",
        lambda: motor[target_id].sendPositionAbsoluteSetpoint(startup_pos, 50)
    )

    plt.ioff()
    plt.show()