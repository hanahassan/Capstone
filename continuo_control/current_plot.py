import time
import csv
import matplotlib.pyplot as plt
import myactuator_rmd_py as rmd

#---------------------------
#   PARAMETERS
#---------------------------

motor_IDs = list(range(1, 7)) + list(range(10, 18))

sample_time = 0.1
output_file = "current_monitor_log.csv"

# Target motor ID

plot_motor_IDs = [15]

#---------------------------
#   INIT MOTORS ONLY
#---------------------------

motor = {}

can_fr = rmd.CanDriver("can0")
motor[1] = rmd.ActuatorInterface(can_fr, 1)
motor[3] = rmd.ActuatorInterface(can_fr, 3)
motor[5] = rmd.ActuatorInterface(can_fr, 5)

can_fl = rmd.CanDriver("can1")
motor[2] = rmd.ActuatorInterface(can_fl, 2)
motor[4] = rmd.ActuatorInterface(can_fl, 4)
motor[6] = rmd.ActuatorInterface(can_fl, 6)

can_hr = rmd.CanDriver("can2")
motor[11] = rmd.ActuatorInterface(can_hr, 11)
motor[13] = rmd.ActuatorInterface(can_hr, 13)
motor[15] = rmd.ActuatorInterface(can_hr, 15)
motor[17] = rmd.ActuatorInterface(can_hr, 17)

can_hl = rmd.CanDriver("can3")
motor[10] = rmd.ActuatorInterface(can_hl, 10)
motor[12] = rmd.ActuatorInterface(can_hl, 12)
motor[14] = rmd.ActuatorInterface(can_hl, 14)
motor[16] = rmd.ActuatorInterface(can_hl, 16)

#---------------------------
#   PLOT CONFIG
#---------------------------

plt.ion()

plt.rcParams.update({
    "font.size": 14,
    "axes.titlesize": 18,
    "axes.labelsize": 16,
    "xtick.labelsize": 14,
    "ytick.labelsize": 14,
    "legend.fontsize": 10
})

fig, ax = plt.subplots(figsize=(14, 8))

time_data = []
current_data = {motor_id: [] for motor_id in plot_motor_IDs}

lines = {}

for motor_id in plot_motor_IDs:
    line, = ax.plot([], [], linewidth=2, label=f"Motor {motor_id}")
    lines[motor_id] = line

ax.set_title("Motor current monitoring", fontsize=30)
ax.set_xlabel("Time [s]", fontsize=20)
ax.set_ylabel("Current [A]", fontsize=20)
ax.grid(True)
ax.minorticks_on()
ax.grid(which="major", linestyle="-", linewidth=1.0)
ax.grid(which="minor", linestyle="--", linewidth=0.5)
ax.legend(loc="upper right")

fig.tight_layout()

#---------------------------
#   MAIN LOOP
#---------------------------

start_time = time.perf_counter()

with open(output_file, mode="w", newline="") as file:
    writer = csv.writer(file)

    header = ["time_s"] + [f"motor_{motor_id}_current" for motor_id in plot_motor_IDs]
    writer.writerow(header)

    try:
        while True:
            loop_start = time.perf_counter()
            t = loop_start - start_time

            row = [t]
            time_data.append(t)

            for motor_id in plot_motor_IDs:
                try:
                    status = motor[motor_id].getMotorStatus2()
                    current = status.current
                except rmd.can.BusError:
                    current = None
                    print(f"BusError reading motor {motor_id}")
                except rmd.ProtocolException as exc:
                    current = None
                    print(f"ProtocolException reading motor {motor_id}: {exc}")

                row.append(current)

                if current is None:
                    current_data[motor_id].append(float("nan"))
                else:
                    current_data[motor_id].append(current)

            writer.writerow(row)
            file.flush()

            for motor_id in plot_motor_IDs:
                lines[motor_id].set_data(time_data, current_data[motor_id])

            ax.relim()
            ax.autoscale_view()

            plt.pause(0.001)

            dt = time.perf_counter() - loop_start

            if dt < sample_time:
                time.sleep(sample_time - dt)

    except KeyboardInterrupt:
        print("Current monitoring stopped.")

plt.ioff()
plt.show()

print(f"Current log saved to {output_file}")