import os
import time
import math
import myactuator_rmd_py as rmd

def init_motors():
    motor = {}
    can = {}

    can["fr"] = rmd.CanDriver("can0")
    motor[1] = rmd.ActuatorInterface(can["fr"], 1)
    motor[3] = rmd.ActuatorInterface(can["fr"], 3)
    motor[5] = rmd.ActuatorInterface(can["fr"], 5)

    can["fl"] = rmd.CanDriver("can1")
    motor[2] = rmd.ActuatorInterface(can["fl"], 2)
    motor[4] = rmd.ActuatorInterface(can["fl"], 4)
    motor[6] = rmd.ActuatorInterface(can["fl"], 6)

    can["hr"] = rmd.CanDriver("can2")
    motor[11] = rmd.ActuatorInterface(can["hr"], 11)
    motor[13] = rmd.ActuatorInterface(can["hr"], 13)
    motor[15] = rmd.ActuatorInterface(can["hr"], 15)
    motor[17] = rmd.ActuatorInterface(can["hr"], 17)

    can["hl"] = rmd.CanDriver("can3")
    motor[10] = rmd.ActuatorInterface(can["hl"], 10)
    motor[12] = rmd.ActuatorInterface(can["hl"], 12)
    motor[14] = rmd.ActuatorInterface(can["hl"], 14)
    motor[16] = rmd.ActuatorInterface(can["hl"], 16)

    motor_IDs = list(range(1, 7)) + list(range(10, 18))

    return motor, motor_IDs, can

if __name__ == '__main__':
    init_motors()