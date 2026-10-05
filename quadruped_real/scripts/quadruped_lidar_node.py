#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node communicate with the liddar and publish its datas. 
Author : 
Date of creation : 29/06/2026
Version : 1.0
"""

# ---------------------------
# IMPORTS
# ---------------------------

import sys
import termios
import tty
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray


# ---------------------------
# MAIN Function
# ---------------------------

def main():
    rospy.init_node("quadruped_lidar_node")

    pub = rospy.Publisher("/lidar", Float32MultiArray, queue_size=1, latch=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        msg = Float32MultiArray()
        msg.data = [1.0, 2.0, 3.0, 4.0]  # valeurs dummy

        pub.publish(msg)

        rate.sleep()


# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    main()


 