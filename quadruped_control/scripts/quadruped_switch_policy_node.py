#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node publish commands to switch between different control policies.
Author :
Date of creation : 29/06/2026
Version : 1.0
"""

# ---------------------------
# IMPORTS
# ---------------------------

import rospy
import threading
from std_msgs.msg import String


class PolicySwitcher:

    # ---------------------------
    # FUNCTIONS DEFINITION
    # ---------------------------
    # Initialization
    def __init__(self):

        # Default policy
        self.current_policy = "flat"
        self.lock = threading.Lock()

        # ROS Publisher 

        self.pub = rospy.Publisher("/switch_cp", String, queue_size=1)

        self.input_thread = threading.Thread(target=self.keyboard_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    # Allow the user to entrer the name of a policy in order to switch to the desired policy
    def keyboard_loop(self):

        while not rospy.is_shutdown():

            try:
                policy = input("Desired policy : ").strip()

                if policy:
                    with self.lock:
                        self.current_policy = policy

                    rospy.loginfo(
                        "New policy selected: %s",
                        policy
                    )

            except EOFError:
                break

    # Runninf the node
    def run(self):

        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():

            msg = String()

            with self.lock:
                msg.data = self.current_policy

            self.pub.publish(msg)

            rate.sleep()


#---------------------------
# MAIN LOOP
#---------------------------          

def main():

    rospy.init_node("quadruped_switch_policy_node")

    node = PolicySwitcher()

    rospy.loginfo("Entrer le nom d'une policy dans le terminal.")

    node.run()


# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    main()