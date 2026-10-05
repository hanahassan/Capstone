#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node publishes fake data to simulate the behavior of real sensors
Author : Florent Pralong
Date of creation : 28/06/2026
Version : 2.0
"""

# ---------------------------
# IMPORTS
# ---------------------------

import csv
import rospy
import rospkg

from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray


class QuadrupedFakeObsDatasNode:

    # ---------------------------
    # INIT
    # ---------------------------

    def __init__(self):

        # PARAMETERS

        rospy.init_node("quadruped_fake_obs_datas_node")

        # Refer to the real csv file
        self.csv_name = rospy.get_param("~csv_name", "flat_obs_actions_targets_pos_3_cut.csv")

        self.publish_hz = rospy.get_param("~publish_hz", 50.0)
        self.loop = rospy.get_param("~loop", True)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("quadruped_control")

        self.csv_path = rospy.get_param(
            "~csv_path",
            package_path + "/src/" + self.csv_name
        )

        # ROS Publisher and Subscriber

        self.pub_imu = rospy.Publisher("/imu", Imu, queue_size=10)
        self.pub_cmd = rospy.Publisher("/cmd", Float32MultiArray, queue_size=10)
        self.pub_joint_states = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.pub_lidar = rospy.Publisher("/lidar", Float32MultiArray, queue_size=10)

        # Joints names order
        self.joint_names = [
            "FL_HAA", "FR_HAA", "HL_HAA", "HR_HAA",
            "FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE",
            "FL_KFE", "FR_KFE", "HL_KFE", "HR_KFE",
            "HL_AFE", "HR_AFE"
        ]

        # CSV Loading
        self.data = self.load_csv()

        rospy.loginfo("Loaded %d rows from %s", len(self.data), self.csv_path)
        rospy.loginfo("Publishing fake observation data at %.2f Hz", self.publish_hz)

    # ---------------------------
    # FUNCTIONS DEFINITION
    # ---------------------------
    # Extract float number from a csv row
    def get_float(self, row, key, line_index):
        try:
            return float(row[key])
        except KeyError:
            rospy.logerr("Missing column '%s' in CSV at line %d", key, line_index + 2)
            raise
        except ValueError:
            rospy.logerr("Invalid float for column '%s' in CSV at line %d", key, line_index + 2)
            raise

    # Loading csv file
    def load_csv(self):
        rows = []

        with open(self.csv_path, "r") as csv_file:
            reader = csv.DictReader(csv_file)

            # Extracting data from the csv file 
            for line_index, row in enumerate(reader):

                imu_data = {
                    "base_lin_vel": [
                        self.get_float(row, "obs_0", line_index),
                        self.get_float(row, "obs_1", line_index),
                        self.get_float(row, "obs_2", line_index),
                    ],
                    "base_ang_vel": [
                        self.get_float(row, "obs_3", line_index),
                        self.get_float(row, "obs_4", line_index),
                        self.get_float(row, "obs_5", line_index),
                    ],
                    "projected_gravity": [
                        self.get_float(row, "obs_6", line_index),
                        self.get_float(row, "obs_7", line_index),
                        self.get_float(row, "obs_8", line_index),
                    ],
                }

                cmd_data = [
                    self.get_float(row, "obs_9", line_index),
                    self.get_float(row, "obs_10", line_index),
                    self.get_float(row, "obs_11", line_index),
                    self.get_float(row, "obs_12", line_index),
                ]

                joint_pos = [
                    self.get_float(row, "obs_13", line_index),
                    self.get_float(row, "obs_14", line_index),
                    self.get_float(row, "obs_15", line_index),
                    self.get_float(row, "obs_16", line_index),
                    self.get_float(row, "obs_17", line_index),
                    self.get_float(row, "obs_18", line_index),
                    self.get_float(row, "obs_19", line_index),
                    self.get_float(row, "obs_20", line_index),
                    self.get_float(row, "obs_21", line_index),
                    self.get_float(row, "obs_22", line_index),
                    self.get_float(row, "obs_23", line_index),
                    self.get_float(row, "obs_24", line_index),
                    self.get_float(row, "obs_25", line_index),
                    self.get_float(row, "obs_26", line_index),
                ]

                joint_vel = [
                    self.get_float(row, "obs_27", line_index),
                    self.get_float(row, "obs_28", line_index),
                    self.get_float(row, "obs_29", line_index),
                    self.get_float(row, "obs_30", line_index),
                    self.get_float(row, "obs_31", line_index),
                    self.get_float(row, "obs_32", line_index),
                    self.get_float(row, "obs_33", line_index),
                    self.get_float(row, "obs_34", line_index),
                    self.get_float(row, "obs_35", line_index),
                    self.get_float(row, "obs_36", line_index),
                    self.get_float(row, "obs_37", line_index),
                    self.get_float(row, "obs_38", line_index),
                    self.get_float(row, "obs_39", line_index),
                    self.get_float(row, "obs_40", line_index),
                ]

                # TO change if the LiDAR is used or not

                # lidar_data = [
                #      self.get_float(row, f"obs_{i}", line_index)
                #     for i in range(55, 396)
                # ]
                lidar_data = []


                rows.append({
                    "imu": imu_data,
                    "cmd": cmd_data,
                    "joint_pos": joint_pos,
                    "joint_vel": joint_vel,
                    "lidar": lidar_data,
                })

        if not rows:
            raise RuntimeError("CSV file is empty")

        return rows

    # Build the message for the /imu topic
    def build_imu_msg(self, data, stamp):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"

        msg.linear_acceleration.x = data["base_lin_vel"][0]
        msg.linear_acceleration.y = data["base_lin_vel"][1]
        msg.linear_acceleration.z = data["base_lin_vel"][2]

        msg.angular_velocity.x = data["base_ang_vel"][0]
        msg.angular_velocity.y = data["base_ang_vel"][1]
        msg.angular_velocity.z = data["base_ang_vel"][2]

        msg.orientation.x = data["projected_gravity"][0]
        msg.orientation.y = data["projected_gravity"][1]
        msg.orientation.z = data["projected_gravity"][2]
        msg.orientation.w = 0.0

        return msg
    
    # Build the message for the /cmd topic
    def build_cmd_msg(self, data):
        msg = Float32MultiArray()
        msg.data = data
        return msg

    # Build the message for the /joint_states topic
    def build_joint_state_msg(self, positions, velocities, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = velocities
        msg.effort = []
        return msg

    # Build the message for the /lidar topic
    def build_lidar_msg(self, data):
        msg = Float32MultiArray()
        msg.data = data
        return msg

    # Sending all data to the correct topic
    def publish_row(self, row):
        stamp = rospy.Time.now()

        self.pub_imu.publish(self.build_imu_msg(row["imu"], stamp))
        self.pub_cmd.publish(self.build_cmd_msg(row["cmd"]))

        self.pub_joint_states.publish(
            self.build_joint_state_msg(
                row["joint_pos"],
                row["joint_vel"],
                stamp
            )
        )

        self.pub_lidar.publish(
            self.build_lidar_msg(row["lidar"])
        )

    # Use to run the node at a specific rate
    def run(self):
        rate = rospy.Rate(self.publish_hz)

        while not rospy.is_shutdown():
            for row in self.data:
                if rospy.is_shutdown():
                    break

                self.publish_row(row)
                rate.sleep()

            if not self.loop:
                rospy.loginfo("Finished publishing fake observation data sequence")
                break


# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    try:
        node = QuadrupedFakeObsDatasNode()
        node.run()

    except rospy.ROSInterruptException:
        pass

    except Exception as e:
        rospy.logerr("Node failed: %s", str(e))