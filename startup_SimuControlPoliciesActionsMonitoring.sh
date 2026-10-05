#!/bin/bash

# Project Name : ContinuO - Quadruped Robot
# Description : This script start all the nodes required to test the monitoring of the commands sent to ContinuO.
# Author : Florent Pralong
# Date of creation : 23/06/2026
# Version : 1.0

# START ROS

# MAIN node 
gnome-terminal -- bash -c "
source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
exec -a roscore roscore
"
# START Visu node
gnome-terminal -- bash -c "
source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
exec -a visu_node rosrun quadruped_monitoring commands_visu_node.py
"

# START Switch policy node
gnome-terminal -- bash -c "
source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
exec -a switchcp rosrun quadruped_control quadruped_switch_policy_node.py
"

sleep 2

# START control node
gnome-terminal -- bash -c "
source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
exec -a control_node rosrun quadruped_control policy_node.py
"

sleep 2

# START fake obs node
gnome-terminal -- bash -c "
source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
exec -a fake_obs_node rosrun quadruped_control quadruped_fake_obs_datas.py
"



# # FakeDatas node
# gnome-terminal -- bash -c "
# source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
# exec -a continuo_fakedatas rosrun quadruped_real quadruped_isaaclab_fakedatas_node.py
# "