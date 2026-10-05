#!/bin/bash

# Project Name : ContinuO - Quadruped Robot
# Description : This script start all the nodes and launch the ROS environment for the real quadruped robot in order to test the positions to robot part. 
# Author : Florent Pralong
# Date of creation : 10/06/2026
# Version : 1.0

#-----------------------------------------------------
# 01-Stop motors by calling quadruped_motor_stop_all.py
#-----------------------------------------------------

echo ""
echo "========================================================================="
echo " Confirm ContinuO is on the ground in the zero position before proceeding."
echo "========================================================================="
echo ""

# User confirmation
read -p "Continue? (y/n): " proceed

if [[ "$proceed" != "y" && "$proceed" != "Y" ]]; then
    echo "Aborted."
    exit 1
fi

# STOP ALL MOTORS
echo "Stopping all motors..."

source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/user/ContinuO_FP/ContinuO-python-scripts-main/myactuator_rmd/build/lib.linux-x86_64-3.8

if ! python3 ~/ContinuO_FP/catkin_ws_fp/src/quadruped_real/scripts/tools/quadruped_motor_stop_all.py; then
    echo "ERROR: stop_all_motors.py failed."
    exit 1
fi

echo "All motors stopped."

#-----------------------------------------------------
# 02-Execute the quadruped_real.launch
#-----------------------------------------------------

echo ""
echo "========================================================================="
echo " Confirm ContinuO is suspended and can move freely before proceeding."
echo "========================================================================="
echo ""

# User confirmation
read -p "Continue? (y/n): " proceed

if [[ "$proceed" != "y" && "$proceed" != "Y" ]]; then
    echo "Aborted."
    exit 1
fi

# SOURCE ROS ENVIRONMENT
source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash

# START required nodes

# MAIN node (clock node is stared in the .launch file)
gnome-terminal -- bash -c "
source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
exec -a continuo_real roslaunch quadruped_real quadruped_real.launch
"
# EMERGENCY STOP node
gnome-terminal -- bash -c "
source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
exec -a continuo_estop rosrun quadruped_real quadruped_emergency_keyboard_node.py
"

echo "Initialization position command sent."

# Wait initialization to take effect
sleep 15

echo "ContinuO is now ready to run a control policy."


#-----------------------------------------------------
# 03-Run the /quadruped_isaaclab_fakedatas_node
#-----------------------------------------------------

echo ""
echo "========================================================================="
echo " Confirm ContinuO is ready to run a control policy before proceeding."
echo "========================================================================="
echo ""

# User confirmation
read -p "Continue? (y/n): " proceed

if [[ "$proceed" != "y" && "$proceed" != "Y" ]]; then
    echo "Aborted."
    exit 1
fi

# START required nodes

# FakeDatas node
gnome-terminal -- bash -c "
source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
exec -a continuo_fakedatas rosrun quadruped_real quadruped_isaaclab_fakedatas_node.py
"

# Plotter node (for development only)
gnome-terminal -- bash -c "
source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash
exec -a plotter rosrun quadruped_real joint_positions_plotter.py
"

#-----------------------------------------------------
# 04-Shutdown and stop all the nodes 
#-----------------------------------------------------

echo ""
echo "========================================================================="
echo " Quit and shut down all ContinuO ROS nodes?"
echo "========================================================================="
echo ""

# User confirmation
read -p "Quit? (y/n): " quit_answer

if [[ "$quit_answer" == "y" || "$quit_answer" == "Y" ]]; then
    echo "Stopping ContinuO ROS launch processes..."

    pkill -SIGINT -f "plotter"
    sleep 2

    pkill -SIGINT -f "continuo_fakedatas"
    sleep 2

    pkill -SIGINT -f "continuo_real"
    sleep 2
    
    pkill -SIGINT -f "continuo_estop"
    sleep 2

    echo "Killing remaining ROS nodes..."

    source ~/ContinuO_FP/catkin_ws_fp/devel/setup.bash

    rosnode kill -a 2>/dev/null

    sleep 2

    echo "Stopping roscore/rosmaster if still running..."

    pkill -SIGINT -f roscore
    pkill -SIGINT -f rosmaster

    echo "Shutdown complete."
else
    echo "Leaving ROS nodes running."
fi