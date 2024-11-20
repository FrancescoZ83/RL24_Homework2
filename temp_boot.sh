#!/bin/bash

colcon build
source install/setup.bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="effort" robot_controller:="effort_controller"
