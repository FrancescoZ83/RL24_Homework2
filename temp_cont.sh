#!/bin/bash

source install/setup.bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort -p traj_type:=lin_trap
