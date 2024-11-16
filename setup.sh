#!/bin/bash

sudo apt update
sudo apt-get install libeigen3-dev libcppunit-dev
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
