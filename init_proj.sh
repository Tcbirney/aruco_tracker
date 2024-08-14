#!/bin/bash

ws_dir="$(pwd)/catkin_ws"
cd $ws_dir
catkin_make
grep -qxF "source $ws_dir/devel/setup.bash" ~/.bashrc || echo "source $ws_dir/devel/setup.bash" >> ~/.bashrc
grep -qxF "export TURTLEBOT3_MODEL=burger" ~/.bashrc || echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

sudo apt update
sudo apt upgrade -y

sudo apt install ros-noetic-aruco-ros ros-noetic-map-server ros-noetic-amcl \
                 ros-noetic-move-base ros-noetic-gmapping -y
