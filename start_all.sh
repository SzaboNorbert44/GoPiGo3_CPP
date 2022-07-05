#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient

catkin_make

gnome-terminal -- ./start_gazebo.sh
sleep 6
gnome-terminal -- ./start_rviz.sh
sleep 7
gnome-terminal -- ./start_converter.sh
sleep 2

 ./start_program.sh


