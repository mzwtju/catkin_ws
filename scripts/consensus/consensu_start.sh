#!/bin/bash

gnome-terminal -x python ~/catkin_ws/scripts/consensus/uav0_consensus_test.py  &
gnome-terminal -x python ~/catkin_ws/scripts/consensus/uav1_consensus_test.py &
gnome-terminal -x python ~/catkin_ws/scripts/consensus/uav2_consensus_test.py &
gnome-terminal -x rosrun rviz rviz