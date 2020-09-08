#!/bin/bash

gnome-terminal -x python ~/test_consensus/catkin_ws/scripts/consensus/hunt_uav0.py  &
gnome-terminal -x python ~/test_consensus/catkin_ws/scripts/consensus/hunt_uav1.py &
gnome-terminal -x python ~/test_consensus/catkin_ws/scripts/consensus/hunt_uav2.py &
gnome-terminal -x python ~/test_consensus/catkin_ws/scripts/consensus/hunt_uav3.py &
gnome-terminal -x rosrun rviz rviz