#!/bin/bash

gnome-terminal -x python offboard_leader_uav0.py  &
gnome-terminal -x python offboard_follower_uav1.py &
gnome-terminal -x python offboard_follower_uav2.py &

