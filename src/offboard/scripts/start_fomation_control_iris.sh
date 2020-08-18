#!/bin/bash

gnome-terminal -x python offboard_leader_iris0.py  &
gnome-terminal -x python offboard_follower_iris1.py &
gnome-terminal -x python offboard_follower_iris2.py &

