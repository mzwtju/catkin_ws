kill -9 $(ps -ef|grep follower_accel_control.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep leader.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
