#!/bin/bash

ps -ef|grep uav0_consensus_test.py|grep -v grep|cut -c 9-15|xargs kill -9 &&
ps -ef|grep uav1_consensus_test.py|grep -v grep|cut -c 9-15|xargs kill -9 &&
ps -ef|grep uav2_consensus_test.py|grep -v grep|cut -c 9-15|xargs kill -9
