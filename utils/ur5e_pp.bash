#!/bin/bash

byobu new-session -d -s pp
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0
byobu split-window -h
byobu select-pane -t 2
byobu split-window -h
byobu select-pane -t 3

byobu send-keys -t 0 'xhost + && docker exec -it ur5e_container bash -it -c "roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch sim:=false"' 'C-m'
sleep 5.
byobu send-keys -t 1 'xhost + && docker exec -it ur5e_container bash -it -c "roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=/catkin_ws/src/universal_robot/ur5e_moveit_config/launch/moveit.rviz"' 'C-m'
sleep 5.
byobu send-keys -t 2 'xhost + && docker exec -it ur5e_container bash -it -c "roslaunch ur5e_tutorials pick_and_place.launch use_gripper:=true"' 'C-m'

byobu attach -t pp