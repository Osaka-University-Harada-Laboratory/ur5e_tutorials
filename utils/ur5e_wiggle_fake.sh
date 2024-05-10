#!/bin/bash

byobu new-session -d -s wiggle
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0

byobu send-keys -t 0 'xhost + && docker exec -it ur5e_container bash -it -c "roslaunch ur5e_moveit_config demo.launch"' 'C-m'
sleep 3.
byobu send-keys -t 1 'xhost + && docker exec -it ur5e_container bash -it -c "roslaunch ur5e_tutorials wiggle.launch fake_execution:=true"' 'C-m'

byobu attach -t wiggle