#!/bin/bash

byobu new-session -d -s moveit
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0

byobu send-keys -t 0 'xhost + && docker exec -it ur5e_container bash -it -c "roslaunch ur5e_moveit_config demo.launch"' 'C-m'

byobu attach -t moveit