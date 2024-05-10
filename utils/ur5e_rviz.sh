#!/bin/bash

byobu new-session -d -s rviz
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0

byobu send-keys -t 0 'xhost + && docker exec -it ur5e_container bash -it -c "roslaunch ur_description view_ur5e.launch
"' 'C-m'

byobu attach -t rviz