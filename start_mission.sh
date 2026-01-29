#!/bin/bash


echo -n "Enter Target LATITUDE: "
read TARGET_LAT

echo -n "Enter Target LONGITUDE: "
read TARGET_LON

SESSION="skylarks_mission"

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION

# Pane 1 - Drone Control
tmux send-keys -t $SESSION:0.0 "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:0.0 "clear" C-m

if [ -z "$TARGET_LAT" ]; then TARGET_LAT="360.0"; fi
if [ -z "$TARGET_LON" ]; then TARGET_LON="360.0"; fi

LAUNCH_CMD="ros2 launch drone_control mission.launch.py target_lat:=$TARGET_LAT target_lon:=$TARGET_LON"

tmux send-keys -t $SESSION:0.0 "$LAUNCH_CMD"

# Pane 2 - Vision
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "clear" C-m
tmux send-keys -t $SESSION:0.1 "ros2 run vision qr_detector" C-m

# Pane 3 - System Monitoring
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.2 "btop" C-m

tmux resize-pane -t $SESSION:0.0 -x 60%

tmux select-pane -t $SESSION:0.0
tmux attach-session -t $SESSION