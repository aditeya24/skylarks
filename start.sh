#!/bin/bash

echo "1) Bright Sun (Fast Shutter)"
echo "2) Cloudy/Evening (Slow Shutter)"
echo -n "Select Lighting [1 or 2]: "
read LIGHT

if [ -e "/dev/real_cam" ]; then
    CAM_DEV="/dev/real_cam"
elif [ -e "/dev/video0" ]; then
    CAM_DEV="/dev/video0"
else
    echo "ERROR: No camera device found! Skipping exposure setup."
    CAM_DEV=""
fi


if [ ! -z "$CAM_DEV" ]; then
    echo "Configuring Camera at $CAM_DEV..."
    
    # We use 'try/catch' style by using || true so script doesn't crash if camera is busy
    if [ "$LIGHT" == "1" ]; then
        v4l2-ctl -d $CAM_DEV --set-ctrl=auto_exposure=1 || true
        v4l2-ctl -d $CAM_DEV --set-ctrl=exposure_time_absolute=1 || true
        echo "--> Exposure set to 1 (Sun)"
    else
        v4l2-ctl -d $CAM_DEV --set-ctrl=auto_exposure=1 || true
        v4l2-ctl -d $CAM_DEV --set-ctrl=exposure_time_absolute=150 || true
        echo "--> Exposure set to 150 (Cloud)"
    fi
fi
echo ""

echo -n "Enter Target LATITUDE: "
read TARGET_LAT

echo -n "Enter Target LONGITUDE: "
read TARGET_LON

SESSION="skylarks_mission"

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION

# Pane 1 - Drone Control
tmux send-keys -t $SESSION:0.0 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:0.0 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:0.0 "clear" C-m

if [ -z "$TARGET_LAT" ]; then TARGET_LAT="360.0"; fi
if [ -z "$TARGET_LON" ]; then TARGET_LON="360.0"; fi

LAUNCH_CMD="ros2 launch drone_control mission.launch.py target_lat:=$TARGET_LAT target_lon:=$TARGET_LON"

tmux send-keys -t $SESSION:0.0 "$LAUNCH_CMD"

# Pane 2 - Vision
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "clear" C-m
tmux send-keys -t $SESSION:0.1 "ros2 topic echo /vision/target_deviation --qos-reliability best_effort" C-m

# Pane 3 - System Monitoring
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.2 "btop" C-m

tmux resize-pane -t $SESSION:0.0 -x 60%

tmux select-pane -t $SESSION:0.0
tmux attach-session -t $SESSION