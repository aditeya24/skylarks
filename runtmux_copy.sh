#!/bin/bash
SESSION_NAME="sample1"
WINDOW_NAME="window1"

# Create a new tmux session and name the window
tmux new-session -d -s sample1 -n window1

# Split the window into three panes
# Create the first pane (pane 0) automatically when the session is created

# Split the first pane horizontally to create pane 1
tmux split-window -h -t sample1:window1.0

# Split pane 1 vertically to create pane 2
tmux split-window -v -t sample1:window1.1

# Split pane 0 vertically to create pane 3
tmux split-window -v -t sample1:window1.0

# Split pane 3 vertically to create pane 4
tmux split-window -v -t sample1:window1.3

# Split pane 0 vertically to create pane 5
tmux split-window -v -t sample1:window1.0

# Send the roscore command to pane 0 of the window
tmux send-keys -t sample1:window1.0 "roscore" Enter

# Pause for 5 seconds before sending the next command
sleep 5

# Send the roslaunch mavros command to pane 1 of the window
tmux send-keys -t sample1:window1.1 "roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0" Enter
sleep 5

# Source and launch camera_qr in pane 3
#tmux send-keys -t sample1:window1.3 "source drone_ws/devel/setup.bash" Enter
sleep 2
tmux send-keys -t sample1:window1.3 "roslaunch camera_qr camera_qr.launch" Enter

# Run virtual camera node
tmux send-keys -t sample1:window1.4 "rosrun camera_qr ros_virtual_camera.py" Enter

# Run payload node
tmux send-keys -t sample1:window1.5 "rosrun servo_control motor_control.py" Enter
sleep 2

# Send keys for waypoint node
tmux send-keys -t sample1:window1.2 "rosrun drone_control mavros_waypoint.py"

# Restart stream service
sudo systemctl restart stream.service &

# Move cursor to pane 2
tmux select-pane -t sample1:window1.2

# Attach to the tmux session
tmux attach-session -t sample1

