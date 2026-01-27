#!/usr/bin/env bash
set -e

SESSION="Vision"

# ---- Commands ----

CREATE_VCAM_CMD='
sudo modprobe -r v4l2loopback || true
sleep 1
sudo modprobe v4l2loopback devices=2 video_nr=40,41 \
card_label="StreamFeed","VisionFeed" exclusive_caps=1,1
'

SPLIT_PIPELINE_CMD='
gst-launch-1.0 -e \
v4l2src device=/dev/video0 io-mode=2 ! \
image/jpeg,width=1280,height=800,framerate=60/1 ! \
jpegdec ! videoconvert ! \
videobalance brightness=-0.10 contrast=0.95 ! \
tee name=t \
t. ! queue leaky=2 ! v4l2sink device=/dev/video40 \
t. ! queue leaky=2 ! v4l2sink device=/dev/video41
'

STREAM_CMD='
gst-launch-1.0 \
v4l2src device=/dev/video40 ! \
videoscale ! video/x-raw,width=640,height=400 ! \
videoconvert ! \
x264enc bitrate=2000 speed-preset=superfast tune=zerolatency ! \
rtph264pay ! \
udpsink host=100.71.135.108 port=5000
'

# ---- Cleanup ----
tmux kill-session -t $SESSION 2>/dev/null || true
pkill -f gst-launch || true

# ---- Create tmux session ----
tmux new-session -d -s $SESSION -n pipelines

# Pane 0: virtual cameras + split pipeline
tmux send-keys -t $SESSION:pipelines "
$CREATE_VCAM_CMD
sleep 2
$SPLIT_PIPELINE_CMD
" C-m

# Split window
tmux split-window -h -t $SESSION:pipelines

# Pane 1: streaming pipeline
tmux send-keys -t $SESSION:pipelines "
sleep 4
$STREAM_CMD
" C-m

# Layout
tmux select-layout -t $SESSION tiled

# Attach
tmux attach -t $SESSION