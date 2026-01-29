#!/bin/bash
set -e

DEFAULT_IP="100.71.135.108"
echo -n "Enter Target IP (Default: $DEFAULT_IP): "
read INPUT_IP

if [ -z "$INPUT_IP" ]; then
    TARGET_IP="$DEFAULT_IP"
else
    TARGET_IP="$INPUT_IP"
fi

echo ">> Streaming to: $TARGET_IP"
echo ""

echo ">> Killing old GStreamer processes..."
pkill -f gst-launch || true

echo ">> Setting up Kernel Modules..."
sudo modprobe -r v4l2loopback
sudo modprobe v4l2loopback devices=2 video_nr=40,41 card_label='StreamFeed','VisionFeed' exclusive_caps=1,1

sleep 1

if [ ! -e /dev/video40 ]; then
    echo "CRITICAL ERROR: /dev/video40 was not created. Check v4l2loopback install."
    exit 1
fi
echo ">> Virtual Devices Ready."

SESSION="skylarks_video"

tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION

SPLIT_CMD="gst-launch-1.0 -e v4l2src device=/dev/video0 io-mode=2 ! image/jpeg,width=1280,height=800,framerate=60/1 ! jpegdec ! videoconvert ! videobalance brightness=-0.10 contrast=0.95 ! tee name=t t. ! queue leaky=2 ! v4l2sink device=/dev/video40 t. ! queue leaky=2 ! v4l2sink device=/dev/video41"

tmux send-keys -t $SESSION:0.0 "$SPLIT_CMD" C-m

tmux split-window -h -t $SESSION:0

STREAM_CMD="gst-launch-1.0 v4l2src device=/dev/video40 ! videoscale ! video/x-raw,width=640,height=400 ! videoconvert ! x264enc bitrate=2000 speed-preset=superfast tune=zerolatency ! rtph264pay ! udpsink host=$TARGET_IP port=5000"

tmux send-keys -t $SESSION:0.1 "sleep 2 && $STREAM_CMD" C-m

tmux select-pane -t $SESSION:0.0
tmux attach-session -t $SESSION