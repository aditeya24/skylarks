#!/bin/bash

# Default to 4 if no argument provided
EXP_TIME=${1:-4}

if [ "$EXP_TIME" -lt 1 ]; then
  EXP_TIME=1
elif [ "$EXP_TIME" -gt 300 ]; then
  EXP_TIME=300
fi

echo "Setting Camera Exposure to: $EXP_TIME"

v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1

v4l2-ctl -d /dev/video0 \
  --set-ctrl=exposure_time_absolute=$EXP_TIME \
  --set-ctrl=gain=0 \
  --set-ctrl=brightness=-5 \
  --set-ctrl=contrast=48 \
  --set-ctrl=gamma=90 \
  --set-ctrl=sharpness=3 \
  --set-ctrl=white_balance_automatic=0 \
  --set-ctrl=white_balance_temperature=4800

echo "Camera configured."