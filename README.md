# Skylarks 2.0 — Autonomous UAV Software Stack

<p align="left">
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge&logo=ros" />
  <img src="https://img.shields.io/badge/Python-3.x-3776AB?style=for-the-badge&logo=python&logoColor=white" />
  <img src="https://img.shields.io/badge/Platform-Raspberry%20Pi%204-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=white" />
  <img src="https://img.shields.io/badge/Flight-ArduPilot-00979D?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Interface-MAVROS-6C2DC7?style=for-the-badge" />
</p>

Autonomous drone software stack built using ROS 2 for GPS navigation, QR-based landing, payload delivery, and return-to-launch.  
Designed for deployment on a Raspberry Pi companion computer communicating with a Pixhawk flight controller.

## Architecture
<p align="center">
  <img src="docs/images/architecture.svg" width="900"/>
</p>


High-level ROS2 autonomy architecture showing perception, control, payload actuation, and MAVROS bridge.

## Nodes

- `drone_controller` — mission state machine  
- `qr_detector` — QR detection node  
- `payload_driver` — servo action server  
- `mavros` — flight controller bridge  

## Mission Flow

```
INIT
  ↓
TAKEOFF
  ↓
TRANSIT
  ↓
SEARCH ──────┐
  ↓          │
QR DETECTED  │ timeout
  ↓          │
LAND <───────┘
  ↓
DROP
  ↓
RTL
```

## Repository Structure

```
.
├── camera.sh            # video split + streaming pipeline
├── exposure.sh          # camera exposure configuration
├── start_mission.sh     # launch full autonomy stack
│
├── docs/
│   └── images/
│       └── architecture.svg
│
└── src/
    ├── drone_control/   # mission FSM + MAVROS control
    ├── vision/          # QR detection node
    ├── payload/         # payload action server
    └── interfaces/      # ROS2 msg + action definitions
```


## Hardware

- Raspberry Pi 4 (companion computer)
- Pixhawk Cube (ArduPilot)
- Arducam OV9281 global shutter camera (1280×800 @ 60 FPS)
- MG90S servo payload mechanism
- GPS module (connected to flight controller)
- USB MAVLink connection (MAVROS)

## Dependencies

- Ubuntu 22.04  
- ROS 2 Humble  
- MAVROS  
- GStreamer  
- v4l2loopback  
- pymap3d  
- pyzbar  

## Setup (Raspberry Pi)

Install ROS2 base

```bash
sudo apt install ros-humble-ros-base
sudo apt install python3-colcon-common-extensions python3-rosdep
```

Clone repository

```bash
git clone https://github.com/aditeya24/skylarks
cd skylarks
```

Install dependencies

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
```

Install video pipeline dependencies

```bash
sudo apt install v4l2loopback-dkms \
gstreamer1.0-tools \
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad
```

Build workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

## Camera Setup

Create virtual cameras

```bash
sudo modprobe v4l2loopback devices=2 video_nr=40,41 \
card_label="StreamFeed","VisionFeed" exclusive_caps=1,1
```

Split camera feed

```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! \
tee name=t ! queue ! v4l2sink device=/dev/video40 \
t. ! queue ! v4l2sink device=/dev/video41
```


## Mission Workflow

Before running a mission, three scripts are used.

### 1. Configure Camera Exposure

```bash
./exposure.sh 4
```

Sets manual exposure and camera parameters for QR detection.

### 2. Start Video Pipeline

```bash
./camera.sh
```

This script:

- creates virtual cameras using v4l2loopback  
- splits camera feed using gstreamer  
- streams video over UDP  
- opens tmux monitoring session  

Devices created

```
/dev/video40  → stream output
/dev/video41  → vision processing
```

### 3. Start Mission

```bash
./start_mission.sh
```

Launches:

- MAVROS  
- drone controller  
- payload driver  
- vision node  
- system monitor

You will be prompted for

- target latitude  
- target longitude  

Mission execution is handled inside a tmux session.


## Video Streaming (Optional)

Stream

```bash
gst-launch-1.0 -v v4l2src device=/dev/video40 ! videoconvert ! \
videoscale ! video/x-raw,width=640,height=400 ! videoconvert ! \
x264enc bitrate=2000 speed-preset=superfast tune=zerolatency ! \
rtph264pay ! udpsink host=<tailscale-ip> port=5000
```

Receive

```bash
gst-launch-1.0 udpsrc port=5000 \
caps="application/x-rtp, media=video, encoding-name=H264, payload=96" \
! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```
