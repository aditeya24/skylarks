#!/bin/bash

sudo modprobe -r v4l2loopback
sudo modprobe -r uvcvideo && sudo modprobe uvcvideo
sudo modprobe v4l2loopback devices=1 video_nr=3 card_label="Virtual Camera" exclusive_caps=1
sudo systemctl restart stream.service
v4l2-ctl --list-devices
