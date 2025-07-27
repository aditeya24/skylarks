#!/usr/bin/env python3

import asyncio
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aiortc import RTCPeerConnection, VideoStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
import av

class VideoStream(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.frame = None
        rospy.init_node("ros_webrtc_stream", anonymous=True)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        if self.frame is None:
            return av.VideoFrame.from_ndarray(np.zeros((480, 640, 3), dtype=np.uint8), format="bgr24")

        frame = av.VideoFrame.from_ndarray(self.frame, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base
        return frame

async def webrtc_server():
    pc = RTCPeerConnection()
    signaling = TcpSocketSignaling("0.0.0.0", 1234)  # WebRTC Signaling Server

    video_stream = VideoStream()
    pc.addTrack(video_stream)

    await signaling.connect()
    offer = await signaling.receive()
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    await signaling.send(pc.localDescription)

    await asyncio.sleep(3600)  # Keep the server running

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(webrtc_server())
