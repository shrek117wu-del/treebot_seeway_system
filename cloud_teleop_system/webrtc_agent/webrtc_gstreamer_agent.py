#!/usr/bin/env python3
"""
Hardware-Accelerated WebRTC ROS 2 Agent
Uses GStreamer webrtcbin and nvv4l2h265enc for ultra-low latency video streaming
and DataChannels for sub-10ms control.

Prerequisites on Jetson:
  sudo apt install libgirepository1.0-dev gcc libcairo2-dev pkg-config python3-dev gir1.2-gtk-3.0
  sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly
  pip3 install websockets PyGObject
"""

import sys
import json
import asyncio
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GstWebRTC, GLib
import websockets

# Match the new signaling server pairing format
SIGNALING_SERVER_URL = "ws://127.0.0.1:8765/?room=room1&role=robot"

class WebRTCGStreamerAgent(Node):
    def __init__(self):
        super().__init__('webrtc_gstreamer_agent')
        self.joy_pub = self.create_publisher(Joy, '/teleop_joy', 10)
        
        Gst.init(None)
        
        # Hardware accelerated GStreamer pipeline for Jetson
        # It takes ROS 2 raw images (or another source), hardware-encodes them to H.265, and wraps in RTP
        # For demonstration, we use a v4l2src. To use a ROS image topic, replace v4l2src with appsrc and feed frames.
        # Here we use an NVIDIA accelerated pipeline assuming a V4L2 camera or Jetson's nvarguscamerasrc.
        # Modify the source ('nvarguscamerasrc' or 'v4l2src') based on your exact camera setup.
        PIPELINE_DESC = (
            "webrtcbin name=webrtc bundle-policy=max-bundle "
            "v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 ! "
            "nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! "
            "nvv4l2h264enc bitrate=1500000 insert-sps-pps=true ! "
            "h264parse ! rtph264pay config-interval=-1 aggregate-mode=zero-latency ! "
            "application/x-rtp,media=video,encoding-name=H264,payload=96 ! webrtc. "
        )
        
        self.get_logger().info(f"Starting GStreamer pipeline: {PIPELINE_DESC}")
        self.pipeline = Gst.parse_launch(PIPELINE_DESC)
        self.webrtc = self.pipeline.get_by_name('webrtc')
        
        # Connect WebRTC signals
        self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
        self.webrtc.connect('on-ice-candidate', self.send_ice_candidate_message)
        self.webrtc.connect('on-data-channel', self.on_data_channel)
        
        self.pipeline.set_state(Gst.State.PLAYING)
        self.ws = None
        self.loop = asyncio.get_event_loop()

    def publish_joy(self, linear_x, angular_z, buttons):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [float(linear_x), 0.0, float(angular_z), 0.0]
        msg.buttons = [int(b) for b in buttons]
        self.joy_pub.publish(msg)

    # --- GStreamer WebRTC Callbacks ---

    def on_negotiation_needed(self, element):
        promise = Gst.Promise.new_with_change_func(self.on_offer_created, element, None)
        element.emit('create-offer', None, promise)

    def on_offer_created(self, promise, element, _):
        promise.wait()
        reply = promise.get_reply()
        offer = reply.get_value('offer')
        promise = Gst.Promise.new()
        element.emit('set-local-description', offer, promise)
        promise.interrupt()
        self.send_sdp_message(offer)

    def send_sdp_message(self, desc):
        if self.ws:
            text = desc.sdp.as_text()
            msg = {'type': 'offer' if desc.type == GstWebRTC.WebRTCSDPType.OFFER else 'answer', 'sdp': text}
            asyncio.run_coroutine_threadsafe(self.ws.send(json.dumps(msg)), self.loop)

    def send_ice_candidate_message(self, element, mlineindex, candidate):
        if self.ws:
            msg = {'candidate': {'candidate': candidate, 'sdpMLineIndex': mlineindex}}
            asyncio.run_coroutine_threadsafe(self.ws.send(json.dumps(msg)), self.loop)

    def on_data_channel(self, webrtc, channel):
        self.get_logger().info(f"DataChannel received: {channel.get_property('label')}")
        channel.connect('on-message-string', self.on_data_channel_message)

    def on_data_channel_message(self, channel, msg_str):
        try:
            data = json.loads(msg_str)
            self.publish_joy(data['linear']['x'], data['angular']['z'], data['buttons'])
        except Exception as e:
            self.get_logger().error(f"Failed to parse control msg: {e}")

    # --- WebSocket Signaling ---

    async def connect_signaling(self):
        async with websockets.connect(SIGNALING_SERVER_URL) as ws:
            self.ws = ws
            self.get_logger().info(f"Connected to signaling server as robot: {SIGNALING_SERVER_URL}")
            
            async for message in ws:
                msg = json.loads(message)
                if 'sdp' in msg:
                    sdp_type = GstWebRTC.WebRTCSDPType.OFFER if msg['type'] == 'offer' else GstWebRTC.WebRTCSDPType.ANSWER
                    res, sdpmsg = GstWebRTC.WebRTCSDPMessage.new_from_text(sdp_type, msg['sdp'])
                    desc = GstWebRTC.WebRTCSessionDescription.new(sdp_type, sdpmsg)
                    
                    promise = Gst.Promise.new_with_change_func(self.on_set_remote_description_complete, self.webrtc, None)
                    self.webrtc.emit('set-remote-description', desc, promise)
                
                elif 'candidate' in msg:
                    cand = msg['candidate']
                    self.webrtc.emit('add-ice-candidate', cand['sdpMLineIndex'], cand['candidate'])

    def on_set_remote_description_complete(self, promise, element, _):
        promise.wait()
        # If we received an offer, we must create an answer
        if element.get_property('signaling-state') == GstWebRTC.WebRTCSignalingState.HAVE_REMOTE_OFFER:
            ans_promise = Gst.Promise.new_with_change_func(self.on_answer_created, element, None)
            element.emit('create-answer', None, ans_promise)

    def on_answer_created(self, promise, element, _):
        promise.wait()
        reply = promise.get_reply()
        answer = reply.get_value('answer')
        set_promise = Gst.Promise.new()
        element.emit('set-local-description', answer, set_promise)
        set_promise.interrupt()
        self.send_sdp_message(answer)


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCGStreamerAgent()
    
    # Run ROS2 spin in a background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Run GLib MainLoop (required by GStreamer) in another thread
    glib_loop = GLib.MainLoop()
    glib_thread = threading.Thread(target=glib_loop.run, daemon=True)
    glib_thread.start()

    # Run asyncio WebSocket loop in main thread
    try:
        node.loop.run_until_complete(node.connect_signaling())
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
