#!/usr/bin/env python3
import asyncio
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaRelay
from av import VideoFrame
import websockets

SIGNALING_SERVER_URL = "ws://127.0.0.1:8765"

class ROSVideoStreamTrack(VideoStreamTrack):
    """
    A WebRTC video track that reads frames from a ROS2 Image topic.
    For ultra-low latency on Jetson, this should ideally be replaced with a GStreamer
    webrtcbin pipeline utilizing nvv4l2h265enc hardware acceleration.
    """
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        self.latest_frame = None
        self.sub = self.node.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Convert OpenCV BGR to PyAV VideoFrame (aiortc uses PyAV)
        frame = VideoFrame.from_ndarray(cv_image, format='bgr24')
        self.latest_frame = frame

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        
        # If no frame yet, yield a blank frame or loop
        while self.latest_frame is None:
            await asyncio.sleep(0.01)

        frame = self.latest_frame
        frame.pts = pts
        frame.time_base = time_base
        return frame


class WebRTCAgentNode(Node):
    def __init__(self):
        super().__init__('webrtc_ros_agent')
        self.joy_pub = self.create_publisher(Joy, '/teleop_joy', 10)
        self.get_logger().info('WebRTC Agent Node Initialized')

    def publish_joy(self, linear_x, angular_z, buttons):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Map back to joy axes: [0] = linear_x, [1] = linear_y, [2] = angular_z
        # This matches standard PS/Xbox mapping for Twist translation
        msg.axes = [float(linear_x), 0.0, float(angular_z), 0.0]
        msg.buttons = [int(b) for b in buttons]
        self.joy_pub.publish(msg)


async def run_webrtc(ros_node):
    pc = RTCPeerConnection()

    # Add video track
    video_track = ROSVideoStreamTrack(ros_node)
    pc.addTrack(video_track)

    @pc.on("datachannel")
    def on_datachannel(channel):
        if channel.label == "teleop_controls":
            ros_node.get_logger().info("DataChannel connected! Ready for sub-10ms control.")
            
            @channel.on("message")
            def on_message(message):
                try:
                    data = json.loads(message)
                    # Convert JSON dict to ROS Joy and publish
                    ros_node.publish_joy(
                        data['linear']['x'], 
                        data['angular']['z'], 
                        data['buttons']
                    )
                except Exception as e:
                    ros_node.get_logger().error(f"Failed to parse control msg: {e}")

    # Connect to Signaling
    async with websockets.connect(SIGNALING_SERVER_URL) as ws:
        ros_node.get_logger().info(f"Connected to signaling server at {SIGNALING_SERVER_URL}")
        
        # Handle incoming SDP offers
        async for message in ws:
            obj = json.loads(message)
            if "type" in obj and obj["type"] == "offer":
                ros_node.get_logger().info("Received Offer. Setting remote description...")
                await pc.setRemoteDescription(RTCSessionDescription(
                    sdp=obj["sdp"], type=obj["type"]
                ))
                
                ros_node.get_logger().info("Creating Answer...")
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                
                await ws.send(json.dumps({
                    "type": pc.localDescription.type,
                    "sdp": pc.localDescription.sdp
                }))
            
            elif "candidate" in obj:
                # ICE Candidate handling omitted for brevity in Python aiortc API
                pass


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCAgentNode()

    # Run ROS2 spin in a separate thread so asyncio event loop doesn't block
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run_webrtc(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
