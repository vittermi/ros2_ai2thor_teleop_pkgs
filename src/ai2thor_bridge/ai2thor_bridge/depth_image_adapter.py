import rclpy # type: ignore
from rclpy.node import Node # type: ignore

import redis
import threading
import json
import base64
import numpy as np
import math

from sensor_msgs.msg import Image # type: ignore
from sensor_msgs.msg import CameraInfo # type: ignore
from std_msgs.msg import Header # type: ignore
from builtin_interfaces.msg import Time


class DepthImageAdapter(Node):
    def __init__(self):
        super().__init__('depth_image_adapter')

        self.declare_parameter('redis_channel_depth', 'ai2thor_depth_image')
        self.declare_parameter('redis_channel_rgb', 'ai2thor_rgb_image')
        self.declare_parameter('redis_host', 'localhost')
        self.declare_parameter('redis_port', 6379)
        self.declare_parameter('depth_topic', '/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/depth/camera_info')
        self.declare_parameter('rgb_topic', '/rgb/image')
        self.declare_parameter('frame_id', 'camera_depth_optical_frame')

        self.redis_channel_depth = self.get_parameter('redis_channel_depth').value
        self.redis_channel_rgb = self.get_parameter('redis_channel_rgb').value
        self.redis_host = self.get_parameter('redis_host').value
        self.redis_port = self.get_parameter('redis_port').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.rgb_pub = self.create_publisher(Image, self.rgb_topic, 10)


        try:
            self._redis = redis.Redis(host=self.redis_host, port=self.redis_port, db=0)
            self._pubsub = self._redis.pubsub()
            self._pubsub.subscribe(self.redis_channel_depth, self.redis_channel_rgb)
            self.get_logger().info(f"Subscribed to Redis channels '{self.redis_channel_depth}', '{self.redis_channel_rgb}'")
        except Exception as e:
            self.get_logger().error(f"Redis connection failed: {e}")
            self._pubsub = None

        self._stop_event = threading.Event()
        if self._pubsub is not None:
            self._thread = threading.Thread(target=self._redis_loop, daemon=True)
            self._thread.start()

    def _redis_loop(self):
        for message in self._pubsub.listen():
            if self._stop_event.is_set():
                break
            if message.get('type') != 'message':
                continue

            channel = message.get('channel')

            raw = message.get('data')
            if not raw:
                continue

            try:
                if isinstance(raw, bytes):
                    raw = raw.decode('utf-8')
                payload = json.loads(raw)
                self._handle_message(payload, channel)
            except Exception as e:
                self.get_logger().warn(f"Failed to decode Redis message: {e}")
        


    def _handle_message(self, payload: dict, channel):
        try:
            height = int(payload['height'])
            width = int(payload['width'])
            encoding = payload['encoding']  # expected "32FC1"
            dtype = payload['dtype']        # expected "float32"
            data_b64 = payload['data']

            raw_bytes = base64.b64decode(data_b64)
        except Exception as e:
            self.get_logger().warn(f"Depth payload parsing error: {e}")
            return
        
        event_ts = float(payload["timestamp"])
        sec = int(event_ts)
        nanosec = int((event_ts - sec) * 1e9)

        stamp = Time(sec=sec, nanosec=nanosec)
        
        self._handle_camera_info(width, height, stamp, self.frame_id)

        msg = Image()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.height = height
        msg.width = width
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.data = raw_bytes

    
        if channel == self.redis_channel_depth.encode():
            msg.step = width * 4  # 4 bytes per float32
            self.depth_pub.publish(msg)
        elif channel == self.redis_channel_rgb.encode():
            msg.step = width * 3  # 4 bytes per float32
            self.rgb_pub.publish(msg)
        else:
            self.get_logger().warn(f"Received message on unexpected channel: {channel}")
            return


    def _handle_camera_info(self, width: int, height: int, stamp, frame_id: str):

        fov_deg = 90.0
        fov_rad = math.radians(fov_deg)

        fy = (height / 2.0) / math.tan(fov_rad / 2.0)
        fx = fy  # square pixels

        cx = width / 2.0
        cy = height / 2.0



        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion

        msg.k = [fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0]

        msg.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]

        msg.p = [fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0]

        self.camera_info_pub.publish(msg)


    def destroy_node(self):
        self._stop_event.set()
        if hasattr(self, 'pubsub') and self._pubsub:
            try:
                self._pubsub.close()
            except Exception:
                pass
        if hasattr(self, '_thread') and self._thread:
            self._thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthImageAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
