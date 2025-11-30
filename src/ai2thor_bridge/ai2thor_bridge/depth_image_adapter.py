import rclpy
from rclpy.node import Node

import redis
import threading
import json
import base64
import numpy as np
import math

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header


class DepthImageAdapter(Node):
    def __init__(self):
        super().__init__('depth_image_adapter')

        self.declare_parameter('redis_channel', 'ai2thor_depth_image')
        self.declare_parameter('redis_host', 'localhost')
        self.declare_parameter('redis_port', 6379)
        self.declare_parameter('depth_topic', '/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/depth_camera_info')
        self.declare_parameter('frame_id', 'camera_link')

        self.redis_channel = self.get_parameter('redis_channel').value
        self.redis_host = self.get_parameter('redis_host').value
        self.redis_port = self.get_parameter('redis_port').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.publisher = self.create_publisher(Image, self.depth_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        try:
            self._redis = redis.Redis(host=self.redis_host, port=self.redis_port, db=0)
            self._pubsub = self._redis.pubsub()
            self._pubsub.subscribe(self.redis_channel)
            self.get_logger().info(f"Subscribed to Redis channel '{self.redis_channel}'")
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

            raw = message.get('data')
            if not raw:
                continue

            try:
                if isinstance(raw, bytes):
                    raw = raw.decode('utf-8')
                payload = json.loads(raw)
                self._handle_depth_message(payload)
            except Exception as e:
                self.get_logger().warn(f"Failed to decode Redis message: {e}")

    def _handle_depth_message(self, payload: dict):
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
        
        stamp = self.get_clock().now().to_msg()
        
        self._handle_camera_info(width, height, stamp, self.frame_id)

        msg = Image()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.height = height
        msg.width = width
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = width * 4  # 4 bytes per float32
        msg.data = raw_bytes

        self.publisher.publish(msg)


    def _handle_camera_info(self, width: int, height: int, stamp, frame_id: str):

        fov_deg = 90.0
        fov_rad = math.radians(fov_deg)
        fx = fy = width / (2.0 * math.tan(fov_rad / 2.0))
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
