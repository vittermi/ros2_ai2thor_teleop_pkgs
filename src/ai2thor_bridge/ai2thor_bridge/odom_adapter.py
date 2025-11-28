import json
import math
import threading

import redis
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from ai2thor_bridge.odom_noise_model import OdomNoiseModel


# check math on ros2 docs (in the quaternion section)
def yaw_deg_to_quaternion(yaw_deg: float):
    yaw_rad = math.radians(yaw_deg)
    half_yaw = yaw_rad * 0.5
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    return (0.0, 0.0, qz, qw)


class Ai2ThorOdomAdapter(Node):

    def __init__(self):
        super().__init__('ai2thor_odom_adapter')

        self.noise_model = OdomNoiseModel()

        self.declare_parameter('redis_host', 'localhost')
        self.declare_parameter('redis_port', 6379)
        self.declare_parameter('redis_channel', 'ai2thor_pose_sensors')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.redis_host = self.get_parameter('redis_host').get_parameter_value().string_value
        self.redis_port = self.get_parameter('redis_port').get_parameter_value().integer_value
        self.redis_channel = self.get_parameter('redis_channel').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self._last_state = None

        try:
            self._redis = redis.Redis(host=self.redis_host, port=self.redis_port, db=0)
            self._pubsub = self._redis.pubsub()
            self._pubsub.subscribe(self.redis_channel)
            self.get_logger().info(
                f'Connected to Redis at {self.redis_host}:{self.redis_port}, '
                f'subscribed to channel "{self.redis_channel}".'
            )
        except Exception as e:
            self._redis = None
            self._pubsub = None
            self.get_logger().error(f'Failed to connect to Redis: {e}')

        self._stop_event = threading.Event()
        if self._pubsub is not None:
            self._thread = threading.Thread(target=self._redis_loop, daemon=True)
            self._thread.start()
        else:
            self._thread = None


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
            except Exception as e:
                self.get_logger().warn(f'Failed to decode JSON from Redis: {e}')
                continue

            self._handle_pose_payload_conversion(payload)


    def _handle_pose_payload_conversion(self, payload: dict):

        try:
            t = float(payload.get('timestamp', 0.0))
            pos = payload['position']
            x, y, z = float(pos.get('x', 0.0)), float(pos.get('y', 0.0)), float(pos.get('z', 0.0))
            rot = payload['rotation']
            yaw = float(rot.get('y', 0.0)) # roll and pitch always approximately 0 for LoCoBot in ai2thor
        except Exception as e:
            self.get_logger().warn(f'Invalid pose payload: {e}')
            return

        current_state = {
            "t": t,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw
        }

        if self._last_state is not None:
            vx, vy, vz, wz = self.noise_model.compute_noisy_velocities(self._last_state, current_state)
        else:
            vx = vy = vz = wz = 0.0

        odom = Odometry()
        now = self.get_clock().now().to_msg()

        qx, qy, qz, qw = self._set_odom_data(
            odom, now, x, y, z, yaw, vx, vy, vz, wz
        )
        self.odom_pub.publish(odom)

        self._set_tf_data(
            now, x, y, z, qx, qy, qz, qw
        )

        self._last_state = current_state



    def _set_odom_data(self, odom, now, x, y, z, yaw, vx, vy, vz, wz):
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z

        qx, qy, qz, qw = yaw_deg_to_quaternion(yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.z = wz

        return qx, qy, qz, qw 
    

    def _set_tf_data(self, now, x, y, z, qx, qy, qz, qw):
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame

        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


    def destroy_node(self):
        """Ensure Redis thread is stopped cleanly."""
        self._stop_event.set()
        if hasattr(self, '_pubsub') and self._pubsub is not None:
            try:
                self._pubsub.close()
            except Exception:
                pass
        if hasattr(self, '_thread') and self._thread is not None:
            self._thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ai2ThorOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
