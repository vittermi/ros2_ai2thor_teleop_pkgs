import rclpy # type: ignore 
from rclpy.node import Node # type: ignore

import json
import redis

from geometry_msgs.msg import TwistStamped # type: ignore
from msg_constants.actions import ActionType
from dataclasses import dataclass

from math import pi

dt = 0.1


@dataclass(frozen=True)
class Action:
    type: str
    magnitude: float


class CmdVelAdapter(Node):

    def __init__(self):
        super().__init__('cmd_vel_adapter')

        self.declare_parameter('topic_name', '/cmd_vel')
        self.declare_parameter("tick_frequency_hz", 10.0)
        self.declare_parameter("move_threshold_m", 0.5)
        self.declare_parameter("yaw_threshold_deg", 1.0)
        self.declare_parameter("linear_deadband", 1e-3)
        self.declare_parameter("angular_deadband", 1e-3)
        self.declare_parameter("enable_strafe", False)
        self.declare_parameter("prefer_rotation_first", True)
        self.declare_parameter('redis_host', 'localhost')
        self.declare_parameter('redis_port', 6379)

        self._topic_name = self.get_parameter('topic_name').value
        self._tick_hz = self.get_parameter("tick_frequency_hz").value
        self._move_threshold_m = self.get_parameter("move_threshold_m").value
        self._yaw_threshold_deg = self.get_parameter("yaw_threshold_deg").value
        self._lin_deadband = self.get_parameter("linear_deadband").value
        self._ang_deadband = self.get_parameter("angular_deadband").value
        self._enable_strafe = self.get_parameter("enable_strafe").value
        self._prefer_rotation_first = self.get_parameter("prefer_rotation_first").value
        self.redis_host = self.get_parameter('redis_host').value
        self.redis_port = self.get_parameter('redis_port').value

        self._dt = 1.0 / self._tick_hz

        self._residual_fwd_m = 0.0
        self._residual_yaw_deg = 0.0

        self.latest_msg = None

        self.subscription = self.create_subscription(
            TwistStamped,
            self._topic_name,
            self.listener_callback,
            10
        )
    
        self._timer = self.create_timer(
            self._dt,
            self.timer_callback
        )

        self.redis_channel = 'ai2thor_commands'

        try:
            self._redis = redis.Redis(host='localhost', port=6379, db=0)

            self._redis.ping()
            self.get_logger().info(
                f'Subscribed to {self._topic_name} and connected to Redis channel "{self.redis_channel}"'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Redis: {e}')
            self._redis = None


    
    def listener_callback(self, msg: TwistStamped):
        self.latest_msg = msg
        

    def timer_callback(self):
        msg = self.latest_msg
        if msg is None:
            return
        
        self.latest_msg = None

        if self._redis is None:
            self.get_logger().warn("Redis connection not available, skipping publish.")
            return

        vx = float(msg.twist.linear.x)
        wz = float(msg.twist.angular.z)

        action_ang, self._residual_yaw_deg = self._angular_vel_to_action(wz, 
                                                                        self._dt, 
                                                                        self._residual_yaw_deg, 
                                                                        self._yaw_threshold_deg, 
                                                                        self._ang_deadband)
        if action_ang is not None: self._publish_action(action_ang.type, action_ang.magnitude)

        action_lin, self._residual_fwd_m = self._linear_vel_to_action(vx, 
                                                                    self._dt, 
                                                                    self._residual_fwd_m, 
                                                                    self._move_threshold_m, 
                                                                    self._lin_deadband)
        
        if action_lin is not None: self._publish_action(action_lin.type, action_lin.magnitude)


    def _publish_action(self, action, magnitude):
        payload = {
            "action": action.value,
            "magnitude": float(magnitude),
            "stamp": self.get_clock().now().nanoseconds,
        }
        try:
            self._redis.publish(self.redis_channel, json.dumps(payload))
        except Exception as e:
            self.get_logger().error(f"Failed to publish to Redis: {e}")



    def _angular_vel_to_action(
        self,
        wz_rad_s: float,
        dt: float,
        residual_yaw_deg: float,
        yaw_threshold_deg: float,
        deadband_rad_s: float,
    ) -> tuple[Action | None, float]:

        if abs(wz_rad_s) < deadband_rad_s:
            wz_rad_s = 0.0

        residual_yaw_deg += (wz_rad_s * dt) * (180.0 / pi)

        if abs(residual_yaw_deg) < yaw_threshold_deg:
            return None, residual_yaw_deg

        degrees = residual_yaw_deg

        action = ActionType.ROTATE_LEFT if degrees > 0 else ActionType.ROTATE_RIGHT 
        magnitude = abs(degrees)

        return Action(action, magnitude), 0.0


    def _linear_vel_to_action(
        self,
        vx_m_s: float,
        dt: float,
        residual_fwd_m: float,
        move_threshold_m: float,
        deadband_m_s: float,
    ) -> tuple[Action | None, float]:

        if abs(vx_m_s) < deadband_m_s:
            vx_m_s = 0.0

        residual_fwd_m += vx_m_s * dt

        if abs(residual_fwd_m) < move_threshold_m:
            return None, residual_fwd_m

        meters = residual_fwd_m

        action = ActionType.MOVE_AHEAD if meters > 0 else ActionType.MOVE_BACK
        magnitude = abs(meters)

        return Action(action, magnitude), 0.0




def main(args=None):
    rclpy.init(args=args)
    node = CmdVelAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
