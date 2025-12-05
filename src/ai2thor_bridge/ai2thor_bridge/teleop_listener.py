import rclpy # type: ignore 
from rclpy.node import Node # type: ignore

import json
import redis

from ai2thor_msgs.msg import TeleopCmd   # same message your teleop publishes


class TeleopListener(Node):

    def __init__(self):
        super().__init__('teleop_listener')

        topic_name = '/ai2thor/teleop_cmd'

        self.subscription = self.create_subscription(
            TeleopCmd,
            topic_name,
            self.listener_callback,
            10
        )
        self.subscription 

        self.get_logger().info(f'Subscribed to topic: {topic_name}')


        self.redis_channel = 'ai2thor_commands'

        try:
            self._redis = redis.Redis(host='localhost', port=6379, db=0)

            self._redis.ping()
            self.get_logger().info(
                f'Subscribed to {topic_name} and connected to Redis channel "{self.redis_channel}"'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Redis: {e}')
            self._redis = None


    
    def listener_callback(self, msg: TeleopCmd):
        
        self.get_logger().info(
            f'Received TeleopCmd: action="{msg.action}", magnitude={msg.magnitude:.3f}'
        )

        if self._redis is None:
            self.get_logger().warn('Redis connection not available, skipping publish.')
            return

        payload = {
            'action': msg.action,
            'magnitude': float(msg.magnitude),
        }

        try:
            serialized = json.dumps(payload)
            self._redis.publish(self.redis_channel, serialized)
        except Exception as e:
            self.get_logger().error(f'Failed to publish to Redis: {e}')



def main(args=None):
    rclpy.init(args=args)
    node = TeleopListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
