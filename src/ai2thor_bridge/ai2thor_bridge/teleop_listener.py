import rclpy
from rclpy.node import Node

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

    
    def listener_callback(self, msg: TeleopCmd):
        
        print(f'[TeleopCmd] action="{msg.action}"  magnitude={msg.magnitude:.3f}')


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
