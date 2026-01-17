#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ModifiedListener(Node):
    def __init__(self):
        super().__init__('modified_listener')
        self.sub = self.create_subscription(
            String,
            'chatter',
            self.callback,
            10
        )

    def callback(self, msg: String):
        s = msg.data
        s_mod = s + "!"   # modify the string itself (rubric requirement)
        self.get_logger().info(f'I heard "{s_mod}"')

def main():
    rclpy.init()
    node = ModifiedListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
