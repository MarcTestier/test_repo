#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class TestNode(Node):

    def __init__(self):
        super().__init__('kone_open_opc_client')

        self.test_sub = self.create_subscription(Empty, 'test_repo', self.test_repo_callback)
        self.test_sub  # prevent unused variable warning


    # Service callback which send the lift to the given floor, etc
    def test_repo_callback(self, msg):
        self.get_logger().info('Blip')
        rclpy.spin_once(self)
        self.get_logger().info('Blop')
        

def main(args=None):
    rclpy.init(args=args)

    node = TestNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - Done automatically when node is garbage collected)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

