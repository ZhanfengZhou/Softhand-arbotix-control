import rclpy
from rclpy.node import Node
import numpy as np


from std_msgs.msg import Float64


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float64, 'topic', 10)
        self.get_logger().info('Publisher: created')

    def run(self):
        i = 0
        self.get_logger().info('Running Publisher')
        while True:
            msg = Float64()
            msg.data = i
            self.get_logger().info('Publishing: {msg.data}' )
            self.publisher_.publish(msg)
            i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.run()
    #rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
