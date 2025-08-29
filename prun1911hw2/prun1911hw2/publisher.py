import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from prun1911service.msg import Prun1911message                            # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Prun1911message, 'topic', 10)  # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Prun1911message()                                                # CHANGE
        msg.data = "Hello"

        timestamp = Header()
        timestamp.stamp = self.get_clock().now().to_msg()
        timestamp.frame_id = str(self.i)

        msg.header = timestamp

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s" with timestamp %s' % (msg.data, msg.header))       # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()