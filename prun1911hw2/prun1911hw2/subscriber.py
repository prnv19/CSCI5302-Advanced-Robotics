import rclpy
from rclpy.node import Node
import sys
import pandas as pd

from prun1911service.msg import Prun1911message                            # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Prun1911message,
            'topic',
            self.listener_callback,
            10)
        if len(sys.argv) > 1:
            self.max_limit = int(sys.argv[1])
        else:
            self.max_limit = float('inf')
        self.subscription
        self.count = 0
        self.latencies = []

    def listener_callback(self, msg):
        receive_time = self.get_clock().now().to_msg()
        sent_time = msg.header.stamp
        latency = (receive_time.sec + receive_time.nanosec * 1e-9) - (sent_time.sec + sent_time.nanosec * 1e-9)

        self.get_logger().info('Heard: "%s" with Latency %f' % (msg.data, latency))

        if self.max_limit != float('inf'):
            self.latencies.append(latency)
            self.count += 1
        
        if self.count >= self.max_limit:
            df = pd.DataFrame(self.latencies, columns=['Latency'])
            df.to_csv('CSCI5302-Advanced-Robotics/Data/latencies_subscriber.csv', index=False)
            self.get_logger().info(f"Saved latencies to CSCI5302-Advanced-Robotics/Data/latencies_subscriber.csv")
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()