from prun1911service.srv import Prun1911service
from std_msgs.msg import String                                                         # CHANGE
import sys
import rclpy
from rclpy.node import Node
import pandas as pd
import time

class StrReverseClient(Node):

    def __init__(self):
        super().__init__('str_reverse_client')
        self.cli = self.create_client(Prun1911service, 'str_reverse')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Prun1911service.Request()

    def send_request(self):
        self.req.input_str = String()
        self.req.input_str.data = sys.argv[1]
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    str_reverse_client = StrReverseClient()

    runs = 1
    latencies = []
    if len(sys.argv) > 2:
        runs = int(sys.argv[2])
    
    for _ in range(runs):
        start_time = time.perf_counter()
        str_reverse_client.send_request()
        while rclpy.ok():
            rclpy.spin_once(str_reverse_client)
            if str_reverse_client.future.done():
                try:
                    response = str_reverse_client.future.result()
                    client_time = time.perf_counter() - start_time
                    latencies.append(client_time - response.time)
                except Exception as e:
                    str_reverse_client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    str_reverse_client.get_logger().info(
                        'Result of str_reverse: for %s = %s with Latency: %f' %
                        (str_reverse_client.req.input_str.data, response.output_str, client_time))
                break
    
    if runs > 1:
        latencies = pd.DataFrame(latencies, columns=['Latency'])
        latencies.to_csv('CSCI5302-Advanced-Robotics/Data/latencies_client.csv', index=False)
        str_reverse_client.get_logger().info('Latencies saved to CSV at CSCI5302-Advanced-Robotics/Data/latencies_client.csv')

    str_reverse_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()