from prun1911service.srv import Prun1911service                                                           # CHANGE

import rclpy
from rclpy.node import Node

import time

class StrReverseService(Node):

    def __init__(self):
        super().__init__('str_reverse_service')
        self.srv = self.create_service(Prun1911service, 'str_reverse', self.str_reverse_callback)       # CHANGE

    def str_reverse_callback(self, request, response):
        start_time = time.perf_counter()
        self.get_logger().info('Incoming request: %s' % request.input_str.data)
        response.output_str = request.input_str.data[::-1]
        self.get_logger().info('Sending response: %s' % response.output_str)
        response.time = time.perf_counter() - start_time
        return response

def main(args=None):
    rclpy.init(args=args)

    str_reverse_service = StrReverseService()

    rclpy.spin(str_reverse_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()