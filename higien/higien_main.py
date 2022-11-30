from std_srvs.srv import Empty
import serial


import rclpy
from rclpy.node import Node

from higien.higien_pump_node import PumpService

def main(args=None):
    rclpy.init(args=args)

    pump_service_node = PumpService()

    rclpy.spin(pump_service_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
