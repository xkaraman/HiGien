from std_srvs.srv import Empty
import serial


import rclpy
from rclpy.node import Node

class PumpService(Node):

    def __init__(self):
        super().__init__('disinfection_services')
        self.get_logger().info('Initilizaing:' )

        self.ser = serial.Serial('/dev/ttyUSB1')
        self.ser.baudrate = 115200

        self.on_pump = self.create_service(Empty, 'higien/on_pump', self.on_pump)
        self.off_pump = self.create_service(Empty, 'higien/off_pump', self.off_pump)
        self.drop_tablet = self.create_service(Empty, 'higien/drop_tablet', self.drop_tablet)

    def on_pump(self, request, response):
        self.get_logger().info('Incoming request\na:' )

        opened = self.ser.isOpen()

        if(opened):
            self.get_logger().info('Serial opened\n:' )

            self.ser.write("on_pump".encode())

        return response

    def off_pump(self, request, response):
        self.get_logger().info('Incoming request\na:' )

        opened = self.ser.isOpen()

        if(opened):
            self.get_logger().info('Serial opened\n:' )

            self.ser.write("off_pump".encode())

        return response

    def drop_tablet(self, request, response):
        self.get_logger().info('Incoming request\na:' )

        opened = self.ser.isOpen()

        if(opened):
            self.get_logger().info('Serial opened\n:' )

            self.ser.write("drop_tablet".encode())

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = PumpService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
