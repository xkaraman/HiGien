from std_srvs.srv import Empty
import serial


import rclpy
from rclpy.node import Node

class PumpService(Node):

    def __init__(self):
        super().__init__('disinfection_node', namespace='higien')
        self.log('Initilizaing pump services')

        self.pump_device_parameter_name = "pump/device"
        self.pump_baudrate_parameter_name = "pump/baudrate"

        device = self.declare_parameter(self.pump_device_parameter_name,"/dev/ttyUSB1")
        baudrate = self.declare_parameter(self.pump_baudrate_parameter_name, 115200)
        
        self.init_pump(device, baudrate)

        self.on_pump = self.create_service(Empty, 'on_pump', self.on_pump)
        self.off_pump = self.create_service(Empty, 'off_pump', self.off_pump)
        self.drop_tablet = self.create_service(Empty, 'drop_tablet', self.drop_tablet)
        self.log('Ready to call pump services')


    def log(self, msg):
        self.get_logger().info(msg)

    def init_pump(self,dev_usb, baudrate):
        self.ser = serial.Serial(dev_usb.value)
        self.ser.baudrate = baudrate.value

    def on_pump(self, request, response):
        self.log("Requesting on_pump")

        opened = self.ser.isOpen()
        if(opened):
            self.ser.write("on_pump".encode())
        else:
            self.log("Serial not opened")

        return response

    def off_pump(self, request, response):
        self.log("Requesting off_pump")

        opened = self.ser.isOpen()
        if(opened):
            self.ser.write("off_pump".encode())
        else:
            self.log("Serial not opened")

        return response

    def drop_tablet(self, request, response):
        self.log("Requesting drop_tablet")
        opened = self.ser.isOpen()

        if(opened):
            self.ser.write("drop_tablet".encode())
        else:
            self.log("Serial not opened")

        return response
