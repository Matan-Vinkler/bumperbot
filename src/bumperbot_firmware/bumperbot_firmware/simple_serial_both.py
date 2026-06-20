#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial

class SimpleSerialBoth(Node):
    def __init__(self):
        super().__init__("simple_serial_both")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.arduino_ = serial.Serial(self.port_, self.baudrate_, timeout=0.1)

        self.sub_ = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)

        self.pub_ = self.create_publisher(String, "serial_receiver", 10)
        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def msgCallback(self, msg: String):
        self.arduino_.write(msg.data.encode("utf-8"))

    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline()
            try:
                data.decode("utf-8")
            except:
                return

            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)

            self.get_logger().info(msg.data)

def main():
    rclpy.init()

    simple_serial_both = SimpleSerialBoth()
    rclpy.spin(simple_serial_both)
    simple_serial_both.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
