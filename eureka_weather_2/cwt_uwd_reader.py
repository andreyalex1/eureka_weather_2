#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import rclpy
from rclpy.node import Node
import math

class cwt_uwd_reader(Node):
    def __init__(self):
        self.wind_speed = 0
        self.wind_direction = 0
        self.temperature = 0
        self.humidity = 0
        self.noise = 0
        self.pressure = 0
        self.pm25 = 0
        self.pm10 = 0

        super().__init__('cwt_uwd_reader')
        self.pub = self.create_publisher(JointState,'weather', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher)
        self.get_logger().info("cwt_uwd_reader Started!")
    def __del__(self):
        self.get_logger().info("cwt_uwd_reader Killed!")
    def publisher(self):
        message = JointState()
        message.name = ['wind_speed', 'wind_direction', 'temperature', 'humidity', 'noise', 'pressure', 'pm25', 'pm10']
        message.position = [float(self.wind_speed), float(self.wind_direction), float(self.temperature), 
                                float(self.humidity), float(self.noise), float(self.pressure), float(self.pm25), float(self.pm10)]
        self.pub.publish(message)
    

def main(args=None):
    rclpy.init()
    wr = cwt_uwd_reader()
    rclpy.spin(wr)

    
    wr.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()