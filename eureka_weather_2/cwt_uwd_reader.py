#!/usr/bin/env python3.10

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import rclpy
from rclpy.node import Node
import math
import serial

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
        try:
            self.ser = serial.Serial('/dev/weather', 4800, timeout=1)
            print(self.ser.name) 
        except serial.serialutil.SerialException:
            serial.close()
            serial.open()
        super().__init__('cwt_uwd_reader')
        self.pub = self.create_publisher(JointState,'weather', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher)
        self.get_logger().info("cwt_uwd_reader Started!")
    def __del__(self):
        self.get_logger().info("cwt_uwd_reader Killed!")
    def publisher(self):
        try:
            message = JointState()
            #wind speed
            self.ser.write(b'\x01\x03\x01\xf4\x00\x01\xc4\x04')
            bt = self.ser.read(7)
            ba = bytearray(bt)
            self.wind_speed = float(int.from_bytes(ba[3:5], "big") / 10)
            #wind direction
            self.ser.write(b'\x01\x03\x01\xf7\x00\x01\x34\x04')
            bt = self.ser.read(7)
            ba = bytearray(bt)
            self.wind_direction = float(int.from_bytes(ba[3:5], "big"))
            # humidity and tempearture
            self.ser.write(b'\x01\x03\x01\xf8\x00\x02\x44\x06')
            bt = self.ser.read(9)
            ba = bytearray(bt)
            self.humidity = float(int.from_bytes(ba[3:5], "big") / 10)
            self.temperature = float(int.from_bytes(ba[5:7], "big") / 10)
            #noise
            self.ser.write(b'\x01\x03\x01\xfa\x00\x01\xa5\xc7')
            bt = self.ser.read(7)
            ba = bytearray(bt)
            self.noise = float(int.from_bytes(ba[3:5], "big") / 10)
            #pm2.5
            self.ser.write(b'\x01\x03\x01\xfb\x00\x01\xf4\x07')
            bt = self.ser.read(7)
            ba = bytearray(bt)
            self.pm25 = float(int.from_bytes(ba[3:5], "big") * 10)
            #pm10
            self.ser.write(b'\x01\x03\x01\xfc\x00\x01\x45\xc6')
            bt = self.ser.read(7)
            ba = bytearray(bt)
            self.pm10 = float(int.from_bytes(ba[3:5], "big") * 10)
            #atmospheric pressure
            self.ser.write(b'\x01\x03\x01\xfd\x00\x01\x14\x06')
            bt = self.ser.read(7)
            ba = bytearray(bt)
            self.pressure = float(int.from_bytes(ba[3:5], "big") / 1000)
            message.name = ['wind_speed', 'wind_direction', 'temperature', 'humidity', 'noise', 'pressure', 'pm25', 'pm10']
            message.position = [float(self.wind_speed), float(self.wind_direction), float(self.temperature), 
                                    float(self.humidity), float(self.noise), float(self.pressure), float(self.pm25), float(self.pm10)]
        #  print(message)
            self.pub.publish(message)
        except serial.serialutil.SerialException:
            self.get_logger().warning("No USB Connection to Weather!")
            try:
                self.ser = serial.Serial('/dev/weather', 4800, timeout=1)
            except serial.serialutil.SerialException:
                None
    

def main(args=None):
    rclpy.init()
    wr = cwt_uwd_reader()
    rclpy.spin(wr)

    
    wr.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()