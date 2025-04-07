#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Int32, String
import pyaudio
import numpy as np
from scipy.fft import fft
from scipy.signal import butter, filtfilt
import curses
from rclpy.node import Node
from std_msgs.msg import String


# Authors: Ike Alafita & Jacob Herrema
# Michigan Technological Univerity 
# Open Sorce Hardware Enterprise
# Open Source Thunniform Robot (fish)

COMMAND_LIST = { # List of mostly useless test commands to verify usablity
    "00": "S",
    "01": "D",
    "10": "T",
    "11": "C" 
}
NUMBER_LIST = {
    "0000": 0,
    "0001": 1,
    "0010": 2,
    "0011": 3,
    "0100": 4,
    "0101": 5,
    "0110": 6,
    "0111": 7,
    "1000": 8,
    "1001": 9,
    "1010": 10,
    "1011": 11,
    "1100": 12,
    "1101": 13,
    "1110": 14,
    "1111": 15
}


class FishHubNode(Node):
    
    def __init__(self):
        super().__init__('fish_hub_node')
        self.subscription = self.create_subscription(String,'/fish_reciver',self.command_callback,10)
    
        self.publisher_imu = self.create_publisher(String, '/fish_imu', 10)
        self.publisher_temp = self.create_publisher(Int32, '/fish_temp', 10)
        self.publisher_death = self.create_publisher(String,'death', 10)
        self.publisher_speed = self.create_publisher(Int32, '/fish_speed', 10)
        self.publisher_depth = self.create_publisher(Int32, '/fish_depth', 10)
        self.publisher_turn = self.create_publisher(Int32, '/fish_turn', 10)
        self.publisher_battery = self.create_publisher(Int32, '/battery_voltage', 10)
        # self.local()
        self.get_logger().info("Hub Node initialized. Ready to take user input.")
        
    def command_callback(self, msg):
        binary_string = msg.data.strip().upper()
        print(binary_string)
        
        self.run(binary_string)

    def local(self):
        while rclpy.ok():
            binary_string = input("enter number: ").strip().upper()
            self.run(binary_string)

    def run(self, binary_string):
        
        
        msg = Int32() 
        #split command into separate blocks
        command_type = COMMAND_LIST.get(binary_string[:2],"X")
        command_number = NUMBER_LIST.get(binary_string[2:],"X")
        print(command_type)
        print(command_number)
        msg.data = command_number
        if command_type == "S":
            self.publisher_speed.publish(msg)
            self.get_logger().info("sent a speed")
        elif command_type == "D":
            self.publisher_depth.publish(msg)
            self.get_logger().info("sent a depth")
        elif command_type == "T":
            self.publisher_turn.publish(msg)
            self.get_logger().info("sent a turn")
        elif command_type == "C":
            if command_number == 15:
                msg = String()
                msg.data = "death"
                self.publisher_death.publish(msg)
                hub_node.destroy_node()
                rclpy.shutdown()
            if command_number == 3:
                self.publisher_temp.publish(msg)
            if command_number == 10:
                self.publisher_battery.publish(msg)
            if command_number == 4:
                msg = String()
                msg.data = "Ax"
                self.publisher_imu.publish(msg)
            if command_number == 5:
                msg = String()
                msg.data = "Ay"
                self.publisher_imu.publish(msg)
            if command_number == 6:
                msg = String()
                msg.data = "Az"
                self.publisher_imu.publish(msg)
            if command_number == 7:
                msg = String()
                msg.data = "0x"
                self.publisher_imu.publish(msg)
            if command_number == 8:
                msg = String()
                msg.data = "0y"
                self.publisher_imu.publish(msg)
            if command_number == 9:
                msg = String()
                msg.data = "0z"
                self.publisher_imu.publish(msg)





def main():
    rclpy.init()
    fish_hub_node = FishHubNode()
    rclpy.spin(fish_hub_node)
    hub_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
