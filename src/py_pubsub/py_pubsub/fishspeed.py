#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys
sys.path.append('/home/OSTR/modem_ws/src/py_pubsub/py_pubsub')
import OSTR_hat_code
from smbus2 import SMBus


# Authors: Ike Alafita & Jacob Herrema
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)

#constants``
PIN = 0
I2C = SMBus(1)
ADDR = 0x1C
M0P1 = 1
M0P2 = 0

# Motor driver modes
MDMRC = 0       # Reverse, Coast -- doesn't seem to make motor move
MDMFC = 1       # Forward, Coast -- doesn't seem to make motor move
MDMFB = 2       # Forward, Brake
MDMRB = 3       # Reverse, Brake



class SpeedNode(Node):
    def __init__(self):
        super().__init__('fish_speed')
        self.PWM = OSTR_hat_code.PWMExpander(ADDR, I2C) # instantiates PWM module
        self.motor = OSTR_hat_code.motorDriver(self.PWM, M0P1, M0P2) # instantiates motor
        self.subscription = self.create_subscription(Int32,'/fish_speed',self.command_callback,10) # recives speed
        self.subscription  # prevent unused variable warning
       # self.subscription_death = self.create_subscription(String, 'death',self.death,10)
        self.get_logger().info("Fish speed module initalized")

    def death(self,msg): # recives death command
        self.run(0)
        hub_node.destroy_node()
        rclpy.shutdown()

    def command_callback(self, msg): # recives speed and makes modifies it

        speed = msg.data
        speed = speed * 16 # turns speed into value from 0-255
        print(speed)
        self.run(speed)

    def run(self, speed): # sets speed to provided number
       self.motor.go(MDMRB, speed)


def main():
    rclpy.init()  
    fish_speed = SpeedNode()
    rclpy.spin(fish_speed)
    hub_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

