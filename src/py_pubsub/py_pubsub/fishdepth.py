#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import sys
sys.path.append('/home/OSTR/modem_ws2/src/py_pubsub/py_pubsub')
import OSTR_hat_code
from smbus2 import SMBus
import time


# Authors: Ike Alafita & Jacob Herrema
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)

# Battery voltage monitoring
RBATL = 10000
RBATU = 56000

# ADC constants
ADDRADC = 0x48
ADCREF = 3.3
ADCBATTC = (RBATL + RBATU)/RBATL * (ADCREF/255)


# Motor current monitoring
RIPROPI = 1200          # motor driver current sense resistor
AIPROPI = 0.000450      # motor driver current monitor current gain
# ADC count to motor current (A)
ADCMIC = (ADCREF / 255) / (RIPROPI * AIPROPI)

#constants
RESPIN = 6
PIN = 0
I2C = SMBus(1)
ADDRPWM = 0x1C
M1P1 = 2
M1P2 = 3

# Motor driver modes
MDMRC = 0       # Reverse, Coast -- doesn't seem to make motor move
MDMFC = 1       # Forward, Coast -- doesn't seem to make motor move
MDMFB = 2       # Forward, Brake
MDMRB = 3       # Reverse, Brake

SPEED = 256
TOL = .25

class depthNode(Node):
    def __init__(self):
        super().__init__('fish_depth')
        self.PWM = OSTR_hat_code.PWMExpander(ADDRPWM, I2C)
        self.motor = OSTR_hat_code.motorDriver(self.PWM, M1P1, M1P2)
        self.adc = OSTR_hat_code.ADC(ADDRADC, I2C)
       #self.run(0)
        self.subscription = self.create_subscription(
            Int32,
            '/fish_depth',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.subscription_death = self.create_subscription(String, 'death',self.death,10)
        self.subscription_death #prevent unused variable warning
        self.local()
        self.get_logger().info("Fish depth module initalized")
    def local(self):
        while rclpy.ok():
            n = int(input("number: "))
            n=n*2/3
            self.run(n)
                
    def death(self,msg):
        self.run(0)
        hub_node.destroy_node()
        rclpy.shutdown()


    def command_callback(self, msg):

        depth = msg.data
        depth = depth*2/3
        if depth > 10:
            depth = 10
        print(depth)
        self.run(depth)

    def run(self, depth):
        current = self.adc.read(RESPIN)/16
        print(current)
        if 0 < (current - depth):
            mode = MDMFB
            while TOL < (current - depth):
                self.motor.go(mode, SPEED)
                current = self.adc.read(RESPIN)/16
                print(current)
        else:
            mode = MDMRB
            while TOL < (depth-current):
                self.motor.go(mode, SPEED)
                current = self.adc.read(RESPIN)/16
                print(current)
        self.motor.go(mode, 0)
        print("success! "+str(current)+" = " +str(depth))

def main():
    rclpy.init()
    fish_depth = depthNode()
    rclpy.spin(fish_depth)
    hub_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

