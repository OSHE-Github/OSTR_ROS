#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import sys
sys.path.append('/home/OSTR/modem_ws2/src/py_pubsub/py_pubsub')
import OSTR_hat_code
from gpiozero import PWMOutputDevice
from smbus2 import SMBus
import time

# Authors: Ike Alafita & Jacob Herrema
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)

#constants
ADDRADC = 0x48
ADCPIN = 2
PIN = 13
I2C = SMBus(1)
ADDR = 0x1C
M0P1 = 1
M0P2 = 0
MIDDLE = .075
FREQUENCY = 50


# Motor driver modes
MDMRC = 0       # Reverse, Coast -- doesn't seem to make motor move
MDMFC = 1       # Forward, Coast -- doesn't seem to make motor move
MDMFB = 2       # Forward, Brake
MDMRB = 3       # Reverse, Brake



class TurnNode(Node):
    def __init__(self):
        super().__init__('fish_turn')
        self.servo13 = PWMOutputDevice(PIN)
        self.servo13.frequency = FREQUENCY
        self.adc = OSTR_hat_code.ADC(ADDRADC, I2C)

        self.subscription = self.create_subscription(Int32,'/fish_turn',self.command_callback,10)
        self.subscription_death = self.create_subscription(String, 'death',self.death,10)
        self.get_logger().info("Fish turn module initalized")
       # self.local()
        
    def death(self,msg):
        self.run(MIDDLE)
        hub_node.destroy_node()
        rclpy.shutdown()
    
   # def local(self):
    #    while rclpy.ok():
     #       print("voltage is"+str(self.adc.read(ADCPIN)*.02013071885))
      #      data = float(input("set turn"))
       #     if data < .025:
        #        data = .025
         #   if data > .125:
          #      data = .125
           # self.run(data)


    def command_callback(self, msg):

        turn = msg.data
        turn = (turn - 7)/ 700
        self.run(turn+MIDDLE)

    def run(self, turn):
        print(turn)
        self.servo13.value = turn
       # current = self.adc.read(ADCPIN)*.02013071885
       # angle = MIDDLE
      #  direction = 0
       # while abs(current-turn) > .1:
        #    print("current = "+str(current)+" turn = "+str(turn)+" angle = "+str(angle)+" direction = "+str(direction))
         #   
          #  self.servo13.value = angle
           # time.sleep(.5)
    #        current = self.adc.read(ADCPIN)*.02013071885
     #       direction = current-turn
      #      angle = angle + 0.001 * direction
            

def main():
    rclpy.init()  
    fish_turn = TurnNode()
    rclpy.spin(fish_turn)
    hub_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

