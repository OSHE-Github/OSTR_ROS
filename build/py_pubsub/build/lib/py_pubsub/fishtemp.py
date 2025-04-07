#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from w1thermsensor import W1ThermSensor, Unit
import sys
sys.path.append('/home/OSTR/modem_ws2/src/py_pubsub/py_pubsub')
import OSTR_hat_code
from smbus2 import SMBus
from datetime import datetime

RBATL = 10000
RBATU = 56000

ADDRADC = 0x48
ADCREF = 3.3
ADCBATTC = (RBATL + RBATU)/RBATL * (ADCREF/255)
I2C = SMBus(1)

class tempNode(Node):
    def __init__(self):
        super().__init__('fish_temp')
        self.tempSensor =W1ThermSensor()
        self.subscription = self.create_subscription(Int32, '/fish_temp', self.command_callback, 10)
        self.subscription_death = self.create_subscription(String,'death', self.death, 10)
        self.subscription_battery = self.create_subscription(Int32, '/battery_voltage', self.get_battery, 10)
        self.publish = self.create_publisher(String, 'tx_messages', 10)
        self.depth_file = "temp.txt"
        self.battery_file = "battery.txt"
        


    def death(self,msg):
        hub_node.destroy_node()
        rclpy.shutdown()

    def command_callback(self,msg):
        celcius = (self.tempSensor.get_temperature())
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        print(str(celcius))
        with open(self.depth_file, "a") as temp:
            temp.write("The Temperature in Fahrenhrit: ")
            temp.write(str(celcius))
            temp.write(" Degrees at ")
            temp.write(current_time)
            temp.write("\n")
        msg = String()
        msg.data =str(celcius)
        self.publish.publish(msg)
       

    def get_battery(self, msg):
        self.adc = OSTR_hat_code.ADC(ADDRADC, I2C)
        current = self.adc.read(7) * ADCBATTC
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        print(str(current))
        with open(self.battery_file, "a") as battery:
            battery.write("The Voltage of the Battery: ")
            battery.write(str(current))
            battery.write(" Volts at ")
            battery.write(current_time)
            battery.write("\n")
        msg = String()
        msg.data = str(current)
        self.publish.publish(msg)


def main():
    rclpy.init()
    fish_temp = tempNode()
    rclpy.spin(fish_temp)
    hub_node.destoy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
