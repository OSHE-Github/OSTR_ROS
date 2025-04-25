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

# Authors: Ike Alafita, Jacob Herrema, Ben Keppers
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)

# Battery voltage divider resistor values
RBATL = 10000
RBATU = 56000

# ADC constants
ADDRADC = 0x48                                      # ADC I2C Address
ADCREF = 3.3                                        # ADC reference voltage (tied to 3.3V from pi by default)
ADCBATTC = (RBATL + RBATU)/RBATL * (ADCREF/255)     # conversion factor for battery voltage measurement
I2C = SMBus(1)

class tempNode(Node):
    def __init__(self):
        super().__init__('fish_temp')
        self.tempSensor =W1ThermSensor()
        self.subscription = self.create_subscription(Int32, '/fish_temp', self.command_callback, 10) # requests tempature 
        self.subscription_death = self.create_subscription(String,'death', self.death, 10) # sends kill command
        self.subscription_battery = self.create_subscription(Int32, '/battery_voltage', self.get_battery, 10) # requests
        self.publish = self.create_publisher(String, 'tx_messages', 10) # returns requested values to hub
        self.depth_file = "temp.txt" # local file for storing tempature
        self.battery_file = "battery.txt" # local file for storing battery voltage
        


    def death(self,msg): # kills program
        hub_node.destroy_node()
        rclpy.shutdown()

    def command_callback(self,msg): # returns tempature value
        celcius = (self.tempSensor.get_temperature()) 
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        print(str(celcius))
        with open(self.depth_file, "a") as temp: # stores return locally
            temp.write("The Temperature in Fahrenhrit: ")
            temp.write(str(celcius))
            temp.write(" Degrees at ")
            temp.write(current_time)
            temp.write("\n")
        msg = String()
        msg.data =str(celcius)
        self.publish.publish(msg) # sends return to transmiter
       

    def get_battery(self, msg): # returns voltage value
        self.adc = OSTR_hat_code.ADC(ADDRADC, I2C)
        current = self.adc.read(7) * ADCBATTC
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        print(str(current))
        with open(self.battery_file, "a") as battery: # stores return locally
            battery.write("The Voltage of the Battery: ")
            battery.write(str(current))
            battery.write(" Volts at ")
            battery.write(current_time)
            battery.write("\n")
        msg = String()
        msg.data = str(current)
        self.publish.publish(msg) # sends return to transmiter


def main():
    rclpy.init()
    fish_temp = tempNode()
    rclpy.spin(fish_temp)
    hub_node.destoy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
