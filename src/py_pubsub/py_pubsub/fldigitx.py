import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import xmlrpc.client
import time

# Author: Ben Keppers
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)

class fldigitx(Node):
    def __init__(self):
        super().__init__('fldigitx')
        
        # connect to fldigi via xmlrpc
        self.fl = xmlrpc.client.ServerProxy("http://127.0.0.1:7362/")
        
        # Turn off auto frequency correction
        self.fl.main.set_afc(False)
        self.fl.main.get_afc()
        
        # Set operating frequency
        self.fl.modem.set_carrier(1500)
        self.fl.modem.get_carrier()
        
        # Set modulation type
        modem = self.fl.modem.set_by_name("MFSK4")
        self.fl.modem.get_name()

        self.get_logger().info("Modem: " + modem)

        # Clear the fldigi input buffer
        self.fl.text.clear_tx()
        self.fl.text.clear_rx()

        # ros stuff
        self.subscriber = self.create_subscription(String, 'tx_messages',self.tx_message_callback, 10)
        self.get_logger().info("TX node initialized")

    def tx_message_callback(self, msg):
        # check if txing, wait for rx if currently txing
        while not(str(self.fl.main.get_trx_status()) == 'rx'):
            time.sleep(0.1)
        self.fl.text.add_tx(msg.data + "^r")        # "^r" changes fldigi to recieve mode. Append this to the string to automatically make fldigi stop transmitting
        self.fl.main.tx()
        self.get_logger().info("Txing: " + msg.data)
        while not(str(self.fl.main.get_trx_status()) == 'rx'):
            time.sleep(0.1)

def main():
    rclpy.init()
    fldigitxnode = fldigitx()
    rclpy.spin(fldigitxnode)
    fldigitxnode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
