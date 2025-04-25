import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import xmlrpc.client
import time

# Author: Ben Keppers & Ike Alafita
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)

COMMAND_LIST = {  # List of commands mapped to 2-bit values
    "S": "00",
    "D": "01",
    "R": "10",
    "L": "10",
    "C": "11" 
}
NUMBER_LIST = {
    "0": "0000", "1": "0001", "2": "0010", "3": "0011",
    "4": "0100", "5": "0101", "6": "0110", "7": "0111",
    "8": "1000", "9": "1001", "A": "1010", "B": "1011",
    "C": "1100", "D": "1101", "E": "1110", "F": "1111"
}

class fldigirx(Node):
    def __init__(self):
        super().__init__('fldigirx')
        
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
        
        # initialize a buffer for data rx'd from fldigi
        self.rx_buffer = ""

        # ros stuff
        self.publisher_command = self.create_publisher(String, '/fish_reciver', 10)
        self.publisher_tx = self.create_publisher(String, 'tx_messages', 10)
        
        # set up so fldigi is polled at the poling interval set below
        rxpoll_period = 0.5         # polling interval to get rx data from fldigi
        self.rxpoll = self.create_timer(rxpoll_period, self.rxpoll_callback)
        self.get_logger().info("Receiver listening for commands.")

        msg = String()
        msg.data= 'RXinit'
        self.publisher_tx.publish(msg)


    def rxpoll_callback(self):
        self.rx_buffer = self.rx_buffer + str(self.fl.rx.get_data())        # check for new rx data from fldigi
        startcond = self.rx_buffer.find("MSG")                              # check if start condition's present
        if(not(startcond == -1)):
            self.get_logger().info("Message fragment: " + self.rx_buffer)   # print rx'd portion
            endcond = self.rx_buffer.find("END")                            # check if end condition's present
            if(not(endcond == -1)):
                # remove start and end conditions
                self.rx_buffer = self.rx_buffer.replace("MSG","")
                self.rx_buffer = self.rx_buffer.replace("END","")
                self.get_logger().info("Command recieved: " + self.rx_buffer)

                # Interperet command
                user_input = self.rx_buffer[0]
                num_input = self.rx_buffer.replace(self.rx_buffer[0], "").strip().upper()

                if user_input == "K":  # Kill command
                    command_string = "111111"
                    self.publish_command(command_string)
                    rclpy.shutdown()
                else:
                    command_string = COMMAND_LIST.get(user_input, "X") # check to make sure string is correct, otherwise returns X
                    if command_string != "X":
                        if command_string == "R": # translates the right and left into one turn command
                            num_input = hex(int(num_input, 16) + 7)[2:].upper()
                        elif command_string == "L":
                            num_input = hex(int(abs(num_input, 16)))[2:].upper()

                        number_string = NUMBER_LIST.get(num_input, "X") # checks number and returns X if out of scope
                        if number_string != "X": # verifies number isnt bad and sends message to fishhub
                            command_string += number_string
                            msg = String()
                            msg.data = command_string
                            self.publisher_command.publish(msg) # sends msg to fishhub
                        else:
                            self.get_logger().warn("Invalid Hex Number!")
                            
                            # send error message
                            time.sleep(1)
                            msg = String()
                            msg.data= 'BADHEX'
                            self.publisher_tx.publish(msg)
                    else:
                        self.get_logger().warn("Invalid Command!")
                        # send error message
                        time.sleep(1)
                        msg = String()
                        msg.data= 'BADCMD'
                        self.publisher_tx.publish(msg)
                        
                
                # send acknowledgement to controller
                time.sleep(1)
                msg = String()
                msg.data= 'ACK'
                self.publisher_tx.publish(msg)

                # clear local buffer to prepare for next command
                self.rx_buffer=""

        # if the beginning of a start condition is not present clear the rx buffer
        elif(not(self.rx_buffer.endswith("MS") or self.rx_buffer.endswith("M"))):
            self.rx_buffer=""

        # otherwise keep receiving, start of string was received
        else:
            self.get_logger().info("Start detect: " + self.rx_buffer)

def main():
    rclpy.init()
    fldigirxnode = fldigirx()
    rclpy.spin(fldigirxnode)
    fldigirxnode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
