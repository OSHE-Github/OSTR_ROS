#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Authors: Ike Alafita & Jacob Herrema
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

class HubNode(Node):
    def __init__(self):
        super().__init__('hub_node')
        self.publisher_ = self.create_publisher(String, '/hub_command', 10)
        self.get_logger().info("Hub Node initialized. Ready to take user input.")
        self.run()

    def run(self):
        while rclpy.ok():
            command_string = ""
            user_input = input("Enter Command: ").strip().upper()
            
            if user_input == "K":  # Kill command
                command_string = "111111"
                self.publish_command(command_string)
                rclpy.shutdown()
            else:
                command_string = COMMAND_LIST.get(user_input, "X")
                if command_string != "X":
                    num_input = input("Enter Hex Number: ").strip().upper()
                    
                    if command_string == "R":
                        num_input = hex(int(num_input, 16) + 7)[2:].upper()
                    elif command_string == "L":
                        num_input = hex(int(abs(num_input, 16) - 14))[2:].upper()
                    
                    number_string = NUMBER_LIST.get(num_input, "X")
                    if number_string != "X":
                        command_string += number_string
                        self.publish_command(command_string)
                    else:
                        self.get_logger().warn("Invalid Hex Number!")
                else:
                    self.get_logger().warn("Invalid Command!")

    def publish_command(self, command_string):
        msg = String()
        msg.data = command_string
        self.publisher_.publish(msg)
        self.get_logger().info(f"Command sent: {command_string}")


def main():
    rclpy.init()
    hub_node = HubNode()
    rclpy.spin(hub_node)
    hub_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
