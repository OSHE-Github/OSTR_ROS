import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Authors: Ike Alafita & Jacob Herrema
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)
# This program is used to run the fish in a wired configuration over an ssh connection

COMMAND_LIST = {  # List of commands mapped to 2-bit values
    "S": "00",
    "D": "01",
    "R": "10",
    "L": "10",
    "C": "11" 
}
NUMBER_LIST = { # Basic hex number for fast string conversion
    "0": "0000", "1": "0001", "2": "0010", "3": "0011",
    "4": "0100", "5": "0101", "6": "0110", "7": "0111",
    "8": "1000", "9": "1001", "A": "1010", "B": "1011",
    "C": "1100", "D": "1101", "E": "1110", "F": "1111"
}

class fakefishrecv(Node):
    
    def __init__(self):
        super().__init__('fake_receiver')
        # Sends inputs to the fishhub as if it were the reviver
        self.publisher_command = self.create_publisher(String, '/fish_reciver', 10)
        self.get_logger().info("Receiver listening for binary commands.")
        self.run()

    def run(self):
        while rclpy.ok(): # main loop waits for inputs before sending them to fishhub
                command_string = ""
                user_input = input("Enter Command: ").strip().upper() # waits for commands letter
                
                if user_input == "K":  # Kill command
                    command_string = "111111"
                    self.publish_command(command_string)
                    rclpy.shutdown()
                else:
                    command_string = COMMAND_LIST.get(user_input, "X") # returns X if command doesnt exist
                    if command_string != "X": # checks for bad return before asking for command number
                        num_input = input("Enter Hex Number: ").strip().upper()
                        
                        if command_string == "R": # handles left right converstion makes them into one unified command from fish persepctive
                            num_input = hex(int(num_input, 16) + 7)[2:].upper()
                        elif command_string == "L":
                            num_input = hex(int(abs(num_input, 16) - 14))[2:].upper()
                        
                        number_string = NUMBER_LIST.get(num_input, "X") # returns X if number is bad
                        if number_string != "X": # If number isnt bad it will save the command into msg and send it to the hub
                            command_string += number_string
                            msg = String()
                            msg.data = command_string
                            self.publisher_command.publish(msg)
                        else:
                            self.get_logger().warn("Invalid Hex Number!")
                    else:
                        self.get_logger().warn("Invalid Command!")

def main():
    rclpy.init()
    fakefish = fakefishrecv()
    rclpy.spin(fakefish)
    fakefish.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
