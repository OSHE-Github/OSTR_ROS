#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import time

class boyancyNode(Node):
    def __init__(self):
        super().__init__('fish_boyancy')
        self.publisher_depth = self.create_publisher(
            Int32,
            '/fish_depth',
            10)
        self.publisher_death = self.create_publisher(String,'death',10)
        self.publisher_speed = self.create_publisher(Int32,'/fish_speed',10)
        self.publisher_turn = self.create_publisher(Int32,'/fish_turn',10)
        self.publisher_imu = self.create_publisher(String,'fish_imu',10)
        self.get_logger().info("Fish depth module initalized")
        self.run()


    def run(self):
        msg = Int32()
        msg.data = 0
        time.sleep(360)
        msgstr = String()
        msgstr.data = "Az"
        self.publisher_imu.publish(msgstr)
        self.publisher_depth.publish(msg)
        print("dive1")
        msg.data = 6
        self.publisher_depth.publish(msg)
        time.sleep(60)
        msg.data = 0
        self.publisher_depth.publish(msg)
        msg.data = 4
        self.publisher_turn.publish(msg)
        time.sleep(3)
        msg.data = 12
        self.publisher_turn.publish(msg)
        time.sleep(3)
        msg.data = 4
        self.publisher_turn.publish(msg)
        time.sleep(3)
        msg.data = 12
        self.publisher_turn.publish(msg)
        time.sleep(3)
        msg.data = 8
        self.publisher_turn.publish(msg)
        time.sleep(48)
        msg.data = 6
        self.publisher_depth.publish(msg)
        print("dive2")
        time.sleep(60)
        msg.data = 8
        self.publisher_depth.publish(msg)
       # self.publisher_speed.publish(msg)
        time.sleep(5)
        msg.data = 0
        self.publisher_speed.publish(msg)
        time.sleep(55)
        self.publisher_imu.publish(msgstr)
        time.sleep(20)
        self.publisher_imu.publish(msgstr)
        time.sleep(20)
        self.publisher_depth.publish(msg)
        self.publisher_imu.publish(msgstr)
        time.sleep(20)
        msg.data = 4
        self.publisher_turn.publish(msg)
       # self.publisher_speed.publish(msg)
        time.sleep(4)
        msg.data = 0
       # self.publisher_speed.publish(msg)
        time.sleep(6)
       # self.publisher_speed.publish(msg)
        msg.data = 6
        self.publisher_depth.publish(msg)
        time.sleep(50)
        self.publisher_imu.publish(msgstr)
        time.sleep(240)
        self.publisher_death.publish(msgstr)

    



def main():
    rclpy.init()  
    fish_boyancy = boyancyNode()
    rclpy.spin(fish_boyancy)
    fish_boyancy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


