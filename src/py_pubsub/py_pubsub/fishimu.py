#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from mpu6050 import MPU6050
from smbus2 import SMBus
from datetime import datetime

ADDRIMU = 0x68
freq_divider = 0x04
g = 9.8


class fishIMUNode(Node):
    def __init__(self):
        super().__init__('fish_imu')
        self.subscribe = self.create_subscription(String, '/fish_imu',self.command_callback, 10)
        self.subscribe_death = self.create_subscription(String, 'death',self.death,10)

        self.publisher =self.create_publisher(String,'tx_messages',10)
        self.mpu = MPU6050(1,ADDRIMU,freq_divider)
        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        self.packetsize = self.mpu.DMP_get_FIFO_packet_size()

        self.get_logger().info("Fish IMU initalized")

        
        
        self.imu_file = "imu_info.txt"
       # self.local()

    def local(self):
        command = String()
        command.data = input("imu test: ")
        self.command_callback(command)

    def command_callback(self,command):
        print(command.data)
        FIFO_buffer = [0]*64
        flag = 1
        msg = String()
        command = command.data
        while flag == 1:
            if self.mpu.isreadyFIFO(self.packetsize):
                FIFO_buffer = self.mpu.get_FIFO_bytes(self.packetsize)
                q = self.mpu.DMP_get_quaternion_int16(FIFO_buffer)
                grav = self.mpu.DMP_get_gravity(q)
                roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(q)
                accel = self.mpu.get_acceleration()
                flag = 0
        if command == "Ax":
            msg.data = str((accel.x*2*g/2**15))
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            with open(self.imu_file, "a") as imu_info:
                imu_info.write(str((msg.data)))
                imu_info.write(" Accelerating in the x axis at " )
                imu_info.write(current_time)
                imu_info.write("\n")
        if command == "Ay":
            msg.data = str((accel.y*2*g/2**15))
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            with open(self.imu_file, "a") as imu_info:
                imu_info.write(str(msg.data))
                imu_info.write(" Accelerating in the y axis at ")
                imu_info.write(current_time)
                imu_info.write("\n")
        if command == "Az":
            msg.data = str((accel.z*2*g/2**15))
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            with open(self.imu_file, "a") as imu_info:
                imu_info.write(str((msg.data)))
                imu_info.write(" Accelerating in the z axis at ")
                imu_info.write(current_time)
                imu_info.write("\n")
        if command == "0x":
            msg.data = str((roll_pitch_yaw.x))
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            with open(self.imu_file, "a") as imu_info:
                imu_info.write(str((msg.data)))
                imu_info.write(" Rotating about the x axis at ")
                imu_info.write(current_time)
                imu_info.write("\n")
        if command == "0y":
            msg.data = str((roll_pitch_yaw.y))
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            with open(self.imu_file, "a") as imu_info:
                imu_info.write(str((msg.data)))
                imu_info.write(" Rotating about the y axis at ")
                imu_info.write(current_time)
                imu_info.write("\n")
        if command == "0z":
            msg.data = str((roll_pitch_yaw.z))
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            with open(self.imu_file, "a") as imu_info:
                imu_info.write(str((msg.data)/1000))
                imu_info.write(" Rotating about the z axis at ")
                imu_info.write(current_time)
                imu_info.write("\n")

        self.publisher.publish(msg)


    def death(self,msg):
        rclpy.shutdown()

def main():
    rclpy.init()
    fish_imu = fishIMUNode()
    rclpy.spin(fish_imu)
    fish_imu.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

