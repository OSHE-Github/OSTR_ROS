#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import pyaudio
import numpy as np

# Authors: Ike Alafita & Jacob Herrema
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)

#this is an outdated part of code that has been replaced with fldigi part

# Parameters 
SAMPLE_RATE = 44100  # Number of samples per second
TONE_DURATION = 0.3  # Duration of each bit tone in seconds
FREQ_ZERO = 2000      # Frequency for binary "1"
FREQ_ONE = 2800      # Frequency for binary "0"

COMMAND_LIST = {  # Mapping of commands to binary strings
    "S": "00",
    "D": "01",
    "R": "10",
    "L": "10",
    "C": "11" 
}

class TransmitterNode(Node):
    def __init__(self):
        super().__init__('acoustic_transmitter')
        self.subscription = self.create_subscription(
            Int32,
            '/hub_command',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.subscription_death = self.create_subscription(String, 'death',self.death,10)
        self.subscription_return = self.create_subscription(Int32, 'return', self.command_callback,10)
        self.p = pyaudio.PyAudio()
        self.audio_stream = self.p.open(
            format=pyaudio.paFloat32, 
            channels=1, 
            rate=SAMPLE_RATE, 
            output=True,
            output_device_index = 1
        )

        self.get_logger().info("Acoustic Transmitter Node Initialized. Listening for commands.")

    def death(self,msg):
        self.ack()
        node.destroy_node()
        rclpy.shutdown()
    

    def command_callback(self, msg):
        data = msg.data
        print("data = "+str(data))
        data = bin(data)[2:]
        length = format(len(data),'04b')
        print("length = "+length)
        self.transmit_command(length)
        self.transmit_command(data)
        
    def generate_tone(self, frequency, duration):
        t = np.linspace(0, duration, int(SAMPLE_RATE * duration), False)
        tone = 0.5 * np.sin(2 * np.pi * frequency * t)
        return tone.astype(np.float32)

    def play_tone(self, frequency):
        tone = self.generate_tone(frequency, TONE_DURATION)
        self.audio_stream.write(tone.tobytes())

    def ack(self):
        self.transmit_command("0000")

    def transmit_command(self, binary_string):
        tone = self.generate_tone(FREQ_ONE,TONE_DURATION*.7)
        self.audio_stream.write(tone.tobytes())
        tone = self.generate_tone(FREQ_ZERO,TONE_DURATION*.3)
        self.audio_stream.write(tone.tobytes())
        print("tone")
        for bit in binary_string:
            if bit == "1":
                tone = self.generate_tone(FREQ_ONE,TONE_DURATION*.7)
                self.audio_stream.write(tone.tobytes())
                tone = self.generate_tone(FREQ_ZERO,TONE_DURATION*.3)
                self.audio_stream.write(tone.tobytes())
            else:
                self.play_tone(FREQ_ZERO)

    def destroy_node(self):
        self.audio_stream.stop_stream()
        self.audio_stream.close()
        self.p.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = TransmitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Transmitter node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
