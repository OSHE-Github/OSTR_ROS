#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np

# Authors: Ike Alafita & Jacob Herrema
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)

#this is an old file that has been replaced with fldigi

# Parameters 
SAMPLE_RATE = 44100  # Number of samples per second
TONE_DURATION = 0.15  # Duration of each bit tone in seconds
FREQ_ONE = 1000      # Frequency for binary "1"
FREQ_ZERO = 500      # Frequency for binary "0"


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
            String,
            '/hub_command',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.p = pyaudio.PyAudio()
        self.audio_stream = self.p.open(
            format=pyaudio.paFloat32, 
            channels=1, 
            rate=SAMPLE_RATE, 
            output=True
        )

        self.get_logger().info("Acoustic Transmitter Node Initialized. Listening for commands.")

    def command_callback(self, msg):
        command = msg.data.strip().upper()
        print(command)
        self.transmit_command(command)
        
    def generate_tone(self, frequency, duration):
        t = np.linspace(0, duration, int(SAMPLE_RATE * duration), False)
        tone = 0.5 * np.sin(2 * np.pi * frequency * t)
        return tone.astype(np.float32)

    def play_tone(self, frequency):
        tone = self.generate_tone(frequency, TONE_DURATION)
        self.audio_stream.write(tone.tobytes())

    def transmit_command(self, binary_string):
        self.play_tone(FREQ_ONE)  # Start tone
        print("tone")
        for bit in binary_string:
            if bit == "1":
                self.play_tone(FREQ_ONE)
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
