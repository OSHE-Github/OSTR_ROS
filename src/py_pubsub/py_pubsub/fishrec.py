#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np
from scipy.fft import fft
from scipy.signal import butter, filtfilt

# Parameters
SAMPLE_RATE = 44100
TONE_DURATION = 0.3
FREQ_ONE = 3500  # Frequency for binary "1"
FREQ_ZERO = 3000  # Frequency for binary "0"
FISH_PACKET = 6  # Default packet size for Fish Hub
HUB_PACKET = 4   # Default packet size for Hub
PACKET_SIZE = None  # Will be set based on transmission source

# High-pass filter to remove low-frequency noise
def highpass_filter(data, cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return filtfilt(b, a, data)

# Low-pass filter to remove high-frequency noise
def lowpass_filter(data, cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

# Detects frequency using FFT
def detect_frequency(data):
    # Apply low & high-pass filters to remove noise
    filtered_data = highpass_filter(data, cutoff=2700, fs=SAMPLE_RATE)

    # Perform FFT to detect frequency components
    yf = fft(filtered_data)
    xf = np.fft.fftfreq(len(yf), 1 / SAMPLE_RATE)

    # Find the highest amplitude frequency
    idx = np.argmax(np.abs(yf))
    peak_freq = abs(xf[idx])

    # Classify based on detected frequency
    if abs(peak_freq - FREQ_ONE) < 150:
        return "1"
    elif abs(peak_freq - FREQ_ZERO) < 150:
        return "0"
    else:  # If an invalid frequency was detected
        return "-1"

# Records and returns audio data
def listen_for_bit(audio_stream):
    data = np.frombuffer(audio_stream.read(int(SAMPLE_RATE * TONE_DURATION)), dtype=np.float32)
    return detect_frequency(data)

# Finds start bit then translates signal into a command
def receive_command(audio_stream, packet_size, node):
    binary_string = ""

    # Wait for the start bit (should be "1")
    bit = listen_for_bit(audio_stream)
    while bit != "1":
        bit = listen_for_bit(audio_stream)
        node.get_logger().info("Waiting for start bit...")  # Debugging

    # Receive the packet
    for bit_index in range(packet_size):
        node.get_logger().info(f"Listening for bit {bit_index + 1}")
        bit = listen_for_bit(audio_stream)

        # Check if the received bit is valid
        if bit == "-1":
            node.get_logger().warn("Failed to detect a valid frequency, aborting command.")
            return "Unknown Command"

        binary_string += bit

    node.get_logger().info(f"Received binary string: {binary_string}")
    return binary_string

class AcousticReceiver(Node):
    def __init__(self):
        super().__init__('acoustic_receiver')
        self.publisher_ = self.create_publisher(String, '/fish_reciver', 10)
        self.subscription_death = self.create_subscription(String, 'death',self.death,10)
        self.get_logger().info("Receiver listening for binary commands.")


        # Initialize PyAudio stream
        self.p = pyaudio.PyAudio()
        self.audio_stream = self.p.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=int(SAMPLE_RATE * TONE_DURATION)
        
            )
        self.run_hub()

    def death(self,msg):
        self.cleanup()
        rclpy.shutdown()

    def run_hub(self):
        global PACKET_SIZE
        PACKET_SIZE = FISH_PACKET
        while rclpy.ok():
            command = receive_command(self.audio_stream, PACKET_SIZE, self)
            msg = String()
            msg.data = command
            self.publisher_.publish(msg)

    def cleanup(self):
        self.audio_stream.stop_stream()
        self.audio_stream.close()
        self.p.terminate()
        self.get_logger().info("Audio stream closed.")

def main():
    rclpy.init()
    receiver = AcousticReceiver()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
