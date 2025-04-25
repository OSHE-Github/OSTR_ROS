#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np
from scipy.fft import fft
from scipy.signal import butter, filtfilt

# Authors: Ike Alafita & Jacob Herrema
# Michigan Technological University 
# Open Source Hardware Enterprise
# Open Source Thunniform Robot (fish)

#this is an old file that has been replaced with fldigi

# Parameters
SAMPLE_RATE = 44100
TONE_DURATION = 0.2
FREQ_ONE = 3000     # Frequency for binary "1"
FREQ_ZERO = 2000    # Frequency for binary "0"
PACKET_SIZE = 8

COMMAND_LIST = {
    "00000001": "UP",
    "00000010": "DOWN",
    "00000011": "LEFT",
    "00000100": "RIGHT",
    "11111111": "KILL"
}

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

class AcousticReceiver(Node):
    def __init__(self):
        super().__init__('acoustic_receiver')
        self.publisher_ = self.create_publisher(String, '/acoustic_modem/receive_data', 10)
        self.get_logger().info("Receiver listening for binary commands.")

        self.p = pyaudio.PyAudio()
        self.audio_stream = self.p.open(
            format=pyaudio.paFloat32, 
            channels=1, 
            rate=SAMPLE_RATE, 
            input=True, 
            frames_per_buffer=int(SAMPLE_RATE * TONE_DURATION)
        )

        self.last_received_command = "Waiting for command..."
        self.run()

    def detect_frequency(self, data):
        # Apply low & high pass filters
        filtered_data = lowpass_filter(data, cutoff=4000, fs=SAMPLE_RATE)
        filtered_data = highpass_filter(filtered_data, cutoff=1000, fs=SAMPLE_RATE)

        # Perform FFT to detect frequency
        yf = fft(filtered_data)
        xf = np.fft.fftfreq(len(yf), 1 / SAMPLE_RATE)

        # Find peak frequency
        idx = np.argmax(np.abs(yf))
        peak_freq = abs(xf[idx])

        # Classify frequency
        if abs(peak_freq - FREQ_ONE) < 50:
            return "1"
        elif abs(peak_freq - FREQ_ZERO) < 50:
            return "0"
        else:
            return "-1"

    def listen_for_bit(self):
        data = np.frombuffer(self.audio_stream.read(int(SAMPLE_RATE * TONE_DURATION)), dtype=np.float32)
        return self.detect_frequency(data)

    def receive_command(self):
        binary_string = ""
        
        # Wait for the start bit
        bit = self.listen_for_bit()
        while bit != "1":
            bit = self.listen_for_bit()
            self.get_logger().info("Start bit not detected, waiting...")

        for bit_index in range(PACKET_SIZE):  
            self.get_logger().info(f"Listening for bit {bit_index + 1}")
            bit = self.listen_for_bit()

            if bit == "-1":
                self.get_logger().info("Failed to detect frequency, aborting command.")
                return "Unknown Command"

            binary_string += bit

        self.get_logger().info(f"Received binary string: {binary_string}")
        return COMMAND_LIST.get(binary_string, "Unknown Command")

    def run(self):
        try:
            while self.last_received_command != "KILL":
                command = self.receive_command()
                if command != "Unknown Command":
                    self.last_received_command = command
                    self.get_logger().info(f"Received command: {self.last_received_command}")
                    
                    msg = String()
                    msg.data = self.last_received_command
                    self.publisher_.publish(msg)

                if self.last_received_command == "KILL":
                    self.get_logger().info("Quitting receiver.")
                    self.shutdown()

        except KeyboardInterrupt:
            self.get_logger().info("Receiver node interrupted.")

    def shutdown(self):
        self.audio_stream.stop_stream()
        self.audio_stream.close()
        self.p.terminate()
        self.get_logger().info("Audio stream closed.")
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    receiver = AcousticReceiver()
    rclpy.spin(receiver)

if __name__ == "__main__":
    main()
