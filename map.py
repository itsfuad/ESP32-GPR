# Python GPR Visualization Program
# This runs on the laptop to visualize the GPR data

import serial
import numpy as np
import matplotlib.pyplot as plt
import argparse

class GPRVisualizer:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud)
        self.data_buffer = []
        self.max_buffer_size = 100
        self.current_scan = None
        self.receiving_data = False
        self.scan_count = 0
        self.samples_per_scan = 0
        self.current_line = 0
        
        # Set up the plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 6))
        self.fig.suptitle('ESP32 GPR Visualization', fontsize=16)
        
        # A-scan plot (single signal)
        self.ax1.set_title('A-Scan (Current Signal)')
        self.ax1.set_xlabel('Sample Number')
        self.ax1.set_ylabel('Signal Amplitude')
        self.line, = self.ax1.plot([], [], 'b-')
        
        # B-scan plot (2D image of multiple scans)
        self.ax2.set_title('B-Scan (Depth Profile)')
        self.ax2.set_xlabel('Scan Number')
        self.ax2.set_ylabel('Depth (samples)')
        self.img = self.ax2.imshow(
            np.zeros((256, self.max_buffer_size)), 
            aspect='auto', 
            cmap='viridis',
            vmin=0,
            vmax=4095  # 12-bit ADC max value
        )
        self.fig.colorbar(self.img, ax=self.ax2, label='Signal Strength')
        
    def read_data(self):
        """Read and process data from serial port"""
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            self._handle_line(line)

    def _handle_line(self, line: str):
        if line == "BEGIN_GPR_DATA":
            self._start_receiving()
        elif line == "END_GPR_DATA":
            self._end_receiving()
        elif self.receiving_data:
            if self.current_line == 0:
                self._process_header(line)
            elif self.current_line <= self.scan_count:
                self._process_scan_line(line)

    def _start_receiving(self):
        self.receiving_data = True
        self.current_line = 0

    def _end_receiving(self):
        self.receiving_data = False
        self.process_received_data()

    def _process_header(self, line: str):
        parts = line.split(',')
        if len(parts) == 2:
            try:
                self.scan_count = int(parts[0])
                self.samples_per_scan = int(parts[1])
            except ValueError:
                print(f"Invalid header line: {line}")
        self.current_line += 1

    def _process_scan_line(self, line: str):
        try:
            values = [int(x) for x in line.split(',')]
            if len(values) != self.samples_per_scan:
                return
            if self.current_line == 1:
                self.current_scan = np.array(values)
            self.data_buffer.append(values)
            if len(self.data_buffer) > self.max_buffer_size:
                self.data_buffer.pop(0)
            self.current_line += 1
        except ValueError:
            print(f"Error parsing line: {line}")
    
    def process_received_data(self):
        """Process the received GPR data"""
        print(f"Processed {self.scan_count} scans with {self.samples_per_scan} samples each")
    
    def apply_signal_processing(self, data):
        """Apply basic signal processing to improve visualization"""
        # Convert to numpy array if it's not already
        data_array = np.array(data)
        
        # Apply simple noise reduction (moving average filter)
        kernel_size = 3
        kernel = np.ones(kernel_size) / kernel_size
        filtered_data = np.apply_along_axis(
            lambda m: np.convolve(m, kernel, mode='same'), 
            axis=1, 
            arr=data_array
        )
        
        # Apply gain control (time-varying gain to compensate for signal loss)
        time_gain = np.linspace(1, 3, data_array.shape[1])
        gain_corrected = filtered_data * time_gain
        
        # Clip values to avoid visualization issues
        gain_corrected = np.clip(gain_corrected, 0, 4095)
        
        return gain_corrected
    
    def run(self):
        """Run the visualization"""
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='ESP32 GPR Visualization')
    parser.add_argument('--port', type=str, required=True, help='Serial port (e.g., COM3 or /dev/ttyUSB0)')
    args = parser.parse_args()
    
    visualizer = GPRVisualizer(args.port)
    visualizer.run()