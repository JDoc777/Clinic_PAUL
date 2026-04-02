#!/usr/bin/env python3
"""
YDLIDAR X2L Visualization Script
Real-time 360° visualization of LIDAR scans
"""

import serial
import time
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import sys


class YDLIDARX2L:
    """Interface for YDLIDAR X2L LiDAR sensor"""
    
    BAUDRATE = 115200
    TIMEOUT = 5
    CMD_SCAN_START = 0xA0
    CMD_SCAN_STOP = 0xA1
    
    def __init__(self, port: str = "/dev/ttyUSB0"):
        self.port = port
        self.serial_conn = None
        self.is_scanning = False
        
    def connect(self) -> bool:
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.BAUDRATE,
                timeout=self.TIMEOUT
            )
            time.sleep(1)
            print(f"✓ Connected to LIDAR on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.stop_scan()
            self.serial_conn.close()
            print("✓ Disconnected from LIDAR")
    
    def start_scan(self) -> bool:
        try:
            cmd = bytes([self.CMD_SCAN_START])
            self.serial_conn.write(cmd)
            time.sleep(0.5)
            self.is_scanning = True
            print("✓ Scan started")
            print("⏳ Waiting for data...")
            return True
        except Exception as e:
            print(f"✗ Failed to start scan: {e}")
            return False
    
    def stop_scan(self) -> bool:
        try:
            cmd = bytes([self.CMD_SCAN_STOP])
            self.serial_conn.write(cmd)
            time.sleep(0.1)
            self.is_scanning = False
            return True
        except Exception as e:
            return False
    
    def read_scan(self):
        if not self.is_scanning:
            return None
        
        try:
            if self.serial_conn.in_waiting == 0:
                print("No data available in buffer")
                return None
            
            print(f"Bytes available: {self.serial_conn.in_waiting}")
            
            scan_data = []
            
            # Try to read packets for up to 0.1 seconds
            start_time = time.time()
            while time.time() - start_time < 0.1:
                if self.serial_conn.in_waiting >= 2:
                    # Look for packet start
                    byte1 = self.serial_conn.read(1)
                    if len(byte1) == 0:
                        break
                    
                    if byte1[0] == 0x54:  # YDLIDAR X2L packet header
                        byte2 = self.serial_conn.read(1)
                        if len(byte2) == 0:
                            break
                        
                        if byte2[0] == 0x2C:  # Valid header
                            # Read rest of packet (40 bytes total)
                            packet = byte1 + byte2 + self.serial_conn.read(38)
                            
                            if len(packet) == 40:
                                # Parse YDLIDAR X2L format
                                start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0
                                end_angle = struct.unpack('<H', packet[6:8])[0] / 100.0
                                
                                # Read 12 measurement points
                                for i in range(12):
                                    offset = 8 + (i * 3)
                                    if offset + 3 <= len(packet):
                                        distance_raw = struct.unpack('<H', packet[offset:offset+2])[0]
                                        distance = distance_raw  # Already in mm
                                        
                                        # Calculate angle for this point
                                        angle = start_angle + (end_angle - start_angle) * i / 11.0
                                        if angle >= 360:
                                            angle -= 360  # Ensure angle stays within 0-359 degrees
                                            
                                        scan_data.append((angle, distance))
            
            return scan_data
        
        except Exception as e:
            print(f"✗ Error reading scan data: {e}")
            return None


def main():
    """Main function to initialize and run the LIDAR visualization."""
    print("🔍 Testing data reception...")
    lidar = YDLIDARX2L()  # Create an instance of the YDLIDARX2L class
    if not lidar.connect():
        print("✗ Unable to connect to LIDAR. Exiting.")
        sys.exit(1)
        
    test_attempts = 0
    got_data = False
    
    while test_attempts < 50 and not got_data:  # Increase attempts
        test_data = lidar.read_scan()
        if test_data:
            print(f"✓ Received {len(test_data)} data points!")
            got_data = True
        else:
            test_attempts += 1
            time.sleep(0.1)  # Total timeout = 5 seconds
    
    if not got_data:
        print("✗ No data received from LIDAR after 5 seconds")
        print("  Try power cycling the device")
        lidar.disconnect()
        sys.exit(1)
    
    # Setup matplotlib
    print("🎨 Opening visualization window...")
    plt.ion()  # Interactive mode
    fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
    fig.suptitle('YDLIDAR X2L 360° Visualization - Press Ctrl+C in terminal to exit', 
                 fontsize=14, fontweight='bold')
    
    scatter = ax.scatter([], [], c='cyan', s=10, alpha=0.8)
    max_range = 8000  # 8 meters in mm
    ax.set_ylim(0, max_range)
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.grid(True, alpha=0.3)
    
    # Status text
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                       verticalalignment='top', fontsize=10,
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    print("✓ Window opened - updating in real-time...")
    print("  Press Ctrl+C here to exit\n")
    
    try:
        frame_count = 0
        all_angles = []
        all_distances = []
        
        while True:
            scan_data = lidar.read_scan()
            
            if scan_data:
                frame_count += 1
                
                # Accumulate points
                for angle, distance in scan_data:
                    all_angles.append(np.radians(angle))
                    all_distances.append(distance)
                
                # Update every 10 frames or when we have enough points
                if frame_count % 5 == 0 or len(all_distances) > 50:
                    if all_angles and all_distances:
                        scatter.set_offsets(np.c_[all_angles, all_distances])
                        
                        min_dist = min(all_distances)
                        max_dist = max(all_distances)
                        avg_dist = np.mean(all_distances)
                        
                        info = f"Points: {len(all_distances)}\nMin: {min_dist:.0f}mm\nMax: {max_dist:.0f}mm\nAvg: {avg_dist:.0f}mm\nFrame: {frame_count}"
                        info_text.set_text(info)
                        
                        fig.canvas.draw()
                        fig.canvas.flush_events()
                        
                        # Keep last 360 degrees worth
                        if len(all_distances) > 500:
                            all_angles = all_angles[-500:]
                            all_distances = all_distances[-500:]
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n⏹ Stopping...")
    finally:
        plt.close('all')
        lidar.disconnect()
        print("✓ Done")


if __name__ == "__main__":
    main()

import serial
import time

port = "/dev/ttyUSB0"
baudrate = 115200
timeout = 5

try:
    with serial.Serial(port, baudrate, timeout=timeout) as ser:
        print(f"Connected to {port}")
        ser.write(bytes([0xA0]))  # Start scan command
        time.sleep(1)
        if ser.in_waiting > 0:
            print(f"Received {ser.in_waiting} bytes")
            data = ser.read(ser.in_waiting)
            print(f"Data: {data}")
        else:
            print("No data received")
except Exception as e:
    print(f"Error: {e}")
