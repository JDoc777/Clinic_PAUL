#!/usr/bin/env python3
"""
YDLIDAR X2L LiDAR Control Script
Controls and reads data from YDLIDAR X2L 360-degree 2D laser range sensor
"""

import serial
import time
import struct
import sys
from typing import List, Tuple, Optional


class YDLIDARX2L:
    """Interface for YDLIDAR X2L LiDAR sensor"""
    
    # YDLIDAR X2L serial configuration
    BAUDRATE = 115200
    TIMEOUT = 5
    
    # Command bytes
    CMD_SCAN_START = 0xA0
    CMD_SCAN_STOP = 0xA1
    CMD_DEVICE_INFO = 0x04
    CMD_HEALTH = 0x05
    
    def __init__(self, port: str = "COM3"):
        """
        Initialize LIDAR connection
        Args:
            port: Serial port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
        """
        self.port = port
        self.serial_conn = None
        self.is_scanning = False
        
    def connect(self) -> bool:
        """Connect to LIDAR device"""
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
        """Disconnect from LIDAR device"""
        if self.serial_conn and self.serial_conn.is_open:
            self.stop_scan()
            self.serial_conn.close()
            print("✓ Disconnected from LIDAR")
    
    def start_scan(self) -> bool:
        """Start scanning"""
        try:
            # Send start scan command
            cmd = bytes([self.CMD_SCAN_START])
            self.serial_conn.write(cmd)
            time.sleep(0.1)
            self.is_scanning = True
            print("✓ Scan started")
            return True
        except Exception as e:
            print(f"✗ Failed to start scan: {e}")
            return False
    
    def stop_scan(self) -> bool:
        """Stop scanning"""
        try:
            cmd = bytes([self.CMD_SCAN_STOP])
            self.serial_conn.write(cmd)
            time.sleep(0.1)
            self.is_scanning = False
            print("✓ Scan stopped")
            return True
        except Exception as e:
            print(f"✗ Failed to stop scan: {e}")
            return False
    
    def read_scan(self) -> Optional[List[Tuple[float, float]]]:
        if not self.is_scanning:
            return None
        
        try:
            scan_data = []
            
            # Read scan packet header (4 bytes)
            header = self.serial_conn.read(4)
            print(f"Raw header: {header}")
            
            if len(header) < 4:
                print("✗ Incomplete header received")
                return None
            
            # Parse header
            if header[0] != 0xFA or header[1] != 0xA0:
                print("✗ Invalid header")
                return None
            
            scan_count = header[2]
            fs_a = header[3]
            
            print(f"Scan count: {scan_count}, FS_A: {fs_a}")
            
            # Read scan points
            for _ in range(scan_count):
                point_data = self.serial_conn.read(5)
                print(f"Raw point data: {point_data}")
                if len(point_data) < 5:
                    print("✗ Incomplete point data")
                    continue
                
                # Parse point: angle (2 bytes), distance (2 bytes), intensity (1 byte)
                angle_raw = struct.unpack('<H', point_data[0:2])[0]
                distance_raw = struct.unpack('<H', point_data[2:4])[0]
                
                # Convert raw values to actual measurements
                angle = angle_raw / 64.0  # degrees
                distance = distance_raw / 4.0  # mm
                
                scan_data.append((angle, distance))
       
            return scan_data

        except Exception as e:
            print(f"✗ Error reading scan: {e}")
            return None
    
    def get_device_info(self) -> Optional[dict]:
        """Get device information"""
        try:
            cmd = bytes([self.CMD_DEVICE_INFO])
            self.serial_conn.write(cmd)
            time.sleep(0.1)
            
            response = self.serial_conn.read(20)
            if len(response) > 0:
                info = {
                    "model": response[0] if len(response) > 0 else None,
                    "firmware": f"{response[2]}.{response[3]}" if len(response) > 3 else None
                }
                print(f"✓ Device Info: Model={info['model']}, Firmware={info['firmware']}")
                return info
        except Exception as e:
            print(f"✗ Failed to get device info: {e}")
        
        return None


def main():
    """Main function - Example usage"""
    
    # Try common serial ports
    ports_to_try = ["COM6", "COM7", "COM3", "COM4", "COM5", "/dev/ttyUSB0", "/dev/ttyUSB1"]
    
    lidar = None
    for port in ports_to_try:
        lidar = YDLIDARX2L(port=port)
        if lidar.connect():
            break
    else:
        print("✗ Could not connect to LIDAR on any port")
        print(f"  Tried: {', '.join(ports_to_try)}")
        print("  Please verify the connection and check Device Manager for the correct COM port")
        sys.exit(1)
    
    try:
        # Get device info
        lidar.get_device_info()
        
        # Start scanning
        if not lidar.start_scan():
            sys.exit(1)
        
        print("\n📊 Reading LIDAR scans (press Ctrl+C to stop)...\n")
        
        scan_count = 0
        try:
            while True:
                scan_data = lidar.read_scan()
                
                if scan_data:
                    scan_count += 1
                    # Find closest obstacle
                    distances = [d for _, d in scan_data if d > 0]
                    if distances:
                        min_distance = min(distances)
                        max_distance = max(distances)
                        avg_distance = sum(distances) / len(distances)
                        
                        print(f"Scan #{scan_count}: {len(distances)} points | "
                              f"Min: {min_distance:.0f}mm | Max: {max_distance:.0f}mm | "
                              f"Avg: {avg_distance:.0f}mm")
                        
                        # Optional: Print points at specific angles
                        points_0deg = [d for a, d in scan_data if abs(a - 0) < 5 and d > 0]
                        points_90deg = [d for a, d in scan_data if abs(a - 90) < 5 and d > 0]
                        
                        if points_0deg:
                            print(f"  Front (0°): {points_0deg[0]:.0f}mm")
                        if points_90deg:
                            print(f"  Left (90°): {points_90deg[0]:.0f}mm")
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n\n⏹ Stopping...")
    
    finally:
        lidar.disconnect()
        print(f"✓ Total scans read: {scan_count}")


if __name__ == "__main__":
    main()

import serial
import time

port = "COM5"
baudrate = 115200
timeout = 5

try:
    with serial.Serial(port, baudrate, timeout=timeout) as ser:
        print(f"Connected to {port}")
        ser.write(bytes([0xA0]))  # Start scan command
        time.sleep(1)
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"Raw data: {data}")
            else:
                print("No data received")
            time.sleep(0.5)
except Exception as e:
    print(f"Error: {e}")
