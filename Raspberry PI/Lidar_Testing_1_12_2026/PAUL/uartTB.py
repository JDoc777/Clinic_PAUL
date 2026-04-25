import serial
import time

ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)

while True:
    ser.write(b"request\n")
    print("sent request")
    print("got:", ser.readline())
    time.sleep(1)