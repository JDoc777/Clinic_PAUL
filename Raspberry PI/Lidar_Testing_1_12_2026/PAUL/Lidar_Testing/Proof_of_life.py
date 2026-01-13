import time
import serial

PORT = "/dev/ttyUSB0"     # change if needed
BAUD = 115200             # common for many YDLIDARs; try 115200 if zero bytes

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print(f"Opened {PORT} @ {BAUD}")
for i in range(30):
    data = ser.read(400)
    print(f"{i:02d}: {len(data)} bytes", (" | " + data[:16].hex()) if data else "")
ser.close()
