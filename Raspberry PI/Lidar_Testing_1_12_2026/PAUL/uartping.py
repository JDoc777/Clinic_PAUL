import serial
import time

# Raspberry Pi 5 UART
ser = serial.Serial(
    port= "/dev/ttyAMA0",
    baudrate=115200,
    timeout=0.1
)

print("Raspberry Pi Ready")

last_ping = time.time()

while True:
    # Send ping every second
    if time.time() - last_ping > 1:
        ser.write(b"PING\n")
        print("Sent: PING")
        last_ping = time.time()

    # Read incoming
    if ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()

        if line:
            print(f"Received: {line}")

            if line == "PING":
                ser.write(b"PONG\n")
                print("Sent: PONG")

            elif line == "PONG":
                print("Handshake successful!")

    time.sleep(0.01)