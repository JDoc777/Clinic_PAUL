import serial
import time
import struct
import threading

# UART config
serial_port = "/dev/ttyAMA0"
baud_rate = 115200
ser = None   # DO NOT open at import

# Packet constants
START, END = 0x7E, 0x7F
PKT_TYPE_CMD  = 0x23   # Pi -> Arduino (command)
PKT_TYPE_DATA = 0x31   # Arduino -> Pi (telemetry)
PAYLOAD_LEN   = 58

# CRC16-CCITT
def crc16_ccitt(data: bytes, init=0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc

# ---- Command builder (unchanged) ----
def build_command_packet(flags, motors, servos, lcd_text="", buzzerInt=0, buzzerDec=0):
    m = tuple(max(-255, min(255, int(x))) for x in motors)
    s = tuple(max(0, min(180, int(x))) for x in servos)
    txt = (lcd_text or "").encode("utf-8")[:64]
    lcd_len = len(txt)
    buzzerInt = max(0, min(255, int(buzzerInt)))
    buzzerDec = max(0, min(255, int(buzzerDec)))

    # Pack flags, motor int16s, 5 servo u8s, then text bytes,
    # then text length and buzzer bytes (text comes BEFORE text_len on the wire)
    header_payload = struct.pack(
        "<BhhhhBBBBB",  # flags, 4 x int16, 5 x uint8 (servos)
        flags, m[0], m[1], m[2], m[3],
        s[0], s[1], s[2], s[3], s[4]
    )
    payload = header_payload + txt + struct.pack("<BBB", lcd_len, buzzerInt, buzzerDec)

    header = struct.pack("<BH", PKT_TYPE_CMD, len(payload))
    crc = crc16_ccitt(header + payload)
    return bytes([START]) + header + payload + struct.pack("<H", crc) + bytes([END])

# ---- Robust framed reader: seeks START and validates CRC ----
def read_frame(ser, want_type=PKT_TYPE_DATA, timeout=1.0):
    t0 = time.time()
    # seek START
    while time.time() - t0 < timeout:
        b = ser.read(1)
        if b == bytes([START]):
            break
    else:
        return None  # timeout

    hdr = ser.read(3)
    if len(hdr) != 3:
        return None
    pkt_type, len_lo, len_hi = hdr
    length = len_lo | (len_hi << 8)

    payload = ser.read(length)
    if len(payload) != length:
        return None

    crc_le = ser.read(2)
    if len(crc_le) != 2:
        return None
    rx_crc = struct.unpack("<H", crc_le)[0]

    if ser.read(1) != bytes([END]):
        return None

    calc = crc16_ccitt(bytes([pkt_type, len_lo, len_hi]) + payload)
    if calc != rx_crc:
        return None
    if want_type is not None and pkt_type != want_type:
        return None

    return payload

# ---- Correct payload struct (58 bytes) ----
# Layout matches Arduino struct:
# 4*int16 + 4*int32 + 8*uint8 + uint16 + 6*float  = 8 + 16 + 8 + 2 + 24 = 58
#payload_fmt = "<hhhhiiiiBBBBBBBBHffffff"
payload_fmt = "<hhhhiiiiBBBBBBBBHffffff"



def decode_payload(b: bytes):
    (sonar_F, sonar_B, sonar_L, sonar_R,
     enc_FR, enc_FL, enc_RR, enc_RL,
     dhti_hum_i, dhti_hum_d, dhti_tmp_i, dhti_tmp_d,
     dhto_hum_i, dhto_hum_d, dhto_tmp_i, dhto_tmp_d,
     relay_bits,
     gyro_pitch, gyro_yaw, gyro_roll,
     accel_surge, accel_heave, accel_sway) = struct.unpack(payload_fmt, b)

    return {
        "sonar":  dict(F=sonar_F, B=sonar_B, L=sonar_L, R=sonar_R),
        "encoder_raw": (enc_FR, enc_FL, enc_RR, enc_RL),  # just raw tuple
        "DHTI":   dict(hum=f"{dhti_hum_i}.{dhti_hum_d}", temp=f"{dhti_tmp_i}.{dhti_tmp_d}"),
        "DHTO":   dict(hum=f"{dhto_hum_i}.{dhto_hum_d}", temp=f"{dhto_tmp_i}.{dhto_tmp_d}"),
        "relay_bits": relay_bits,
        "gyro":   dict(pitch=gyro_pitch, yaw=gyro_yaw, roll=gyro_roll),
        "accel":  dict(surge=accel_surge, heave=accel_heave, sway=accel_sway),
    }

def open_serial(timeout=1):
    global ser
    if ser is None:
        ser = serial.Serial(serial_port, baud_rate, timeout=timeout)
    return ser

def close_serial():
    global ser
    if ser:
        try:
            ser.close()
        except Exception:
            pass
        ser = None

def start_reader(start_writer=True, do_handshake=True):
    """
    Open serial and start background read (and optional write) threads.
    Returns (shared_data, running_event, ser).
    """
    s = open_serial()
    if do_handshake:
        send_ascii_line("request")
        time.sleep(0.5)
        resp = recv_ascii_line(timeout=1.0)
        if resp != "ack":
            raise RuntimeError("Handshake failed or no response")
    s.reset_input_buffer()

    shared_data = UARTSharedData()
    running_event = threading.Event()
    running_event.set()

    read_thread = UARTReadThread(s, shared_data, running_event)
    read_thread.daemon = True
    read_thread.start()

    write_thread = None
    if start_writer:
        write_thread = UARTWriteThread(s, shared_data, running_event)
        write_thread.daemon = True
        write_thread.start()

    # Return handles so caller can stop/join if desired
    return shared_data, running_event, s

def send_ascii_line(s: str):
    # ensure serial open
    open_serial()
    ser.write((s + "\n").encode("utf-8"))

def recv_ascii_line(timeout=1.0):
    open_serial()
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = ser.readline()
        if not line:
            continue
        try:
            return line.decode("utf-8").strip()
        except UnicodeDecodeError:
            continue
    return None

class UARTSharedData:
    def __init__(self):
        self.lock = threading.Lock()
        self.command_params = {
            "flags": 0,
            "motors": (0, 0, 0, 0),
            "servos": (0, 0, 0, 0, 0),
            "lcd_text": "",
            "buzzerInt": 0,
            "buzzerDec": 0
        }
        self.received_data = None

    def set_command_params(self, flags=None, motors=None, servos=None, lcd_text=None, buzzerInt=None, buzzerDec=None):
        with self.lock:
            if flags is not None:
                self.command_params["flags"] = flags
            if motors is not None:
                self.command_params["motors"] = motors
            if servos is not None:
                self.command_params["servos"] = servos
            if lcd_text is not None:
                self.command_params["lcd_text"] = lcd_text
            if buzzerInt is not None:
                self.command_params["buzzerInt"] = buzzerInt
            if buzzerDec is not None:
                self.command_params["buzzerDec"] = buzzerDec

    def get_command_params(self):
        with self.lock:
            return self.command_params.copy()

    def set_received_data(self, data):
        with self.lock:
            self.received_data = data

    def get_received_data(self):
        with self.lock:
            return self.received_data

    def get_encoder_raw(self):
        with self.lock:
            if self.received_data and "encoder_raw" in self.received_data:
                return self.received_data["encoder_raw"]
            return None

class UARTWriteThread(threading.Thread):
    def __init__(self, ser, shared_data, running_event):
        super().__init__()
        self.ser = ser
        self.shared_data = shared_data
        self.running_event = running_event
        self.packet_count = 0

    def run(self):
        while self.running_event.is_set():
            params = self.shared_data.get_command_params()
            pkt = build_command_packet(
                flags=params["flags"],
                motors=params["motors"],
                servos=params["servos"],
                lcd_text=params["lcd_text"],
                buzzerInt=params["buzzerInt"],
                buzzerDec=params["buzzerDec"]
            )
            self.ser.write(pkt)
            self.packet_count += 1
            #print(f"Sent command packet #{self.packet_count}")
            time.sleep(0.01)

class UARTReadThread(threading.Thread):
    def __init__(self, ser, shared_data, running_event, debug_period=0.5):
        super().__init__()
        self.ser = ser
        self.shared_data = shared_data
        self.running_event = running_event
        self.packet_count = 0
        # debug_period: minimum seconds between debug prints to avoid clogging terminal
        self.debug_period = float(debug_period)
        self._last_debug_time = 0.0

    def run(self):
        while self.running_event.is_set():
            now = time.time()
            frame = read_frame(self.ser, want_type=PKT_TYPE_DATA, timeout=0.05)
            if frame:
                # only print debug info periodically (every self.debug_period seconds)
                if len(frame) != PAYLOAD_LEN:
                    if now - self._last_debug_time >= self.debug_period:
                        print(f"Got data but wrong length: {len(frame)}")
                        try:
                            print(f"DEBUG: Raw ({len(frame)} bytes): {frame.hex()}")
                        except Exception:
                            print(f"DEBUG: Raw ({len(frame)} bytes): {frame!r}")
                        self._last_debug_time = now
                else:
                    from struct import unpack
                    F, B, L, R = struct.unpack("<hhhh", frame[0:8])
                    print(f"RAW sonar shorts from wire: F={F} B={B} L={L} R={R}")
                    data = decode_payload(frame)
                    self.shared_data.set_received_data(data)
                    self.packet_count += 1
                    if now - self._last_debug_time >= self.debug_period:
                        try:
                            print(f"DEBUG: Received raw frame ({len(frame)} bytes): {frame.hex()}")
                        except Exception:
                            print(f"DEBUG: Received raw frame ({len(frame)} bytes): {frame!r}")
                        print(f"DEBUG: Decoded payload: {data}")
                        self._last_debug_time = now
            else:
                # print absence of valid packets only periodically
                if now - self._last_debug_time >= self.debug_period:
                    print("No valid data packet received.")
                    self._last_debug_time = now
            time.sleep(0.01)

def main():
    ser = open_serial(timeout=1)
    #ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print("Starting UART communication with Arduino...")

    # --- Handshake: send 'request', wait for 'ack' as ASCII ---
    send_ascii_line("request")
    time.sleep(0.5)
    resp = recv_ascii_line(timeout=1.0)
    if resp != "ack":
        print("Handshake failed or no response.")
        #ser.close()
        send_ascii_line("request")
        return
    print("Handshake successful!")
    ser.reset_input_buffer()

    shared_data = UARTSharedData()
    running_event = threading.Event()
    running_event.set()

    write_thread = UARTWriteThread(ser, shared_data, running_event)
    read_thread = UARTReadThread(ser, shared_data, running_event)
    write_thread.start()
    read_thread.start()

    # Example: update command params from another thread
    def update_commands():
        while running_event.is_set():
            shared_data.set_command_params(motors=(+200, -150, +0, +50), lcd_text="PAUL online")
            time.sleep(1)

    updater = threading.Thread(target=update_commands)
    updater.start()

    try:
        while True:
            data = shared_data.get_received_data()
            if data:
                time.sleep(0.5)
    except KeyboardInterrupt:
        running_event.clear()
        updater.join()
        write_thread.join()
        read_thread.join()
        ser.close()
        print("Stopped.")

if __name__ == "__main__":
    main()
