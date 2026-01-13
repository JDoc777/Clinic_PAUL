import time
import struct
import serial
import numpy as np

# ===== EDIT THESE =====
PORT = "/dev/ttyUSB0"
BAUD = 230400
# ======================

SYNC_CANDIDATES = (b"\xAA\x55", b"\x55\xAA")

def find_first_sync(buf: bytearray):
    best_i = None
    best_s = None
    for s in SYNC_CANDIDATES:
        i = buf.find(s)
        if i != -1 and (best_i is None or i < best_i):
            best_i = i
            best_s = s
    return best_i, best_s

def try_parse_frame(buf: bytearray):
    """
    Common frame (best-effort):
      [sync(2)] [type(1)] [count(1)] [start_u16(2)] [end_u16(2)] [checksum_u16(2)] [dist_u16 * count]
    Angles assumed 0.01 deg. Distances in mm.
    Returns (pkt_type, angles_deg[np], dists_mm[np]) or None.
    """
    i, _ = find_first_sync(buf)
    if i is None:
        # trim if no sync found
        if len(buf) > 50000:
            del buf[:-20000]
        return None

    # drop junk before sync
    if i > 0:
        del buf[:i]

    MIN_HDR = 2 + 1 + 1 + 2 + 2 + 2
    if len(buf) < MIN_HDR:
        return None

    pkt_type = buf[2]
    n = buf[3]
    if n == 0 or n > 200:
        # probably false sync; shift and retry later
        del buf[:1]
        return None

    start_raw = struct.unpack_from("<H", buf, 4)[0]
    end_raw   = struct.unpack_from("<H", buf, 6)[0]

    start_deg = (start_raw / 100.0) % 360.0
    end_deg   = (end_raw   / 100.0) % 360.0
    if end_deg < start_deg:
        end_deg += 360.0

    samples_off = MIN_HDR
    samples_bytes = n * 2
    total_len = samples_off + samples_bytes
    if len(buf) < total_len:
        return None

    frame = bytes(buf[:total_len])
    del buf[:total_len]

    d = np.frombuffer(frame[samples_off:samples_off + samples_bytes], dtype=np.uint16).astype(np.float32)

    # angles linearly across the packet
    if n == 1:
        a = np.array([start_deg % 360.0], dtype=np.float32)
    else:
        a = (np.linspace(start_deg, end_deg, n, endpoint=True) % 360.0).astype(np.float32)

    # filter invalid zeros
    valid = d > 0
    return pkt_type, a[valid], d[valid]

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.2)
    print(f"Opened {PORT} @ {BAUD}")

    buf = bytearray()

    last = time.time()
    bytes_in = 0
    frames = 0
    points = 0

    while True:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
            bytes_in += len(chunk)

        # parse multiple frames per loop (bounded)
        for _ in range(50):
            out = try_parse_frame(buf)
            if out is None:
                break
            _, angles, dists = out
            frames += 1

            for ang, dist in zip(angles, dists):
                print(f"angle={ang:0.1f} deg  dist={dist:0.0f} mm")
                points += 1

        now = time.time()
        if now - last >= 1.0:
            print(f"[stats] bytes/s={bytes_in} frames/s={frames} points/s={points} buf={len(buf)}")
            bytes_in = 0
            frames = 0
            points = 0
            last = now

if __name__ == "__main__":
    main()
