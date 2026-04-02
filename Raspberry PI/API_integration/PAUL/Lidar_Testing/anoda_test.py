#!/usr/bin/env python3
import time
import struct
import serial

PORT = "/dev/ttyUSB0"
BAUD = 230400

SYNC_LIST = (b"\xAA\x55", b"\x55\xAA")  # try both

RMIN_M = 0.02   # relaxed so we don't filter everything
RMAX_M = 12.0

def find_sync(buf: bytearray):
    best_i = -1
    best_s = None
    for s in SYNC_LIST:
        i = buf.find(s)
        if i != -1 and (best_i == -1 or i < best_i):
            best_i = i
            best_s = s
    return best_i, best_s

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.2)
    ser.reset_input_buffer()
    print(f"Opened {PORT} @ {BAUD}")

    buf = bytearray()

    last = time.time()
    bytes_in = 0
    frames = 0
    points = 0
    no_sync_secs = 0

    while True:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
            bytes_in += len(chunk)

        # keep buffer bounded
        if len(buf) > 200000:
            del buf[:100000]

        # parse as many frames as possible
        parsed_this_loop = False
        for _ in range(50):
            i, sync = find_sync(buf)
            if i == -1:
                break

            # drop junk before sync
            if i > 0:
                del buf[:i]

            # need sync (2) + fixed header (8)
            if len(buf) < 10:
                break

            # Layout assumption (X2/X2L common):
            # sync(2) CT(1) LSN(1) FSA(2) LSA(2) CS(2) samples(2*LSN)
            CT = buf[2]
            LSN = buf[3]

            # sanity
            if LSN == 0 or LSN > 200:
                del buf[:1]   # shift and resync
                continue

            FSA_raw = struct.unpack_from("<H", buf, 4)[0]
            LSA_raw = struct.unpack_from("<H", buf, 6)[0]
            CS      = struct.unpack_from("<H", buf, 8)[0]

            frame_len = 10 + 2 * LSN
            if len(buf) < frame_len:
                break

            frame = bytes(buf[:frame_len])
            del buf[:frame_len]
            parsed_this_loop = True

            # decode angles (common formula)
            start_deg = ((FSA_raw >> 1) / 64.0) % 360.0
            end_deg   = ((LSA_raw >> 1) / 64.0) % 360.0
            if end_deg < start_deg:
                end_deg += 360.0

            if LSN == 1:
                angles = [start_deg]
            else:
                step = (end_deg - start_deg) / (LSN - 1)
                angles = [((start_deg + k * step) % 360.0) for k in range(LSN)]

            samples = struct.unpack_from("<" + "H" * LSN, frame, 10)

            # auto-pick distance scale: try 0.25mm vs 1.0mm
            # (we score based on how many fall into a reasonable range)
            def score(scale_mm):
                c = 0
                for raw in samples:
                    units = (raw & 0xFFFC)
                    dm = (units * scale_mm) / 1000.0
                    if RMIN_M <= dm <= RMAX_M:
                        c += 1
                return c

            scale_mm = 0.25 if score(0.25) >= score(1.0) else 1.0

            frames += 1

            # print a short header line occasionally
            if frames % 20 == 1:
                print(f"[frame] sync={sync.hex()} CT=0x{CT:02x} LSN={LSN} "
                      f"FSA=0x{FSA_raw:04x} LSA=0x{LSA_raw:04x} CS=0x{CS:04x} scale={scale_mm}mm")

            # print points (limit per frame so we don't spam too hard)
            printed_here = 0
            for ang, raw in zip(angles, samples):
                units = (raw & 0xFFFC)
                dist_m = (units * scale_mm) / 1000.0
                if RMIN_M <= dist_m <= RMAX_M:
                    print(f"angle={ang:0.1f} deg  dist={dist_m*1000.0:0.0f} mm")
                    points += 1
                    printed_here += 1
                    if printed_here >= 25:
                        break

        now = time.time()
        if now - last >= 1.0:
            if not parsed_this_loop:
                no_sync_secs += 1
            else:
                no_sync_secs = 0

            head = " ".join(f"{b:02x}" for b in buf[:16])
            print(f"[stats] bytes/s={bytes_in} frames/s={frames} points/s={points} buf={len(buf)} head={head} no_sync_s={no_sync_secs}")
            bytes_in = 0
            frames = 0
            points = 0
            last = now

if __name__ == "__main__":
    main()
