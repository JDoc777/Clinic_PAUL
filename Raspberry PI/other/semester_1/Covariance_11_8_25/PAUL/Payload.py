# PiOut.py
# Compose and send command fields (flags, motors, servos, text) via shared_data.
# Matches Arduino applyCommand(flags, int16 m[4], uint8 s[4], const char* text, uint8 text_len)

# -------- Optional flag bit constants --------
FLAG_LED0      = 1 << 0  # bit 0
FLAG_LED1      = 1 << 1  # bit 1
FLAG_RELAY     = 1 << 2  # bit 2
FLAG_HANDSHAKE = 1 << 3  # bit 3
FLAG_RESET     = 1 << 4  # bit 4
FLAG_RESERVED5 = 1 << 5  # bit 5
FLAG_RESERVED6 = 1 << 6  # bit 6
FLAG_RESERVED7 = 1 << 7  # bit 7

import time
import threading

def _clamp_int16(x, lo=-255, hi=255):
    # Your Arduino uses values directly for mecanumDrive; clamp to PWM-ish range
    xi = int(x)
    if xi < lo: xi = lo
    if xi > hi: xi = hi
    return xi

def _clamp_u8(x, lo=0, hi=255):
    xi = int(x)
    if xi < lo: xi = lo
    if xi > hi: xi = hi
    return xi

def compose_flags(
    led0=False, led1=False, relay=False, handshake=False, reset=False,
    reserved5=False, reserved6=False, reserved7=False
) -> int:
    """Turn 8 booleans into a single 8-bit integer (0–255)."""
    return (
        (1 if led0 else 0)       << 0 |
        (1 if led1 else 0)       << 1 |
        (1 if relay else 0)      << 2 |
        (1 if handshake else 0)  << 3 |
        (1 if reset else 0)      << 4 |
        (1 if reserved5 else 0)  << 5 |
        (1 if reserved6 else 0)  << 6 |
        (1 if reserved7 else 0)  << 7
    ) & 0xFF

# ---------------- Public helpers ----------------

def set_flags(shared_data, **kwargs) -> int:
    """
    Build the flags byte from booleans and store in shared_data.
    Usage: set_flags(sd, led0=True, relay=True)  # sets flags=0b00000101 (5)
    """
    flags = compose_flags(**kwargs)
    shared_data.set_command_params(flags=flags)
    return flags

def set_flags_byte(shared_data, flags_byte: int) -> int:
    """Store a precomputed 0–255 flags value directly."""
    flags = int(flags_byte) & 0xFF
    shared_data.set_command_params(flags=flags)
    return flags

def set_motors(shared_data, fl: int, fr: int, rl: int, rr: int):
    """
    Order matches Arduino applyCommand: m[0]=fl, m[1]=fr, m[2]=rl, m[3]=rr
    Values are clamped to [-255, 255] and stored as int16 on send.
    """
    motors = (
        _clamp_int16(fl),
        _clamp_int16(fr),
        _clamp_int16(rl),
        _clamp_int16(rr),
    )
    shared_data.set_command_params(motors=motors)

def set_servos(shared_data, s0: int, s1: int, s2: int, s3: int, s4: int):
    """
    Store five servo positions (uint8). Clamped to [0, 180].
    """
    servos = (
        _clamp_u8(s0),
        _clamp_u8(s1),
        _clamp_u8(s2),
        _clamp_u8(s3),
        _clamp_u8(s4),
    )
    shared_data.set_command_params(servos=servos)

def set_text(shared_data, text: str):
    """
    Store LCD text. The wire format includes a 1-byte length (0–255),
    so keep this <= 255 bytes when UTF-8 encoded.
    """
    t = (text or "")
    if len(t.encode("utf-8")) > 255:
        # hard trim in bytes without splitting multibyte chars
        b = t.encode("utf-8")[:255]
        t = b.decode("utf-8", errors="ignore")
    shared_data.set_command_params(lcd_text=t)

def buzzer(shared_data, int_part: int, dec_part: int):
    """
    Set buzzer frequency as two parts: integer (thousands) and decimal (hundreds/tens/units).
    E.g., for 3520 Hz, use (3, 520). For 3140 Hz, use (3, 140).
    Both int_part and dec_part are clamped to [0, 255].
    """
    i = _clamp_u8(int_part)
    d = _clamp_u8(dec_part)
    shared_data.set_command_params(buzzer=(i, d))


def happy_birthday(shared_data, tempo: float = 0.4, blocking: bool = True, repeat: int = 1):
    """Play the 'Happy Birthday' melody on the buzzer.

    - shared_data: object exposing set_command_params(buzzer=(int,dec)).
    - tempo: seconds per beat (float). Smaller => faster.
    - blocking: if True, function blocks until melody finishes; if False, plays in background thread.
    - repeat: how many times to play the melody.

    Uses the existing `buzzer` helper to clamp values and set the buzzer. Ensures buzzer is turned off
    at the end of playback.
    """

    # Note frequencies (Hz) for common notes used in the melody
    freqs = {
        'C7': 2090, 'CS7': 2220, 'D7': 2350, 'DS7': 2490,
        'E7': 2640, 'F7': 2790, 'FS7': 2960, 'G7': 3140,
        'GS7': 3320, 'A7': 3520, 'AS7': 3730, 'B7': 3950,
        'REST': 0
    }

    # Melody: (note, beats). Use None for a rest.
    melody = [
        
        ("E7", 8), ("E7", 8), ("REST", 8), ("E7", 8), ("REST", 8), ("C7", 8), ("E7", 8),
        ("G7", 4), ("REST", 4), ("G7", 8), ("REST", 4),
        ("G7", 8), ("REST", 4), ("E7", 4),
        ("A7", 4), ("B7", 4), ("AS7", 8), ("A7", 4),
        ("G7", 8), ("E7", 8), ("G7", 8), ("A7", 4), ("F7", 8), ("G7", 8),
        ("REST", 8), ("E7", 4), ("C7", 8), ("D7", 8), ("B7", 4),
 
        ("C7", 4), ("G7", 8), ("REST", 4), ("E7", 4),
        ("A7", 4), ("B7", 4), ("AS7", 8), ("A7", 4),
        ("G7", 8), ("E7", 8), ("G7", 8), ("A7", 4), ("F7", 8), ("G7", 8),
        ("REST", 8), ("E7", 4), ("C7", 8), ("D7", 8), ("B7", 4),

        ("REST", 4), ("G7", 8), ("FS7", 8), ("F7", 8), ("DS7", 8), ("E7", 4),
        ("REST", 8), ("GS7", 8), ("A7", 8), ("C7", 8), ("REST", 8), ("A7", 8), ("C7", 8), ("D7", 8),
        ("REST", 4), ("DS7", 4), ("REST", 8), ("D7", 4),
        ("C7", 2), ("REST", 2),

        ("REST", 4), ("G7", 8), ("FS7", 8), ("F7", 8), ("DS7", 8), ("E7", 4),
        ("REST", 8), ("GS7", 8), ("A7", 8), ("C7", 8), ("REST", 8), ("A7", 8), ("C7", 8), ("D7", 8),
        ("REST", 4), ("DS7", 4), ("REST", 8), ("D7", 4),
        ("C7", 2), ("REST", 2),

        ("C7", 8), ("C7", 4), ("C7", 8), ("REST", 8), ("C7", 8), ("D7", 4),
        ("E7", 8), ("C7", 4), ("A7", 8), ("G7", 2),

        ("C7", 8), ("C7", 4), ("C7", 8), ("REST", 8), ("C7", 8), ("D7", 8), ("E7", 8),
        ("REST", 1),
        ("C7", 8), ("C7", 4), ("C7", 8), ("REST", 8), ("C7", 8), ("D7", 4),
        ("E7", 8), ("C7", 4), ("A7", 8), ("G7", 2),
        ("E7", 8), ("E7", 8), ("REST", 8), ("E7", 8), ("REST", 8), ("C7", 8), ("E7", 8),
        ("G7", 4), ("REST", 4), ("G7", 8), ("REST", 4),
        ("C7", 4), ("G7", 8), ("REST", 4), ("E7", 4),

        ("A7", 4), ("B7", 4), ("AS7", 8), ("A7", 4),
        ("G7", 8), ("E7", 8), ("G7", 8), ("A7", 4), ("F7", 8), ("G7", 8),
        ("REST", 8), ("E7", 4), ("C7", 8), ("D7", 8), ("B7", 4),

        ("C7", 4), ("G7", 8), ("REST", 4), ("E7", 4),
        ("A7", 4), ("B7", 4), ("AS7", 8), ("A7", 4),
        ("G7", 8), ("E7", 8), ("G7", 8), ("A7", 4), ("F7", 8), ("G7", 8),
        ("REST", 8), ("E7", 4), ("C7", 8), ("D7", 8), ("B7", 4),

        ("E7", 8), ("C7", 4), ("G7", 8), ("REST", 4), ("GS7", 4),
        ("A7", 8), ("F7", 4), ("F7", 8), ("A7", 2),
        ("D7", 8), ("A7", 4), ("A7", 8), ("A7", 8), ("G7", 8), ("F7", 8),

        ("E7", 8), ("C7", 4), ("A7", 8), ("G7", 4),
        ("E7", 8), ("C7", 4), ("G7", 8), ("REST", 4), ("GS7", 4),
        ("A7", 8), ("F7", 4), ("F7", 8), ("A7", 2),
        ("B7", 8), ("F7", 4), ("F7", 8), ("F7", 8), ("E7", 8), ("D7", 8),
        ("C7", 8), ("E7", 4), ("E7", 8), ("C7", 8),

        ("E7", 8), ("C7", 4), ("G7", 8), ("REST", 4), ("GS7", 4),
        ("A7", 8), ("F7", 4), ("F7", 8), ("A7", 2),
        ("D7", 8), ("A7", 4), ("A7", 8), ("A7", 8), ("G7", 8), ("F7", 8),

        ("E7", 8), ("C7", 4), ("A7", 8), ("G7", 4),
        ("E7", 8), ("C7", 4), ("G7", 8), ("REST", 4), ("GS7", 4),
        ("A7", 8), ("F7", 4), ("F7", 8), ("A7", 2),
        ("B7", 8), ("F7", 4), ("F7", 8), ("F7", 8), ("E7", 8), ("D7", 8),
        ("C7", 8), ("E7", 4), ("E7", 8), ("C7", 8),
        ("C7", 8), ("C7", 4), ("C7", 8), ("REST", 8), ("C7", 8), ("D7", 4), ("E7", 8),
        ("REST", 1),

        ("C7", 8), ("C7", 4), ("C7", 8), ("REST", 8), ("C7", 8), ("D7", 4),
        ("E7", 8), ("C7", 4), ("A7", 8), ("G7", 2),
        ("E7", 8), ("E7", 8), ("REST", 8), ("E7", 8), ("REST", 8), ("C7", 8), ("E7", 8),
        ("G7", 4), ("REST", 4), ("G7", 8), ("REST", 4),
        ("E7", 8), ("C7", 4), ("G7", 8), ("REST", 4), ("GS7", 4),
        ("A7", 8), ("F7", 4), ("F7", 8), ("A7", 2),
        ("D7", 8), ("A7", 4), ("A7", 8), ("A7", 8), ("G7", 8), ("F7", 8),

        ("E7", 8), ("C7", 4), ("A7", 8), ("G7", 4),
        ("E7", 8), ("C7", 4), ("G7", 8), ("REST", 4), ("GS7", 4),
        ("A7", 8), ("F7", 4), ("F7", 8), ("A7", 2),
        ("B7", 8), ("F7", 4), ("F7", 8), ("F7", 8), ("E7", 8), ("D7", 8),
        ("C7", 8), ("E7", 4), ("E7", 8), ("C7", 8),

    # Game over sound
        ("C7", 4), ("G7", 4), ("E7", 4),
        ("A7", 8), ("B7", 8), ("A7", 8), ("GS7", 8), ("AS7", 8), ("GS7", 8),
        ("G7", 8), ("D7", 8), ("E7", 2)



    ]

    def play_once():
        try:
            for note, beats in melody:
                # Normalize durations:
                # - If beats is a large integer like 4,8,16 treat it as a note denominator
                #   (4 => quarter note => duration = tempo, 8 => eighth => tempo*0.5).
                # - Otherwise treat beats as number of beats (1.0 = one beat => tempo seconds).
                b = float(beats)
                if b >= 4.0 and abs(b - round(b)) < 1e-8:
                    # denominator-style: duration = tempo * (4 / beats)
                    dur = max(0.0, float(tempo) * (4.0 / b))
                else:
                    dur = max(0.0, b * float(tempo))
                if note is None:
                    # rest
                    buzzer(shared_data, 0, 0)
                    time.sleep(dur)
                    continue

                f = freqs.get(note)
                if f is None:
                    # unknown note: treat as rest
                    buzzer(shared_data, 0, 0)
                    time.sleep(dur)
                    continue

                # split frequency into two parts: ip = freq // 1000, dp = freq % 1000
                # e.g. 3520 -> ip=3, dp=520. REST (0) maps to (0,0).
                if f == 0:
                    print("REST")
                    ip = 0
                    dp = 0
                else:
                    #f = f / 8.0  # adjust for buzzer scaling
                    fi = int(f)
                    ip = fi // 1000
                    dp = (fi % 1000) // 10
                # play slightly less than full beat to create separation
                on_time = dur * 0.9
                off_time = max(0.0, dur - on_time)

                buzzer(shared_data, ip, dp)
                time.sleep(on_time)
                buzzer(shared_data, 0, 0)
                time.sleep(off_time)
        finally:
            # ensure buzzer silenced at end
            buzzer(shared_data, 0, 0)

    if repeat <= 0:
        return None

    if blocking:
        for _ in range(int(repeat)):
            play_once()
        return None
    else:
        def _bg():
            for _ in range(int(repeat)):
                play_once()

        t = threading.Thread(target=_bg, daemon=True)
        t.start()
        return t


def set_all(shared_data,
            flags: int = None,
            motors: tuple | list = None,
            servos: tuple | list = None,
            text: str = None):
    """
    Convenience: set any subset of (flags, motors, servos, text) in one call.
    Motors must be (fl, fr, rl, rr). Servos must be 4 elements.
    """
    if motors is not None:
        fl, fr, rl, rr = motors
        set_motors(shared_data, fl, fr, rl, rr)
    if servos is not None:
        s0, s1, s2, s3, s4 = servos
        set_servos(shared_data, s0, s1, s2, s3, s4)
    if text is not None:
        set_text(shared_data, text)
    if flags is not None:
        set_flags_byte(shared_data, flags)

# -------- Optional: small helpers to toggle bits --------

def enable(shared_data, mask: int):
    """OR a bitmask into flags (e.g., enable(sd, FLAG_RELAY))."""
    cur = shared_data.get_command_params()["flags"] & 0xFF
    shared_data.set_command_params(flags=(cur | (mask & 0xFF)))

def disable(shared_data, mask: int):
    """Clear a bitmask from flags."""
    cur = shared_data.get_command_params()["flags"] & 0xFF
    shared_data.set_command_params(flags=(cur & ~(mask & 0xFF)))
