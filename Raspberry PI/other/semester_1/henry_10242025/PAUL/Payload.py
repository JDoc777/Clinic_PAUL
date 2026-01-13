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

def _clamp_int16(x, lo=-255, hi=255):
    # Your Arduino uses values directly for mecanumDrive; clamp to PWM-ish range
    xi = int(x)
    if xi < lo: xi = lo
    if xi > hi: xi = hi
    return xi

def _clamp_u8(x, lo=0, hi=180):
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

def set_buzzer(shared_data, buzzerInt: int, buzzerDec: int):
    """
    Store buzzer intensity and duration (uint8). Clamped to [0, 255].
    """
    buzzer = (
        _clamp_u8(buzzerInt, 0, 3),
        _clamp_u8(buzzerDec, 0, 99),
    )
    shared_data.set_command_params(buzzerInt=buzzer[0], buzzerDec=buzzer[1])

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

def set_all(shared_data,
            flags: int = None,
            motors: tuple | list = None,
            servos: tuple | list = None,
            buzzer: tuple | list = None,
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
    if buzzer is not None:
        buzzerInt, buzzerDec = buzzer
        set_buzzer(shared_data, buzzerInt, buzzerDec)

# -------- Optional: small helpers to toggle bits --------

def enable(shared_data, mask: int):
    """OR a bitmask into flags (e.g., enable(sd, FLAG_RELAY))."""
    cur = shared_data.get_command_params()["flags"] & 0xFF
    shared_data.set_command_params(flags=(cur | (mask & 0xFF)))

def disable(shared_data, mask: int):
    """Clear a bitmask from flags."""
    cur = shared_data.get_command_params()["flags"] & 0xFF
    shared_data.set_command_params(flags=(cur & ~(mask & 0xFF)))
