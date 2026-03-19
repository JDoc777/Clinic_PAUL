import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# CONFIG
# =========================
CSV_FILE = "wheel_omega_log.csv"
FS = 100.0   # sampling frequency in Hz (change if needed)

# =========================
# LOAD CSV
# =========================
df = pd.read_csv(CSV_FILE)

required_cols = [
    "omega1", "omega2", "omega3", "omega4",
    "omega1_f", "omega2_f", "omega3_f", "omega4_f"
]
for col in required_cols:
    if col not in df.columns:
        raise ValueError(f"Missing required column: {col}")

# =========================
# FFT FUNCTION
# =========================
def compute_fft(signal, fs):
    signal = np.asarray(signal, dtype=float)
    n = len(signal)

    if n < 2:
        raise ValueError("Not enough samples for FFT.")

    signal = signal - np.mean(signal)  # remove DC offset

    fft_vals = np.fft.rfft(signal)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)

    magnitude = np.abs(fft_vals) / n
    if len(magnitude) > 2:
        magnitude[1:-1] *= 2

    return freqs, magnitude

# =========================
# FIRST 4 VALUES
# =========================
omega1 = df["omega1"].to_numpy(dtype=float)
omega2 = df["omega2"].to_numpy(dtype=float)
omega3 = df["omega3"].to_numpy(dtype=float)
omega4 = df["omega4"].to_numpy(dtype=float)

freq1, mag1 = compute_fft(omega1, fs=FS)
freq2, mag2 = compute_fft(omega2, fs=FS)
freq3, mag3 = compute_fft(omega3, fs=FS)
freq4, mag4 = compute_fft(omega4, fs=FS)

# =========================
# NEXT 4 VALUES
# =========================
omega1_f = df["omega1_f"].to_numpy(dtype=float)
omega2_f = df["omega2_f"].to_numpy(dtype=float)
omega3_f = df["omega3_f"].to_numpy(dtype=float)
omega4_f = df["omega4_f"].to_numpy(dtype=float)

freq1f, mag1f = compute_fft(omega1_f, fs=FS)
freq2f, mag2f = compute_fft(omega2_f, fs=FS)
freq3f, mag3f = compute_fft(omega3_f, fs=FS)
freq4f, mag4f = compute_fft(omega4_f, fs=FS)

# =========================
# PLOT 1: FIRST 4 VALUES
# =========================
plt.figure(figsize=(10, 6))
plt.plot(freq1, mag1, label="omega1")
plt.plot(freq2, mag2, label="omega2")
plt.plot(freq3, mag3, label="omega3")
plt.plot(freq4, mag4, label="omega4")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.title("FFT of First 4 Values")
plt.legend()
plt.grid(True)
plt.tight_layout()

# =========================
# PLOT 2: NEXT 4 VALUES
# =========================
plt.figure(figsize=(10, 6))
plt.plot(freq1f, mag1f, label="omega1_f")
plt.plot(freq2f, mag2f, label="omega2_f")
plt.plot(freq3f, mag3f, label="omega3_f")
plt.plot(freq4f, mag4f, label="omega4_f")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.title("FFT of Next 4 Values")
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()