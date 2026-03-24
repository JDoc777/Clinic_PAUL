import pandas as pd
import matplotlib.pyplot as plt
import os

# =========================
# CONFIG
# =========================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_FILE = os.path.join(SCRIPT_DIR, 'wheel_omega_log.csv')

# =========================
# LOAD CSV
# =========================
df = pd.read_csv(INPUT_FILE)

# Convert time to seconds (normalize start to 0)
t = df["arduino_time"].values
t = (t - t[0]) / 1000.0  # assuming Arduino time is in ms

# =========================
# PLOT FUNCTION
# =========================
def plot_wheel(wheel_num):
    raw = df[f"omega{wheel_num}"]
    filt = df[f"omega{wheel_num}_f"]

    plt.figure()

    plt.plot(t, raw, label=f"omega{wheel_num} raw", linestyle='dotted')
    plt.plot(t, filt, label=f"omega{wheel_num} filtered", linewidth=2)

    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.title(f"Wheel {wheel_num} PID Response")
    plt.grid(True)
    plt.legend()

# =========================
# PLOT ALL WHEELS
# =========================
for i in range(1, 5):
    plot_wheel(i)

plt.show()