import pandas as pd
import matplotlib.pyplot as plt

# ===== CHANGE THIS TO YOUR FILE =====
CSV_FILE = "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Lidar_Testing_1_12_2026/debug_log.csv"
# ===================================

def main():
    # Load CSV
    df = pd.read_csv(CSV_FILE)

    # Check columns
    print("Columns:", df.columns)

    t = df["time"]

    # ===== PLOTS =====

    # 1. Velocity + Omega
    plt.figure()
    plt.plot(t, df["vx_cmd"], label="vx_cmd")
    plt.plot(t, df["omega_cmd"], label="omega_cmd")
    plt.title("Velocity Commands")
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.legend()
    plt.grid()

    # 2. Wheel Speeds
    plt.figure()
    plt.plot(t, df["w1"], label="FL")
    plt.plot(t, df["w2"], label="FR")
    plt.plot(t, df["w3"], label="BL")
    plt.plot(t, df["w4"], label="BR")
    plt.title("Wheel Speeds")
    plt.xlabel("Time (s)")
    plt.ylabel("Rad/s")
    plt.legend()
    plt.grid()

    # 3. Heading Error
    plt.figure()
    plt.plot(t, df["heading_error"])
    plt.title("Heading Error")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (rad)")
    plt.grid()

    # 4. Mode (ACK vs TIP)
    plt.figure()
    mode_numeric = df["mode"].astype("category").cat.codes
    plt.plot(t, mode_numeric)
    plt.title("Mode (ACK=0, TIP=1)")
    plt.xlabel("Time (s)")
    plt.ylabel("Mode")
    plt.grid()

    plt.show()


if __name__ == "__main__":
    main()