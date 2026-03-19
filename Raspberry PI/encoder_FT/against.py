import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

CSV_FILE = "wheel_omega_log.csv"
TIME_SCALE = 1_000_000.0  # micros -> seconds


def load_csv(csv_file: str) -> pd.DataFrame:
    expected_cols = [
        "arduino_time",
        "omega1", "omega2", "omega3", "omega4",
        "omega1_f", "omega2_f", "omega3_f", "omega4_f"
    ]

    # First try: read normally
    df = pd.read_csv(csv_file)

    print("Columns found:", list(df.columns))
    print("\nFirst 5 raw rows:")
    print(df.head())

    # Case 1: file already has proper headers
    if set(expected_cols).issubset(df.columns):
        pass

    # Case 2: file has no header, and first row got treated as header
    elif len(df.columns) == 9:
        df = pd.read_csv(csv_file, header=None, names=expected_cols)

    else:
        raise ValueError(
            "CSV does not match expected 9-column format.\n"
            f"Found columns: {list(df.columns)}"
        )

    # Convert all expected columns to numeric
    for col in expected_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    print("\nAfter numeric conversion, first 5 rows:")
    print(df.head())

    # Drop bad rows
    df = df.dropna(subset=expected_cols).reset_index(drop=True)

    if df.empty:
        raise ValueError(
            "No valid numeric rows found in the CSV.\n"
            "Check whether the file contains plain comma-separated numbers like:\n"
            "time,omega1,omega2,omega3,omega4,omega1_f,omega2_f,omega3_f,omega4_f\n"
            "32863011,-21.2,0.0,-16.4,-18.2,-20.1,0.0,-15.8,-17.5"
        )

    df["time_s"] = df["arduino_time"] / TIME_SCALE
    df["time_s"] = df["time_s"] - df["time_s"].iloc[0]

    return df


def print_info(df: pd.DataFrame) -> None:
    t = df["time_s"].to_numpy()
    if len(t) > 1:
        dt = np.diff(t)
        mean_dt = np.mean(dt)
        print(f"\nSamples: {len(df)}")
        print(f"Duration: {t[-1]:.3f} s")
        print(f"Mean dt: {mean_dt:.6f} s")
        print(f"Estimated fs: {1.0 / mean_dt:.3f} Hz")


def plot_raw_vs_filtered(df: pd.DataFrame) -> None:
    t = df["time_s"].to_numpy()

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    wheel_pairs = [
        ("omega1", "omega1_f", "Wheel 1"),
        ("omega2", "omega2_f", "Wheel 2"),
        ("omega3", "omega3_f", "Wheel 3"),
        ("omega4", "omega4_f", "Wheel 4"),
    ]

    for ax, (raw_col, filt_col, title) in zip(axes, wheel_pairs):
        ax.plot(t, df[raw_col].to_numpy(), label=raw_col)
        ax.plot(t, df[filt_col].to_numpy(), label=filt_col)
        ax.set_title(title)
        ax.set_ylabel("rad/s")
        ax.grid(True)
        ax.legend()

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()


def main():
    df = load_csv(CSV_FILE)
    print_info(df)
    plot_raw_vs_filtered(df)


if __name__ == "__main__":
    main()