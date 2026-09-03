"""
Error calculation functions
Written by: Jordan Budiono
Date: 25/04/2026
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# -------- SETTINGS --------
CSV_FILE = "position_log.csv"
INVALID_VALUE = -1073741824  # SUMO invalid double
# --------------------------

def main():
    # Load CSV
    df = pd.read_csv(CSV_FILE)

    # Remove invalid SUMO rows
    df = df[
        (df["sumo_x"] != INVALID_VALUE) &
        (df["sumo_y"] != INVALID_VALUE)
    ]

    # Compute Euclidean distance error (meters)
    df["position_error_m"] = np.sqrt(
        (df["gps_x"] - df["sumo_x"])**2 +
        (df["gps_y"] - df["sumo_y"])**2
    )

    # ---- Plot Error vs Time ----
    plt.figure()
    plt.plot(df["sim_time"], df["position_error_m"])
    plt.xlabel("Simulation Time (s)")
    plt.ylabel("Position Error (meters)")
    plt.title("GPS vs SUMO Position Error Over Time")
    plt.grid(True)

    # ---- Print Statistics ----
    print("\nError Statistics:")
    print(f"Mean Error:   {df['position_error_m'].mean():.2f} m")
    print(f"Median Error: {df['position_error_m'].median():.2f} m")
    print(f"Max Error:    {df['position_error_m'].max():.2f} m")
    print(f"Min Error:    {df['position_error_m'].min():.2f} m")
    print(f"RMSE:         {np.sqrt(np.mean(df['position_error_m']**2)):.2f} m")

    plt.show()


if __name__ == "__main__":
    main()