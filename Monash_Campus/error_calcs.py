import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math

# ---------- SETTINGS ----------
CSV_FILE = "good_data.csv"
# --------------------------------

# Load data
df = pd.read_csv(CSV_FILE)

# Remove rows with missing values
df = df.dropna(subset=[
    "keep0_sumo_x", "keep0_sumo_y",
    "keep1_sumo_x", "keep1_sumo_y"
])

# Compute Euclidean distance
df["keep_diff_m"] = np.sqrt(
    (df["keep0_sumo_x"] - df["keep1_sumo_x"])**2 +
    (df["keep0_sumo_y"] - df["keep1_sumo_y"])**2
)

time = df["sim_time"].values
error = df["keep_diff_m"].values

# ---------- Statistics ----------
mean_error = np.mean(error)
rmse = math.sqrt(np.mean(error**2))
max_error = np.max(error)
std_dev = np.std(error)

# ---------- Correct Matching Percentage ----------
# Exact zero comparison (as you defined)
num_correct = np.sum(error == 0)
total_points = len(error)

if total_points > 0:
    correct_percentage = (num_correct / total_points) * 100
else:
    correct_percentage = 0.0

# ---------- Print Results ----------
print("\n===== keepRoute Comparison Statistics =====")
print(f"Total Points:        {total_points}")
print(f"Mean distance:       {mean_error:.3f} m")
print(f"RMSE:                {rmse:.3f} m")
print(f"Max distance:        {max_error:.3f} m")
print(f"Std Dev:             {std_dev:.3f} m")
print(f"Correct Matches:     {num_correct}")
print(f"Correct %:           {correct_percentage:.2f} %")
print("===========================================")

# ---------- Plot ----------
plt.figure()
plt.plot(time, error, linewidth=2)
plt.xlabel("Simulation Time (s)")
plt.ylabel("Distance (m)")
plt.title("Distance between eBike and route", fontweight='bold')
plt.xlim(time.min()-1, time.max()+1)
plt.ylim(error.min()-1, error.max() + (0.2*(error.max()-error.min())))
plt.grid(True, linestyle='--', color='#dbd8d0')
plt.show()