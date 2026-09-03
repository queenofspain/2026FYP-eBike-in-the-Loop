import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

# ---------- SETTINGS ----------
# CSV_FILE = "position_log.csv"
CSV_FILE = "full_campus_2.csv"
FRAME_INTERVAL_S = 0.025      # seconds between frames
# --------------------------------

# Load data
df = pd.read_csv(CSV_FILE)

# Remove rows with missing values
df = df.dropna(subset=[
    "gps_x", "gps_y",
    "keep0_sumo_x", "keep0_sumo_y",
    "keep1_sumo_x", "keep1_sumo_y"
])

gps_x = df["gps_x"]
gps_y = df["gps_y"]
k0_x = df["keep0_sumo_x"]
k0_y = df["keep0_sumo_y"]
k1_x = df["keep1_sumo_x"]
k1_y = df["keep1_sumo_y"]

# # Compute Euclidean distance
# df["keep_diff_m"] = np.sqrt(
#     (df["keep0_sumo_x"] - df["keep1_sumo_x"])**2 +
#     (df["keep0_sumo_y"] - df["keep1_sumo_y"])**2
# )

# Compute Euclidean distance
df["keep_diff_m"] = np.sqrt(
    (df["keep0_sumo_x"] - df["gps_x"])**2 +
    (df["keep0_sumo_y"] - df["gps_y"])**2
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

# Add in a threshold, since points may never perfectly line up
percentages = []
thresholds = [0.1, 0.25, 0.5, 1, 1.5, 1.75, 2, 2.5, 3, 5, 10]

for threshold in thresholds:
    # threshold = 1

    num_correct = np.sum(error <= threshold)
    total_points = len(error)

    if total_points > 0:
        correct_percentage = (num_correct / total_points) * 100
    else:
        correct_percentage = 0.0

    percentages.append(correct_percentage)

    # ---------- Print Results ----------
    print("\n===== keepRoute Comparison Statistics =====")
    print(f"Threshold (m):        {threshold}")
    print(f"Total Points:        {total_points}")
    print(f"Mean error (m):       {mean_error:.3f} m")
    print(f"RMSE (m):                {rmse:.3f} m")
    print(f"Max distance (m):        {max_error:.3f} m")
    print(f"Std Dev:             {std_dev:.3f} m")
    print(f"Correct Matches:     {num_correct}")
    print(f"Correct %:           {correct_percentage:.2f} %")
    print("===========================================")

lo = 153
hi = 186

local_error = np.sqrt((gps_x[lo:hi] - k0_x[lo:hi])**2 + (gps_y[lo:hi] - k0_y[lo:hi])**2)
local_mean = np.mean(local_error)
print(f"local mean = {local_mean:.2}")

tot = len(local_error)
correct = np.sum(local_error <= 1.5)

local_cmp = 100*correct/tot
print(f"local cmp = {local_cmp:.2}")

plt.figure(figsize=(10, 6))
bars = plt.bar([str(t) for t in thresholds], percentages, width=0.5)
plt.title("Threshold vs CMP")
plt.xlabel("Threshold value (m)")
plt.ylabel("Correctly matched points (%)")
# plt.xticks(thresholds)
plt.bar_label(bars, fmt='%.1f%%', padding=3)

plot_mode = 0

if plot_mode == 0:
    
# ============================================================
# STATIC PATH PLOT
# ============================================================

    # plt.figure()
    # plt.plot(time, error, linewidth=2)
    # plt.xlabel("Simulation Time (s)")
    # plt.ylabel("Distance (m)")
    # plt.title("Distance between eBike and route", fontweight='bold')
    # plt.xlim(time.min()-1, time.max()+1)
    # plt.ylim(error.min()-1, error.max() + (0.2*(error.max()-error.min())))
    # plt.grid(True, linestyle='--', color='#dbd8d0')
    # plt.show()

    # plt.figure()

    # plt.subplot(1,3,1)
    # plt.plot(gps_x, gps_y, linewidth=2)
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("GPS", fontweight='bold')
    # plt.grid(True, color='#dbd8d0')

    # plt.subplot(1,3,2)
    # plt.plot(k0_x, k0_y, linewidth=2)
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("KeepRoute = 0", fontweight='bold')
    # plt.grid(True, color='#dbd8d0')

    # plt.subplot(1,3,3)
    # plt.plot(k1_x, k1_y, linewidth=2)
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.title("KeepRoute = 1", fontweight='bold')
    # plt.grid(True, color='#dbd8d0')

    # plt.suptitle("N1 Route")


    plt.figure()

    # Excerpt
    # lines
    # plt.plot(gps_x[lo:hi], gps_y[lo:hi], '-', linewidth=2)
    # plt.plot(k0_x[lo:hi], k0_y[lo:hi], '--', linewidth=2)
    # points
    # plt.plot(gps_x[lo:hi], gps_y[lo:hi], '.', color="#45BEFF", linewidth=4)
    # plt.plot(k0_x[lo:hi], k0_y[lo:hi], '.', color="#ff0000", linewidth=4)
    
    # full path
    plt.plot(gps_x, gps_y, '-', linewidth=2)
    plt.plot(k0_x, k0_y, '--', linewidth=2)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Preliminary Route")
    plt.suptitle("GPS Path vs SUMO Path")
    # plt.axis([400, 600, 1200, 1300])
    # plt.legend(["GPS", "KeepRoute=0", "KeepRoute=1"])
    plt.legend(["GPS path", "SUMO path"])
    plt.axis('equal') 

    plt.show()

elif plot_mode == 1:

# ============================================================
# ANIMATED TRAJECTORY PLOT
# ============================================================

    plt.ion()

    fig, ax = plt.subplots()

    gps_plot, = ax.plot([], [], '.', markersize=8)
    sumo_plot, = ax.plot([], [], '.', markersize=8)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("GPS Path vs SUMO Path")

    # ---------- Set axis limits ----------
    all_x = np.concatenate([gps_x.values, k0_x.values])
    all_y = np.concatenate([gps_y.values, k0_y.values])

    padding = 20

    ax.set_xlim(all_x.min() - padding, all_x.max() + padding)
    ax.set_ylim(all_y.min() - padding, all_y.max() + padding)

    ax.axis('equal')

    ax.legend(["GPS path", "SUMO path"], loc='upper left')

    # ---------- Animate ----------
    for i in range(len(df)):

        gps_plot.set_data(gps_x[:i+1], gps_y[:i+1])
        sumo_plot.set_data(k0_x[:i+1], k0_y[:i+1])

        fig.canvas.draw()
        fig.canvas.flush_events()

        plt.pause(FRAME_INTERVAL_S)

    plt.ioff()
    plt.show()