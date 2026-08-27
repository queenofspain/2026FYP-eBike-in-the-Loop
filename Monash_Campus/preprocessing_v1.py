# Pre-processing GPS data
# Recalculates the lat and lon of the gps data after processing (since I manually removed the gps earlier)

import pandas as pd
import numpy as np
import traci
import sumolib

NET_FILE = "Monash_Campus/2026-03-11-17-20-46/editedosm3.net.xml"
GPS_FILE = "Monash_Campus/Data/gps_old.csv"
GROUND_TRUTH_FILE = "Monash_Campus/Data/ground_truth_old.csv"

OUTPUT_FILE = "Monash_Campus/Data/gps_data_processed.csv"

gps = pd.read_csv(GPS_FILE)
ground_truth = pd.read_csv(GROUND_TRUTH_FILE)

gps = gps.dropna(subset=[
    "gps_x",
    "gps_y"
])

gps["gt_x"] = ground_truth["gt_x"].values
gps["gt_y"] = ground_truth["gt_y"].values

df = gps.copy()

# Remove rows with missing coordinate data
df = df.dropna(subset=[
    "gps_x",
    "gps_y",
    "gt_x",
    "gt_y"
]).copy()

# ============================================================
# Calculate positional error
# ============================================================

df["error_x"] = df["gps_x"] - df["gt_x"]
df["error_y"] = df["gps_y"] - df["gt_y"]

df["position_error"] = np.sqrt(
    df["error_x"]**2 +
    df["error_y"]**2
)

# ============================================================
# Section 5.2 — Newson & Krumm MAD estimator
# ============================================================

median_error = np.median(df["position_error"])
sigma_z = 1.4826 * median_error
# print(f"sigma_z = {sigma_z}")

keep_rows = []
previous_x = None
previous_y = None

for index, row in gps.iterrows():

    x = row["gps_x"]
    y = row["gps_y"]

    # Skip points with missing coordinates
    if pd.isna(x) or pd.isna(y):
        continue

    # First valid point is always kept
    if previous_x is None:
        keep_rows.append(index)

        previous_x = x
        previous_y = y

        continue

    # Calculate 2D distance from previous retained point
    distance = np.sqrt(
        (x - previous_x)**2 +
        (y - previous_y)**2
    )

    # Keep point only if it is at least sigma_z away
    if distance >= sigma_z:

        keep_rows.append(index)

        # Update previous retained point
        previous_x = x
        previous_y = y

gps_filtered = gps.loc[keep_rows].copy()

print("============================================================")
print("Filtering points < sigma_z away from previous")
print("============================================================")

print(f"Original points = {len(gps)}")
print(f"Removed points = {len(gps) - len(gps_filtered)}")
print(f"Filtered points = {len(gps_filtered)}")

# Convert filtered GPS x/y coordinates back to longitude/latitude
# Load SUMO network
net = sumolib.net.readNet(NET_FILE)

gps_filtered["lon"] = np.nan
gps_filtered["lat"] = np.nan

for index, row in gps_filtered.iterrows():

    x = row["gps_x"]
    y = row["gps_y"]

    lon, lat = net.convertXY2LonLat(x, y)

    gps_filtered.loc[index, "lon"] = lon
    gps_filtered.loc[index, "lat"] = lat

gps_filtered.to_csv(
    OUTPUT_FILE,
    index=False
)
