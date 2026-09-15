# Pre-processing GPS data
# Assumes the lat/lon are the same size as the gps (I need to remove these as well as the gps)

import pandas as pd
import numpy as np
import traci
import sumolib

NET_FILE = "Monash_Campus/2026-08-25-19-43-30/osm.net.xml"
GPS_FILE = "Monash_Campus/Data/edited_100926.csv"
GROUND_TRUTH_FILE = "Monash_Campus/Data/ground_truth_100926.csv"

OUTPUT_FILE = "Monash_Campus/Data/gps_data_processed.csv"

gps = pd.read_csv(GPS_FILE)
ground_truth = pd.read_csv(GROUND_TRUTH_FILE)

gps = gps.dropna(subset=[
    "raw_x",
    "raw_y"
])

gps["actual_x"] = ground_truth["actual_x"].values
gps["actual_y"] = ground_truth["actual_y"].values

df = gps.copy()

# Remove rows with missing coordinate data
df = df.dropna(subset=[
    "raw_x",
    "raw_y",
    "actual_x",
    "actual_y"
]).copy()

# ============================================================
# Calculate positional error
# ============================================================

df["error_x"] = df["raw_x"] - df["actual_x"]
df["error_y"] = df["raw_y"] - df["actual_y"]

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

    x = row["raw_x"]
    y = row["raw_y"]

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

gps_filtered.to_csv(
    OUTPUT_FILE,
    index=False
)
