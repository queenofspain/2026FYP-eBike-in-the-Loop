"""
Plot each matched-route CSV against the ground truth trajectory.

For each `matched_*` CSV, this produces one plot containing:
  - Ground truth path (actual_x / actual_y): line + dots
  - That method's matched path (matched_x / matched_y): line + dots

The ground truth's true position (actual_x, actual_y) is identical across
every matched file (it's the input trajectory fed into each matcher), so it
is NOT what differs per method. The column that actually varies per matching
method/run is matched_x / matched_y -- that's what's plotted here as "the
matched route".

Titles clearly state the method name and whether Kalman filtering was applied,
pulled directly from each file's `method` and `kalman` columns.

Usage:
    python plot_matched_routes.py

Expects all CSVs (matched_*.csv and ground_truth_*.csv) in the same
directory as this script (or edit DATA_DIR below), and writes one PNG
per matched file into OUTPUT_DIR.
"""

import glob
import os

import matplotlib.pyplot as plt
import pandas as pd

# ---- Configuration -------------------------------------------------------

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(DATA_DIR, "plots")

GROUND_TRUTH_GLOB = "Data\ground_truth_*.csv"
MATCHED_GLOB = "Data\matched_*.csv"

# ---------------------------------------------------------------------------


def to_bool(value) -> bool:
    """Robustly interpret the `kalman` column, which shows up as True/False,
    'True'/'False', or 'TRUE'/'FALSE' depending on the source file."""
    return str(value).strip().lower() in ("true", "1")


def load_ground_truth(data_dir: str) -> pd.DataFrame:
    matches = glob.glob(os.path.join(data_dir, GROUND_TRUTH_GLOB))
    if not matches:
        raise FileNotFoundError(f"No ground truth file found matching {GROUND_TRUTH_GLOB!r} in {data_dir}")
    gt_path = matches[0]
    gt = pd.read_csv(gt_path)
    gt = gt.sort_values("phone_timestamp").reset_index(drop=True)
    return gt


def plot_matched_route(matched_path: str, gt: pd.DataFrame, output_dir: str) -> str:
    df = pd.read_csv(matched_path)
    df = df.sort_values("phone_timestamp").reset_index(drop=True)

    method = str(df["method"].iloc[0])
    kalman_on = to_bool(df["kalman"].iloc[0])
    kalman_label = "WITH Kalman Filtering" if kalman_on else "WITHOUT Kalman Filtering"

    fig, ax = plt.subplots(figsize=(10, 8))

    # Ground truth: line + dots
    ax.plot(gt["actual_x"], gt["actual_y"], "-", color="black", linewidth=1.5,
             alpha=0.6, label="Ground Truth (path)", zorder=1)
    ax.scatter(gt["actual_x"], gt["actual_y"], color="black", s=18,
                label="Ground Truth (points)", zorder=2)

    # Matched route: line + dots
    ax.plot(df["matched_x"], df["matched_y"], "-", color="tab:orange", linewidth=1.5,
             alpha=0.7, label=f"{method} Matched (path)", zorder=3)
    ax.scatter(df["matched_x"], df["matched_y"], color="tab:orange", s=18,
                label=f"{method} Matched (points)", zorder=4)

    ax.set_title(f"Matched Route vs Ground Truth\nMethod: {method} \u2014 {kalman_label}",
                 fontsize=13, fontweight="bold")
    ax.set_xlabel("X (m, local projection)")
    ax.set_ylabel("Y (m, local projection)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()

    base = os.path.splitext(os.path.basename(matched_path))[0]
    kalman_tag = "kalman" if kalman_on else "no_kalman"
    out_name = f"{base}__{method}_{kalman_tag}.png"
    out_path = os.path.join(output_dir, out_name)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_ground_truth_only(gt: pd.DataFrame, output_dir: str) -> str:
    fig, ax = plt.subplots(figsize=(10, 8))

    ax.plot(gt["actual_x"], gt["actual_y"], "-", color="black", linewidth=1.5,
             alpha=0.6, label="Ground Truth (path)", zorder=1)
    ax.scatter(gt["actual_x"], gt["actual_y"], color="black", s=18,
                label="Ground Truth (points)", zorder=2)

    ax.set_title("Ground Truth Route", fontsize=13, fontweight="bold")
    ax.set_xlabel("X (m, local projection)")
    ax.set_ylabel("Y (m, local projection)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()

    out_path = os.path.join(output_dir, "ground_truth_only.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    gt = load_ground_truth(DATA_DIR)

    matched_files = sorted(glob.glob(os.path.join(DATA_DIR, MATCHED_GLOB)))
    if not matched_files:
        raise FileNotFoundError(f"No matched files found matching {MATCHED_GLOB!r} in {DATA_DIR}")

    print(f"Found {len(matched_files)} matched route file(s). Generating plots...")
    for matched_path in matched_files:
        out_path = plot_matched_route(matched_path, gt, OUTPUT_DIR)
        print(f"  saved -> {out_path}")

    gt_out_path = plot_ground_truth_only(gt, OUTPUT_DIR)
    print(f"  saved -> {gt_out_path}")

    print(f"\nDone. Plots written to: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()