"""
positional_error.py
======================================================================
Positional error for SUMO map-matching output.

Straight-line ("as the crow flies") distance between each recorded GPS
fix and the point the matching pipeline snapped it to, summarised as
max / min / mean / standard deviation -- one row per input file, so the
five methods (and their Kalman-filtered counterparts) line up for easy
comparison.

No command-line arguments: edit the FILES dict below and re-run. The
script never talks to SUMO/traci -- it only reads the CSVs
post_processed_matching_v2.py already wrote, so nothing else needs to
be running.

WHICH DISTANCE THIS MEASURES
-----------------------------
"GPS point" here means the recorded phone fix BEFORE any Kalman
pre-filtering (gps_x/gps_y, or raw_x/raw_y if your CSV doesn't have a
gps_x/gps_y pass-through column) -- not the ground-truth simulated
position. That's a deliberate reading of "distance between the GPS
points and matched points": it's the error the matcher actually had to
correct for, phone noise included.

If your CSV also carries a ground-truth column pair (gt_x/gt_y, as the
synthetic runs in this project do), set REFERENCE_COLUMNS = ("gt_x",
"gt_y") instead to measure a different, arguably more interesting
number: how far the FINAL matched point ended up from where the bike
actually was, with phone noise factored out of the comparison entirely.
Either way the MATCHED_COLUMNS side stays ("matched_x", "matched_y").

Rows the matcher skipped (matched=False) or that are missing one of the
four coordinates are left out of the stats rather than counted as zero
error -- see load_errors().
----------------------------------------------------------------------
"""

import csv
import math
import os
import statistics

# ---------- EDIT THIS ----------
# label -> path. Add, remove, or rename entries freely; this is the only
# thing you should need to touch between runs. Paths are relative to
# this script's own directory unless you use an absolute path.
FILES = {
    "native":        "Data/Unfiltered Outputs/matched_native_20260905-195657.csv",
    "native_kalman": "Data/Filtered Outputs/matched_native_kalman_20260905-194946.csv",
    "topo":          "Data/Unfiltered Outputs/matched_topo_20260905-195705.csv",
    "topo_kalman":   "Data/Filtered Outputs/matched_topo_kalman_20260905-194415.csv",
    "fuzzy":         "Data/Unfiltered Outputs/matched_fuzzy_20260905-195714.csv",
    "fuzzy_kalman":  "Data/Filtered Outputs/matched_fuzzy_kalman_20260905-194958.csv",
    "hmm":           "Data/Unfiltered Outputs/matched_hmm_20260905-195724.csv",
    "hmm_kalman":    "Data/Filtered Outputs/matched_hmm_kalman_20260905-195319.csv",
    "st":            "Data/Unfiltered Outputs/matched_st_20260905-195733.csv",
    "st_kalman":     "Data/Filtered Outputs/matched_st_kalman_20260905-195604.csv",
}

# Which column pair counts as "the GPS point" for this measurement.
# ("gps_x", "gps_y") -- recorded phone fix (falls back to raw_x/raw_y
#                       below if your CSV doesn't have gps_x/gps_y)
# ("gt_x", "gt_y")   -- simulated ground truth, if your CSV has it
REFERENCE_COLUMNS = ("gps_x", "gps_y")
REFERENCE_FALLBACK_COLUMNS = ("raw_x", "raw_y")
MATCHED_COLUMNS = ("matched_x", "matched_y")

# Where to also write the summary table as a CSV. Set to None to skip.
SUMMARY_OUTPUT = "Data/positional_error_summary.csv"
# --------------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _resolve(path):
    return path if os.path.isabs(path) else os.path.join(SCRIPT_DIR, path)


def _to_float(value):
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _is_true(value):
    return str(value).strip().lower() in ("true", "1", "yes")


def load_errors(csv_path, ref_cols, ref_fallback_cols, matched_cols):
    """Return the list of straight-line distances (meters) between the
    reference column pair and the matched column pair, for every row
    where both are present and the row was matched. Falls back to
    ref_fallback_cols if the primary reference columns aren't in this
    CSV at all (e.g. no gps_x/gps_y pass-through column)."""
    ref_x_col, ref_y_col = ref_cols
    match_x_col, match_y_col = matched_cols

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []

        if ref_x_col not in fieldnames or ref_y_col not in fieldnames:
            if ref_fallback_cols and all(c in fieldnames for c in ref_fallback_cols):
                ref_x_col, ref_y_col = ref_fallback_cols
            else:
                raise ValueError(
                    f"{csv_path} has neither {ref_cols} nor {ref_fallback_cols} "
                    f"columns. Found columns: {fieldnames}"
                )
        missing = [c for c in (match_x_col, match_y_col) if c not in fieldnames]
        if missing:
            raise ValueError(f"{csv_path} is missing column(s) {missing}. "
                              f"Found columns: {fieldnames}")

        distances = []
        skipped_unmatched = 0
        skipped_missing = 0

        for row in reader:
            if "matched" in row and not _is_true(row["matched"]):
                skipped_unmatched += 1
                continue

            rx, ry = _to_float(row[ref_x_col]), _to_float(row[ref_y_col])
            mx, my = _to_float(row[match_x_col]), _to_float(row[match_y_col])
            if None in (rx, ry, mx, my):
                skipped_missing += 1
                continue

            distances.append(math.hypot(mx - rx, my - ry))

    return distances, skipped_unmatched, skipped_missing


def summarize(distances):
    if not distances:
        return None
    return {
        "n": len(distances),
        "max": max(distances),
        "min": min(distances),
        "mean": statistics.mean(distances),
        # Sample std (n-1); a single point has no spread to speak of.
        "std": statistics.stdev(distances) if len(distances) > 1 else 0.0,
    }


def main():
    results = {}

    for label, path in FILES.items():
        abs_path = _resolve(path)
        if not os.path.exists(abs_path):
            print(f"[WARN] {label}: file not found at {abs_path} -- skipping.")
            continue

        try:
            distances, skipped_unmatched, skipped_missing = load_errors(
                abs_path, REFERENCE_COLUMNS, REFERENCE_FALLBACK_COLUMNS, MATCHED_COLUMNS
            )
        except ValueError as e:
            print(f"[WARN] {label}: {e}")
            continue

        stats = summarize(distances)
        results[label] = stats

        print(f"\n=== {label} ({path}) ===")
        if stats is None:
            print("  No usable points (nothing matched, or the reference/"
                  "matched columns were blank for every row).")
            continue
        print(f"  n matched & usable : {stats['n']}")
        if skipped_unmatched:
            print(f"  skipped (unmatched): {skipped_unmatched}")
        if skipped_missing:
            print(f"  skipped (missing)  : {skipped_missing}")
        print(f"  max  : {stats['max']:.3f} m")
        print(f"  min  : {stats['min']:.3f} m")
        print(f"  mean : {stats['mean']:.3f} m")
        print(f"  std  : {stats['std']:.3f} m")

    if not results:
        print("\n[WARN] No files produced usable results.")
        return

    print("\n" + "=" * 70)
    print(f"{'method':<16}{'n':>6}{'max (m)':>12}{'min (m)':>12}"
          f"{'mean (m)':>12}{'std (m)':>12}")
    print("-" * 70)
    for label, stats in results.items():
        if stats is None:
            print(f"{label:<16}{'--':>6}{'--':>12}{'--':>12}{'--':>12}{'--':>12}")
        else:
            print(f"{label:<16}{stats['n']:>6}{stats['max']:>12.3f}"
                  f"{stats['min']:>12.3f}{stats['mean']:>12.3f}{stats['std']:>12.3f}")
    print("=" * 70)

    if SUMMARY_OUTPUT:
        out_path = _resolve(SUMMARY_OUTPUT)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["method", "n", "max_m", "min_m", "mean_m", "std_m"])
            for label, stats in results.items():
                if stats is None:
                    writer.writerow([label, 0, "", "", "", ""])
                else:
                    writer.writerow([label, stats["n"], stats["max"], stats["min"],
                                      stats["mean"], stats["std"]])
        print(f"\n[INFO] Summary written to {out_path}")


if __name__ == "__main__":
    main()
