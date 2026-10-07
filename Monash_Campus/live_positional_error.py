"""
positional_error.py
======================================================================
Positional error for SUMO map-matching output.

Straight-line ("as the crow flies") distance between each recorded GPS
fix and the point the matching pipeline snapped it to, summarised as
max / min / mean / standard deviation.

INPUT FORMAT
------------
Each input file is ONE RUN and contains every method, interleaved
row-wise (fix 1 for all methods, then fix 2 for all methods, etc.).
Rows are told apart by two columns:

    method  -- native / topo / fuzzy / hmm / st
    kalman  -- TRUE/FALSE (any capitalisation) -- was the Kalman
               pre-filter on for that row

so each file yields 10 groups, labelled "native", "native_kalman",
"topo", "topo_kalman", ... as before.

WHAT THE SCRIPT DOES
--------------------
1. For every run file, computes the positional error stats
   (n, max, min, mean, std) separately for each of the 10 groups.
2. Averages those per-run stats across all runs, for each group, and
   prints/writes one row per method.

(The average is a mean of the per-run numbers, e.g. "average of the
per-run means". Every run counts equally, whatever its length.)

No command-line arguments: edit RUN_FILES below and re-run. The script
never talks to SUMO/traci -- it only reads the CSVs.

WHICH DISTANCE THIS MEASURES
-----------------------------
"GPS point" here means the recorded phone fix BEFORE any Kalman
pre-filtering (raw_x/raw_y) -- not the Kalman-filtered position
(filt_x/filt_y). That's a deliberate reading of "distance between the
GPS points and matched points": it's the error the matcher actually had
to correct for, phone noise included. It is the same quantity as the
match_error_raw_m column already in the files.

To measure from the Kalman-filtered point instead, set
REFERENCE_COLUMNS = ("filt_x", "filt_y"). The matched side stays
("matched_x", "matched_y").

Rows the matcher skipped (matched=False) or that are missing one of the
four coordinates are left out of the stats rather than counted as zero
error -- see load_run().
----------------------------------------------------------------------
"""

import csv
import math
import os
import statistics

# ---------- EDIT THIS ----------
# One path per run. Paths are relative to this script's own directory
# unless you use an absolute path. Add or remove runs freely.
RUN_FILES = [
    "C:/Uni/Final Year Project/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-152039.csv",
    "C:/Uni/Final Year Project/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-152040.csv",
    "C:/Uni/Final Year Project/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-155952.csv",
    "C:/Uni/Final Year Project/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-155953.csv",
]

# Which column pair counts as "the GPS point" for this measurement.
# ("raw_x", "raw_y")   -- recorded phone fix, before Kalman filtering
# ("filt_x", "filt_y") -- Kalman-filtered fix
REFERENCE_COLUMNS = ("raw_x", "raw_y")
MATCHED_COLUMNS = ("matched_x", "matched_y")

# Where to write the CSVs. Set either to None to skip it.
SUMMARY_OUTPUT = "Data/positional_error_summary.csv"      # averaged over runs
PER_RUN_OUTPUT = "Data/positional_error_per_run.csv"      # one row per run x method
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


def _run_name(path):
    """'all_methods_20261005-152039.csv' -> '20261005-152039'."""
    stem = os.path.splitext(os.path.basename(path))[0]
    return stem.replace("all_methods_", "", 1)


def _group_label(method, kalman):
    return f"{method}_kalman" if _is_true(kalman) else method


def load_run(csv_path, ref_cols, matched_cols):
    """Read one run file and return {label: info}, where label is e.g.
    "native" or "native_kalman" (in order of first appearance) and info is
    {"distances": [...], "skipped_unmatched": int, "skipped_missing": int}.

    distances are the straight-line distances (meters) between the
    reference column pair and the matched column pair, for every row of
    that method/kalman group where both are present and the row was
    matched."""
    ref_x_col, ref_y_col = ref_cols
    match_x_col, match_y_col = matched_cols

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []

        required = ["method", "kalman", ref_x_col, ref_y_col,
                    match_x_col, match_y_col]
        missing = [c for c in required if c not in fieldnames]
        if missing:
            raise ValueError(f"{csv_path} is missing column(s) {missing}. "
                             f"Found columns: {fieldnames}")

        groups = {}
        for row in reader:
            label = _group_label(row["method"], row["kalman"])
            g = groups.setdefault(label, {"distances": [],
                                          "skipped_unmatched": 0,
                                          "skipped_missing": 0})

            if "matched" in row and not _is_true(row["matched"]):
                g["skipped_unmatched"] += 1
                continue

            rx, ry = _to_float(row[ref_x_col]), _to_float(row[ref_y_col])
            mx, my = _to_float(row[match_x_col]), _to_float(row[match_y_col])
            if None in (rx, ry, mx, my):
                g["skipped_missing"] += 1
                continue

            g["distances"].append(math.hypot(mx - rx, my - ry))

    return groups


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


def average_over_runs(per_run_stats):
    """per_run_stats: list of stats dicts (one per run, None entries
    ignored). Returns the average of each stat across runs, plus the
    number of runs that contributed."""
    usable = [s for s in per_run_stats if s is not None]
    if not usable:
        return None
    return {
        "runs": len(usable),
        "n": statistics.mean(s["n"] for s in usable),
        "max": statistics.mean(s["max"] for s in usable),
        "min": statistics.mean(s["min"] for s in usable),
        "mean": statistics.mean(s["mean"] for s in usable),
        "std": statistics.mean(s["std"] for s in usable),
    }


def _print_table(title, rows, first_col, show_runs=False):
    """rows: list of (label, stats-or-None)."""
    width = 82 if show_runs else 70
    print("\n" + "=" * width)
    print(title)
    head = f"{first_col:<16}"
    if show_runs:
        head += f"{'runs':>6}"
    head += f"{'n':>8}{'max (m)':>12}{'min (m)':>12}{'mean (m)':>12}{'std (m)':>12}"
    print(head)
    print("-" * width)
    for label, s in rows:
        line = f"{label:<16}"
        if s is None:
            if show_runs:
                line += f"{'--':>6}"
            line += f"{'--':>8}{'--':>12}{'--':>12}{'--':>12}{'--':>12}"
        else:
            if show_runs:
                line += f"{s['runs']:>6}"
            n = s["n"]
            n_txt = f"{n:.1f}" if show_runs else f"{n}"
            line += (f"{n_txt:>8}{s['max']:>12.3f}{s['min']:>12.3f}"
                     f"{s['mean']:>12.3f}{s['std']:>12.3f}")
        print(line)
    print("=" * width)


def main():
    # label -> list of per-run stats, in first-seen order
    by_label = {}
    # (run_name, label, stats) for the per-run output
    per_run_rows = []

    for path in RUN_FILES:
        abs_path = _resolve(path)
        run = _run_name(path)
        if not os.path.exists(abs_path):
            print(f"[WARN] run {run}: file not found at {abs_path} -- skipping.")
            continue

        try:
            groups = load_run(abs_path, REFERENCE_COLUMNS, MATCHED_COLUMNS)
        except ValueError as e:
            print(f"[WARN] run {run}: {e}")
            continue

        table_rows = []
        for label, info in groups.items():
            stats = summarize(info["distances"])
            by_label.setdefault(label, []).append(stats)
            per_run_rows.append((run, label, stats))
            table_rows.append((label, stats))

        skipped = sum(g["skipped_unmatched"] for g in groups.values())
        title = f"Run {run}  ({path})"
        if skipped:
            title += f"  -- {skipped} unmatched row(s) excluded"
        _print_table(title, table_rows, "method")

    if not by_label:
        print("\n[WARN] No files produced usable results.")
        return

    averaged = {label: average_over_runs(stats_list)
                for label, stats_list in by_label.items()}

    n_runs = len({r for r, _, _ in per_run_rows})
    _print_table(f"AVERAGE ACROSS {n_runs} RUN(S)  (each stat averaged over runs)",
                 list(averaged.items()), "method", show_runs=True)

    if PER_RUN_OUTPUT:
        out_path = _resolve(PER_RUN_OUTPUT)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["run", "method", "n", "max_m", "min_m", "mean_m", "std_m"])
            for run, label, s in per_run_rows:
                if s is None:
                    writer.writerow([run, label, 0, "", "", "", ""])
                else:
                    writer.writerow([run, label, s["n"], s["max"], s["min"],
                                     s["mean"], s["std"]])
        print(f"\n[INFO] Per-run results written to {out_path}")

    if SUMMARY_OUTPUT:
        out_path = _resolve(SUMMARY_OUTPUT)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["method", "runs", "avg_n", "avg_max_m", "avg_min_m",
                             "avg_mean_m", "avg_std_m"])
            for label, s in averaged.items():
                if s is None:
                    writer.writerow([label, 0, "", "", "", "", ""])
                else:
                    writer.writerow([label, s["runs"], s["n"], s["max"], s["min"],
                                     s["mean"], s["std"]])
        print(f"[INFO] Averaged summary written to {out_path}")


if __name__ == "__main__":
    main()