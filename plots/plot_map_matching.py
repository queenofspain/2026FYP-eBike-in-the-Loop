#!/usr/bin/env python3
"""
Map-matching comparison plots
=============================

Generates, for EACH input CSV (one CSV = one run), 17 figures:

    01_raw                                  raw GPS only                    (1)
    02_all_methods                          raw GPS + all 10 combinations   (1)
    03_<method>_<kalman|nokalman>           raw GPS + one combination      (10)
    04_<method>_kalman_comparison           raw GPS + method with/without
                                            Kalman filtering                (5)

Usage
-----
    python plot_map_matching.py                      # uses DATA_FILES below
    python plot_map_matching.py a.csv b.csv --out figs

EVERYTHING you are likely to want to change lives in the CONFIG section
directly below (titles, axis labels, legend entries, colours, line styles,
line widths, markers, legend placement, figure size, file formats ...).
Nothing below the "END OF CONFIG" marker needs to be edited.
"""

import argparse
import glob
import os

import matplotlib

matplotlib.use("Agg")  # no display needed; set SHOW = True below to open windows
import matplotlib.pyplot as plt
import pandas as pd

# =============================================================================
# CONFIG  -  edit freely
# =============================================================================

# ---- input / output ---------------------------------------------------------
DATA_FILES = sorted(glob.glob("/mnt/user-data/uploads/all_methods_*.csv"))
OUT_DIR = "figures"            # one sub-folder per run is created inside
SAVE_FORMATS = ["png"]         # e.g. ["png", "pdf", "svg"]
DPI = 200
SHOW = False                   # True = also pop up each figure (needs a display)

# ---- run names (used in titles and for the sub-folder names) -----------------
# key = CSV file name without ".csv"; any file not listed falls back to its stem.
RUN_NAMES = {
    "all_methods_20261005-152039_jordan1": "Run 1",
    "all_methods_20261005-152040":         "Run 2",
    "all_methods_20261005-155952_jordan2": "Run 3",
    "all_methods_20261005-155953":         "Run 4",
}

# ---- titles -----------------------------------------------------------------
# Placeholders:  {run}     run name from RUN_NAMES
#                {series}  legend label of the plotted series ("single" only)
#                {method}  display name of the method ("kalman" only)
TITLES = {
    "raw":    "Raw GPS track ({run})",
    "all":    "Raw GPS and all map-matching methods ({run})",
    "single": "Raw GPS and {series} ({run})",
    "kalman": "{method}: with vs without Kalman filtering ({run})",
}
# Exceptions for one specific figure: key = file stem (e.g. "03_hmm_kalman",
# "04_st_kalman_comparison", "01_raw").  {run} is still substituted.
TITLE_OVERRIDES = {
    # "03_hmm_kalman": "My custom HMM title ({run})",
}

# ---- axes -------------------------------------------------------------------
XLABEL = "x (m)"
YLABEL = "y (m)"
EQUAL_ASPECT = True            # x and y are metres, so keep 1 m = 1 m on both axes
GRID = True
FIGSIZE = (9, 8)
TITLE_FONTSIZE = 13
LABEL_FONTSIZE = 11
TICK_FONTSIZE = 9

# ---- legend (any matplotlib Axes.legend keyword can go in here) ---------------
LEGEND = dict(
    loc="upper left",
    bbox_to_anchor=(1.02, 1.0),   # outside the plot, on the right
    frameon=False,
    fontsize=10,
    ncol=1,
    title=None,                    # e.g. "Method"
)
LEGEND_LINE_WIDTH = 2.5            # thickness of the lines drawn in the legend
                                   # (None = same as plotted lines)

# ---- the methods (display order = plotting order = legend order) -------------
# key (must match the 'method' column) -> legend name and colour
METHODS = {
    "native": dict(name="SUMO native",    color="#2a78d6"),   # blue
    "topo":   dict(name="Topological",    color="#eb6834"),   # orange
    "fuzzy":  dict(name="Fuzzy logic",    color="#1baf7a"),   # aqua
    "hmm":    dict(name="HMM",            color="#eda100"),   # yellow
    "st":     dict(name="ST-matching",    color="#e87ba4"),   # magenta
}

# Added to the method name to build legend entries for each Kalman setting
KALMAN_SUFFIX = {False: "", True: " + Kalman"}

# Default style of every method line, split by Kalman on/off
STYLE_BY_KALMAN = {
    False: dict(linestyle="-",  linewidth=1.4, marker=None, markersize=3, alpha=1.0),
    True:  dict(linestyle="--", linewidth=1.4, marker=None, markersize=3, alpha=1.0),
}

# Style of the raw GPS series (legend entry, colour, line style, markers ...)
RAW_STYLE = dict(
    label="Raw GPS",
    color="#898781",
    linestyle="-",
    linewidth=0.8,
    marker="o",
    markersize=2.5,
    alpha=0.8,
)

# Per-series overrides: anything here beats the defaults above.
# Keys are "raw" or (method, kalman_bool).  Any Line2D keyword is allowed
# (label, color, linestyle, linewidth, marker, markersize, alpha, zorder ...).
# Examples (uncomment / copy):
SERIES_OVERRIDES = {
    # "raw":            dict(color="black", marker=None),
    # ("hmm", True):    dict(label="HMM (Kalman filtered)", color="#4a3aa7", linestyle=":"),
    # ("st", False):    dict(linewidth=2.5),
}

# If a method failed to match some fixes (e.g. fuzzy), the matched coordinates are
# NaN there.  False = leave a visible gap in the line; True = join across the gap.
CONNECT_GAPS = False

# ---- plot chrome ------------------------------------------------------------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID_COLOR = "#e1e0d9"
GRID_LINEWIDTH = 0.6

# =============================================================================
# END OF CONFIG
# =============================================================================


def build_series():
    """Merge defaults + overrides into one style dict per series key."""
    series = {"raw": dict(RAW_STYLE, zorder=1)}
    for m, info in METHODS.items():
        for kal in (False, True):
            style = dict(STYLE_BY_KALMAN[kal])
            style["color"] = info["color"]
            style["label"] = info["name"] + KALMAN_SUFFIX[kal]
            style["zorder"] = 3 if kal else 2
            series[(m, kal)] = style
    for key, upd in SERIES_OVERRIDES.items():
        series[key].update(upd)
    return series


SERIES = build_series()


def load_run(path):
    """Return (raw_df, {(method, kalman): df}) with rows sorted by fix_seq."""
    df = pd.read_csv(path)
    df["kalman"] = df["kalman"].astype(str).str.lower().eq("true")
    groups = {}
    for (m, k), g in df.groupby(["method", "kalman"]):
        groups[(m, bool(k))] = g.sort_values("fix_seq").reset_index(drop=True)
    # raw GPS is identical for every method / Kalman setting, so take one copy
    ref = next(iter(groups.values()))
    raw = ref[["fix_seq", "raw_x", "raw_y"]].rename(columns={"raw_x": "x", "raw_y": "y"})
    return raw, groups


def xy_for(key, raw, groups):
    if key == "raw":
        d = raw
    else:
        d = groups[key].rename(columns={"matched_x": "x", "matched_y": "y"})
    d = d[["x", "y"]]
    return d.dropna() if CONNECT_GAPS else d


def make_figure(keys, title, raw, groups):
    fig, ax = plt.subplots(figsize=FIGSIZE, facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    for key in keys:
        d = xy_for(key, raw, groups)
        ax.plot(d["x"], d["y"], **SERIES[key])

    ax.set_title(title, fontsize=TITLE_FONTSIZE, color=INK, loc="left")
    ax.set_xlabel(XLABEL, fontsize=LABEL_FONTSIZE, color=INK_SECONDARY)
    ax.set_ylabel(YLABEL, fontsize=LABEL_FONTSIZE, color=INK_SECONDARY)
    ax.tick_params(labelsize=TICK_FONTSIZE, colors=INK_SECONDARY)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID_COLOR)
    if GRID:
        ax.grid(True, color=GRID_COLOR, linewidth=GRID_LINEWIDTH)
        ax.set_axisbelow(True)
    if EQUAL_ASPECT:
        ax.set_aspect("equal", adjustable="datalim")

    leg = ax.legend(**LEGEND)
    if leg is not None:
        for t in leg.get_texts():
            t.set_color(INK)
        if LEGEND_LINE_WIDTH is not None:
            for ln in leg.get_lines():
                ln.set_linewidth(LEGEND_LINE_WIDTH)
    fig.tight_layout()
    return fig


def save(fig, folder, stem):
    for fmt in SAVE_FORMATS:
        fig.savefig(os.path.join(folder, f"{stem}.{fmt}"), dpi=DPI,
                    bbox_inches="tight", facecolor=fig.get_facecolor())
    if SHOW:
        plt.show()
    plt.close(fig)


def title_for(stem, kind, run, **kw):
    tpl = TITLE_OVERRIDES.get(stem, TITLES[kind])
    return tpl.format(run=run, **kw)


def plot_run(path, out_root):
    stem_name = os.path.splitext(os.path.basename(path))[0]
    run = RUN_NAMES.get(stem_name, stem_name)
    folder = os.path.join(out_root, run.replace(" ", "_"))
    os.makedirs(folder, exist_ok=True)
    raw, groups = load_run(path)
    n = 0

    # 1) raw GPS alone
    s = "01_raw"
    save(make_figure(["raw"], title_for(s, "raw", run), raw, groups), folder, s); n += 1

    # 2) raw GPS + all methods
    s = "02_all_methods"
    keys = ["raw"] + [(m, k) for m in METHODS for k in (False, True)]
    save(make_figure(keys, title_for(s, "all", run), raw, groups), folder, s); n += 1

    # 3) raw GPS + each individual method/Kalman combination (10)
    for m in METHODS:
        for k in (False, True):
            s = f"03_{m}_{'kalman' if k else 'nokalman'}"
            t = title_for(s, "single", run, series=SERIES[(m, k)]["label"])
            save(make_figure(["raw", (m, k)], t, raw, groups), folder, s); n += 1

    # 4) raw GPS + method with and without Kalman filtering (5)
    for m, info in METHODS.items():
        s = f"04_{m}_kalman_comparison"
        t = title_for(s, "kalman", run, method=info["name"])
        save(make_figure(["raw", (m, False), (m, True)], t, raw, groups), folder, s); n += 1

    print(f"{run}: {n} figures -> {folder}/")
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="*", default=DATA_FILES, help="input CSV file(s)")
    ap.add_argument("--out", default=OUT_DIR, help="output folder")
    args = ap.parse_args()
    if not args.csv:
        ap.error("no CSV files found - pass them on the command line or set DATA_FILES")
    total = sum(plot_run(p, args.out) for p in args.csv)
    print(f"Done: {total} figures.")


if __name__ == "__main__":
    main()
