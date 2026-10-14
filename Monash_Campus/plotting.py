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

and, when the SUMO network file is available (see NET_FILE), 17 more per run
that compare the matched tracks with the SUMO route:

    05_route_raw                            SUMO route + raw GPS            (1)
    06_route_all_methods                    route + all 10 combinations     (1)
    07_route_<method>_<kalman|nokalman>     route + one combination        (10)
    08_route_<method>_kalman_comparison     route + method with/without
                                            Kalman filtering                (5)

plus, once for all runs together (folder All_runs), one figure per method:

    09_<method>_all_runs                    SUMO route + all runs of that
                                            method; left panel without,
                                            right panel with Kalman         (5)

The SUMO route geometry is read from the network (.net.xml): the .rou.xml
only lists edge IDs.  Without the network file the route figures (05-08) are
skipped and the 09 figures are drawn without the route layer.

Usage
-----
    python plot_map_matching.py                      # uses DATA_FILES below
    python plot_map_matching.py a.csv b.csv --out figs
    python plot_map_matching.py --net my_network.net.xml --route my_route.rou.xml

EVERYTHING you are likely to want to change lives in the CONFIG section
directly below (titles, axis labels, legend entries, colours, line styles,
line widths, markers, legend placement, figure size, file formats ...).
Nothing below the "END OF CONFIG" marker needs to be edited.
"""

import argparse
import glob
import gzip
import os
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")  # no display needed; set SHOW = True below to open windows
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ---- path helpers -------------------------------------------------------------
# Relative paths in the CONFIG are looked up from the folder you run the script in,
# then from the folder this script lives in, then that folder's parent - so it works
# whether you start it from the project root or from inside Monash_Campus.
# A leading "/" or "\" is treated as relative (on Windows it would mean the drive root).
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SEARCH_DIRS = [os.getcwd(), _SCRIPT_DIR, os.path.dirname(_SCRIPT_DIR)]


def _candidates(path):
    yield path
    rel = path.lstrip("/\\")
    for base in _SEARCH_DIRS:
        yield os.path.join(base, rel)


def resolve(path):
    """First existing location of `path`, or `path` unchanged if none exists."""
    if path:
        for cand in _candidates(path):
            if os.path.exists(cand):
                return cand
    return path


def find_files(pattern):
    """Sorted list of files matching `pattern` (same search rules as resolve)."""
    for cand in _candidates(pattern):
        hits = sorted(glob.glob(cand))
        if hits:
            return hits
    return []


# =============================================================================
# CONFIG  -  edit freely
# =============================================================================

# ---- input / output ---------------------------------------------------------
DATA_PATTERN = "/Data/all_methods_*.csv"   # folder + file pattern of the run CSVs (wildcards OK)
DATA_FILES = find_files(DATA_PATTERN)      # -> sorted list of the matching files
OUT_DIR = "figures"            # one sub-folder per run is created inside
SAVE_FORMATS = ["png"]         # e.g. ["png", "pdf", "svg"]
DPI = 200
SHOW = False                   # True = also pop up each figure (needs a display)

# ---- run names (used in titles and for the sub-folder names) -----------------
# key = CSV file name without ".csv"; any file not listed falls back to its stem.
RUN_NAMES = {
    "all_methods_20261005-152039":          "Run 1",
    "all_methods_20261005-152040":          "Run 2",
    "all_methods_20261005-155952":          "Run 3",
    "all_methods_20261005-155953":          "Run 4",
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
    # --- SUMO-route figures ---
    "route_raw":    "SUMO route and raw GPS track ({run})",
    "route_all":    "SUMO route and all map-matching methods ({run})",
    "route_single": "SUMO route and {series} ({run})",
    "route_kalman": "SUMO route and {method}: with vs without Kalman filtering ({run})",
    # --- one figure per method, all runs ({method} only) ---
    "all_runs":     "{method}: all runs against the SUMO route",
    "all_runs_noroute": "{method}: all runs",   # used when no network file is available
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

# ---- SUMO route (reference layer) --------------------------------------------
# The .rou.xml only lists edge IDs; the geometry comes from the SUMO network
# (.net.xml, or .net.xml.gz) that the matching was run on.  If NET_FILE does not
# exist the route figures (05-08) are skipped and 09 is drawn without the route.
ROUTE_FILE = "Monash_Campus/2026-08-25-19-43-30/full_campus_v3.rou.xml"
NET_FILE = "Monash_Campus/2026-08-25-19-43-30/osm.net.xml"   # <- SUMO network file (or use --net)
ROUTE_LANE_INDEX = None      # None = first lane that allows bicycles; or force e.g. 0
ROUTE_GAP_BREAK_M = 60.0     # consecutive edges normally end a few metres apart (the
                             # junction); these are joined with a straight line.  Only if
                             # they are further apart than this is a gap left in the route.
INCLUDE_RAW_IN_ROUTE_PLOTS = False   # True = also draw raw GPS in figures 06-08

# Style of the route line (any Line2D keyword).  A wide, faint line sits under the
# matched tracks so they stay readable on top of it.
ROUTE_STYLE = dict(
    label="SUMO route",
    color="#0b0b0b",
    linestyle="-",
    linewidth=5.0,
    alpha=0.22,
)

# ---- one figure per method with all runs (figure 09) ---------------------------
ALL_RUNS_FOLDER = "All_runs"
RUN_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7"]   # one per CSV, in DATA_FILES order
RUN_STYLE_DEFAULT = dict(linestyle="-", linewidth=1.3, marker=None, markersize=3, alpha=0.9)
# Per-run overrides, key = run name from RUN_NAMES; any Line2D keyword incl. label/color.
RUN_STYLE_OVERRIDES = {
    # "Run 2": dict(linestyle="--", label="Run 2 (rain)"),
}
# Panel titles: left panel = no Kalman filter, right panel = Kalman filter
MULTIRUN_PANEL_TITLES = {False: "Without Kalman filtering", True: "With Kalman filtering"}
MULTIRUN_FIGSIZE = (16, 8.5)
MULTIRUN_LEGEND = dict(loc="lower center", bbox_to_anchor=(0.5, 0.0), ncol=5,
                       frameon=False, fontsize=10)

# =============================================================================
# END OF CONFIG
# =============================================================================


def build_series():
    """Merge defaults + overrides into one style dict per series key."""
    series = {"raw": dict(RAW_STYLE, zorder=1),
              "route": dict(ROUTE_STYLE, zorder=0.5)}
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


def run_name(stem):
    """Display name for a CSV: exact match in RUN_NAMES, else the first key the file
    name starts with (so a suffix such as '_jordan1' on the file name doesn't matter)."""
    if stem in RUN_NAMES:
        return RUN_NAMES[stem]
    for key, name in RUN_NAMES.items():
        if stem.startswith(key):
            return name
    return stem


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


def xy_for(key, raw, groups, route=None):
    if key == "route":
        return pd.DataFrame(route, columns=["x", "y"])   # keeps NaN breaks
    if key == "raw":
        d = raw
    else:
        d = groups[key].rename(columns={"matched_x": "x", "matched_y": "y"})
    d = d[["x", "y"]]
    return d.dropna() if CONNECT_GAPS else d


def style_axes(ax, title, adjustable="datalim"):
    """Titles, labels, spines, grid and aspect shared by every figure."""
    ax.set_facecolor(SURFACE)
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
        ax.set_aspect("equal", adjustable=adjustable)


# ----------------------------------------------------------------------------
# SUMO route: edge IDs from the .rou.xml, geometry from the .net.xml
# ----------------------------------------------------------------------------
def read_route_edges(route_file):
    for el in ET.parse(route_file).getroot().iter("route"):
        if el.get("edges"):
            return el.get("edges").split()
    raise ValueError(f"no <route edges=...> found in {route_file}")


def _lane_allows_bicycle(lane):
    allow, disallow = lane.get("allow"), lane.get("disallow")
    if allow is not None:
        return "bicycle" in allow.split() or "all" in allow.split()
    if disallow is not None:
        return "bicycle" not in disallow.split()
    return True


def _pick_lane(lanes):
    lanes = sorted(lanes, key=lambda l: int(l.get("index", 0)))
    if ROUTE_LANE_INDEX is not None:
        for l in lanes:
            if int(l.get("index", 0)) == ROUTE_LANE_INDEX:
                return l
    for l in lanes:
        if _lane_allows_bicycle(l):
            return l
    return lanes[0] if lanes else None


def read_edge_shapes(net_file, wanted):
    """{edge_id: (N,2) array} of the chosen lane's centre line, net coordinates."""
    opener = gzip.open if net_file.endswith(".gz") else open
    shapes = {}
    with opener(net_file, "rb") as fh:
        for _, el in ET.iterparse(fh, events=("end",)):
            if el.tag != "edge":
                continue
            eid = el.get("id")
            if eid in wanted and el.get("function") != "internal":
                lane = _pick_lane(el.findall("lane"))
                if lane is not None and lane.get("shape"):
                    shapes[eid] = np.array([[float(v) for v in p.split(",")[:2]]
                                            for p in lane.get("shape").split()])
            el.clear()
    return shapes


def build_route_polylines(edges, shapes, gap_m):
    """Concatenate edge shapes in route order; start a new stretch at missing edges
    or where consecutive edges are not connected (> gap_m apart)."""
    parts, cur, missing = [], [], []
    for e in edges:
        s = shapes.get(e)
        if s is None:
            missing.append(e)
            if cur:
                parts.append(np.vstack(cur)); cur = []
            continue
        if cur and np.hypot(*(s[0] - cur[-1][-1])) > gap_m:
            parts.append(np.vstack(cur)); cur = []
        cur.append(s)
    if cur:
        parts.append(np.vstack(cur))
    return parts, missing


def load_route(route_file, net_file):
    """(N,2) array of the route with NaN rows between stretches, or None."""
    if not (route_file and os.path.exists(route_file)):
        print(f"[route] route file not found: {route_file} -> route figures skipped")
        return None
    if not (net_file and os.path.exists(net_file)):
        print(f"[route] network file not found: {net_file}\n"
              "        The .rou.xml only lists edge IDs; set NET_FILE (or --net) to the SUMO\n"
              "        .net.xml to draw the route.  Route figures (05-08) skipped.")
        return None
    edges = read_route_edges(route_file)
    shapes = read_edge_shapes(net_file, set(edges))
    parts, missing = build_route_polylines(edges, shapes, ROUTE_GAP_BREAK_M)
    if not parts:
        print("[route] none of the route's edges were found in the network file "
              "-> route figures skipped")
        return None
    pieces = []
    for p in parts:
        pieces += [p, np.full((1, 2), np.nan)]
    print(f"[route] {len(edges) - len(missing)}/{len(edges)} edges found in the network, "
          f"{len(parts)} continuous stretch(es)")
    if missing:
        print("[route] edges missing from the network (route drawn with a gap there): "
              + ", ".join(missing[:10]) + (" ..." if len(missing) > 10 else ""))
    return np.vstack(pieces[:-1])


def check_route_alignment(route, raw):
    """Sanity check: how far is the raw GPS from the route?  Large values usually mean
    the network file does not belong to these CSVs (different net / offset)."""
    dense = []
    for a, b in zip(route[:-1], route[1:]):
        if np.isnan(a).any() or np.isnan(b).any():
            continue
        n = max(int(np.hypot(*(b - a))), 1)
        dense.append(a + (b - a) * (np.arange(n)[:, None] / n))
    if not dense:
        return
    dense = np.vstack(dense)
    pts = raw[["x", "y"]].to_numpy()
    dist = np.array([np.hypot(*(dense - p).T).min() for p in pts])
    print(f"[route] raw GPS to route distance: median {np.median(dist):.1f} m, "
          f"95th percentile {np.percentile(dist, 95):.1f} m")
    if np.median(dist) > 30:
        print("[route] WARNING: the route is far from the GPS track - check that the "
              "network file matches the one used for these runs.")


def make_figure(keys, title, raw, groups, route=None):
    fig, ax = plt.subplots(figsize=FIGSIZE, facecolor=SURFACE)
    for key in keys:
        d = xy_for(key, raw, groups, route)
        ax.plot(d["x"], d["y"], **SERIES[key])
    style_axes(ax, title)

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


def plot_run(path, out_root, route=None):
    stem_name = os.path.splitext(os.path.basename(path))[0]
    run = run_name(stem_name)
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

    # ---- SUMO route figures (need the network file) --------------------------
    if route is not None:
        base = ["route"] + (["raw"] if INCLUDE_RAW_IN_ROUTE_PLOTS else [])

        # 5) route + raw GPS
        s = "05_route_raw"
        save(make_figure(["route", "raw"], title_for(s, "route_raw", run),
                         raw, groups, route), folder, s); n += 1

        # 6) route + all methods
        s = "06_route_all_methods"
        keys = base + [(m, k) for m in METHODS for k in (False, True)]
        save(make_figure(keys, title_for(s, "route_all", run),
                         raw, groups, route), folder, s); n += 1

        # 7) route + each individual method/Kalman combination (10)
        for m in METHODS:
            for k in (False, True):
                s = f"07_route_{m}_{'kalman' if k else 'nokalman'}"
                t = title_for(s, "route_single", run, series=SERIES[(m, k)]["label"])
                save(make_figure(base + [(m, k)], t, raw, groups, route),
                     folder, s); n += 1

        # 8) route + method with and without Kalman filtering (5)
        for m, info in METHODS.items():
            s = f"08_route_{m}_kalman_comparison"
            t = title_for(s, "route_kalman", run, method=info["name"])
            save(make_figure(base + [(m, False), (m, True)], t, raw, groups, route),
                 folder, s); n += 1

    print(f"{run}: {n} figures -> {folder}/")
    return n


def plot_all_runs(paths, out_root, route=None):
    """One figure per method: SUMO route + every run, without (left) and with (right)
    Kalman filtering."""
    folder = os.path.join(out_root, ALL_RUNS_FOLDER)
    os.makedirs(folder, exist_ok=True)
    runs = []
    for i, p in enumerate(paths):
        name = os.path.splitext(os.path.basename(p))[0]
        run = run_name(name)
        style = dict(RUN_STYLE_DEFAULT, color=RUN_COLORS[i % len(RUN_COLORS)],
                     label=run, zorder=2)
        style.update(RUN_STYLE_OVERRIDES.get(run, {}))
        runs.append((load_run(p)[1], style))

    n = 0
    for m, info in METHODS.items():
        s = f"09_{m}_all_runs"
        fig, axes = plt.subplots(1, 2, figsize=MULTIRUN_FIGSIZE, sharex=True, sharey=True,
                                 facecolor=SURFACE)
        for ax, k in zip(axes, (False, True)):
            if route is not None:
                ax.plot(route[:, 0], route[:, 1], **SERIES["route"])
            for groups, style in runs:
                d = xy_for((m, k), None, groups)
                ax.plot(d["x"], d["y"], **style)
            style_axes(ax, MULTIRUN_PANEL_TITLES[k], adjustable="box")
        axes[1].set_ylabel("")
        axes[1].tick_params(labelleft=False)

        handles, labels = axes[0].get_legend_handles_labels()
        leg = fig.legend(handles, labels, **MULTIRUN_LEGEND)
        for t in leg.get_texts():
            t.set_color(INK)
        if LEGEND_LINE_WIDTH is not None:
            for ln in leg.get_lines():
                ln.set_linewidth(LEGEND_LINE_WIDTH)
        kind = "all_runs" if route is not None else "all_runs_noroute"
        fig.suptitle(title_for(s, kind, "", method=info["name"]), x=0.01, ha="left",
                     fontsize=TITLE_FONTSIZE, color=INK)
        fig.tight_layout(rect=(0, 0.07, 1, 0.95))
        save(fig, folder, s); n += 1

    print(f"All runs: {n} figures -> {folder}/")
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="*", default=DATA_FILES, help="input CSV file(s) or pattern(s)")
    ap.add_argument("--out", default=OUT_DIR, help="output folder")
    ap.add_argument("--route", default=ROUTE_FILE, help="SUMO route file (.rou.xml)")
    ap.add_argument("--net", default=NET_FILE, help="SUMO network file (.net.xml[.gz])")
    args = ap.parse_args()

    csv_files = []
    for a in args.csv:
        csv_files += find_files(a) if any(c in a for c in "*?[") else [resolve(a)]
    if not csv_files:
        ap.error(f"no CSV files found for DATA_PATTERN = '{DATA_PATTERN}'.\n"
                 "  Looked in: " + "; ".join(_SEARCH_DIRS) + "\n"
                 "  Fix DATA_PATTERN in the config, or pass the files on the command line.")
    not_found = [p for p in csv_files if not os.path.isfile(p)]
    if not_found:
        ap.error("CSV file not found: " + ", ".join(not_found))
    print(f"[data] {len(csv_files)} run file(s):")
    for p in csv_files:
        print(f"       {run_name(os.path.splitext(os.path.basename(p))[0])}: {p}")

    route = load_route(resolve(args.route), resolve(args.net))
    if route is not None:
        check_route_alignment(route, load_run(csv_files[0])[0])

    total = sum(plot_run(p, args.out, route) for p in csv_files)
    total += plot_all_runs(csv_files, args.out, route)
    print(f"Done: {total} figures.")


if __name__ == "__main__":
    main()