"""
live_accuracy.py
======================================================================
Map-matching accuracy for LIVE rides, with enough separate metrics to
tell which method is actually best -- and to stop one that "games" a
single metric (e.g. by freezing on the road while the rider moves on)
from looking like the winner.

THE CORE IDEA: A REFERENCE FOR WHERE THE RIDER REALLY WAS
---------------------------------------------------------
A live ride has no per-fix ground truth, but the route is known (the
<route edges="..."/> in ROUTE_FILE). So the script:

  1. lays the route out as one continuous line with a distance-along-
     route ("chainage", s) at every point,
  2. snaps every RAW phone fix onto that line, searching only a window
     ahead of the previous fix so it can't jump to another part of the
     route (junctions, loops, the route passing itself),
  3. cleans that progress curve: off-route GPS outliers are dropped and
     interpolated, a rolling median removes along-route noise, and an
     isotonic fit makes it non-decreasing (the rider doesn't ride
     backwards along the route).

The result, s_ref, is the best estimate of how far along the route the
rider was at each fix. It is built from RAW GPS only and is identical
for every method, so Kalman and non-Kalman groups face the same
yardstick, and the Kalman filter can't move it.

Every matched point is then split into two components:
  along-track = s_matched - s_ref   (negative = behind the rider)
  cross-track = distance to the nearest lane of the route
and the 2D error is hypot(along, cross).

METRICS (per method/kalman group)
---------------------------------
Position
  err_mean / err_median / err_p95 / err_max / err_rmse  2D error (m)
  within_5m_pct / within_10m_pct  share of fixes within 5 m / 10 m
Along the route
  along_bias   mean signed along-track error (m). Negative = lags behind
  along_mae    mean |along-track error| (m)
Off the route
  cross_mean / cross_p95   distance to the route's lanes (m)
Road choice
  edge_pct     fixes on the edge the rider was actually on at that moment
               (within EDGE_TOL_M of an edge boundary, either edge counts)
  route_pct    fixes on ANY edge of the route (looser; ignores timing)
  nk_error     Newson & Krumm route error (d+ + d-)/d0 over the part of
               the route this ride covered. 0 = perfect
Behaviour
  stuck_pct    of the fixes where the rider moved >= MOVING_M, the share
               where the matched point didn't move at all
  jumps_per_km matched-point steps faster than REF_MAX_SPEED_MPS allows
  path_ratio   matched path length / distance actually ridden.
               ~1 good; >1 zig-zag / back-and-forth; <1 skipping
  match_pct    share of fixes the matcher returned a match for
  latency_ms / latency_p95_ms   processing time per fix (match_ms)

UNMATCHED FIXES
---------------
UNMATCHED_POLICY = "hold_last" (default) scores an unmatched fix as the
last matched position, i.e. what a live display would actually show.
"exclude" leaves them out of the error stats (match_pct still shows how
many there were).

RANKING
-------
Metrics are grouped into categories (RANK_CATEGORIES): position, road
choice, behaviour, and speed (weight 0 by default, shown but not
counted). Each metric is scored 0..100 across the groups (100 = best
group, 0 = worst), averaged within its category, and the categories are
combined with their weights into one overall score. Grouping stops any
one aspect (e.g. four closely related position metrics) from outvoting
the rest.

RANK_MODE = "score" scales each metric by the size of the gaps, so a
0.7% vs 0% difference barely matters but 26% vs 0% does. "rank" uses
the order only.

The ranking also scores every run on its own and shows each group's
place in each run. A winner that isn't 1st in most runs isn't a clear
winner -- report it as a tie.

LIMITATIONS
-----------
- s_ref is an estimate built from raw GPS. A matcher that simply drops
  each raw fix straight onto the correct road tends to score close to
  zero along the route, which slightly favours plain snapping matchers.
  Cross-track and road-choice metrics don't depend on s_ref.
- Assumes each ride followed ROUTE_FILE from (near) its start and that
  raw_x / matched_x are SUMO network coordinates. The script warns if
  the raw GPS sits far from the route or starts away from the start.
- Along-track error is capped at +/- MATCH_SEARCH_M.

OUTPUTS (in OUTPUT_DIR)
-----------------------
  accuracy_per_run.csv   every metric, one row per run x group
  accuracy_summary.csv   every metric averaged over runs (+ run-to-run std
                         of err_mean)
  accuracy_ranking.csv   overall + category scores, place in each run,
                         and the 0..100 score for every ranked metric
  accuracy_per_fix.csv   per-fix along / cross / 2D error and edge check,
                         for plotting error along the ride

No command-line arguments: edit the CONFIG block below and re-run.
Needs SUMO_HOME (for sumolib) and numpy.
----------------------------------------------------------------------
"""

import csv
import math
import os
import statistics
import sys
import xml.etree.ElementTree as ET
from collections import Counter

import numpy as np

# =========================== CONFIG ===========================
# Point straight at a .net.xml, OR leave NET_FILE as None and point
# SUMOCFG at the .sumocfg -- the net file is then read out of it.
NET_FILE = None
SUMOCFG = r"2026-08-25-19-43-30/osm.sumocfg"

ROUTE_FILE = "2026-08-25-19-43-30/full_campus_v3.rou.xml"
ROUTE_ID = None  # None = auto-pick when the file defines exactly one route

# One path per run. Relative paths are relative to this script's folder.
RUN_FILES = [
    "C:/aaUniFiles/FYP/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-152039.csv",
    "C:/aaUniFiles/FYP/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-152040.csv",
    "C:/aaUniFiles/FYP/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-155952.csv",
    "C:/aaUniFiles/FYP/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-155953.csv",
]

# CSV columns
COL_METHOD, COL_KALMAN, COL_FIX = "method", "kalman", "fix_seq"
COL_TIME = "phone_timestamp"          # seconds; falls back to fix_seq if absent
COL_RAW = ("raw_x", "raw_y")
COL_FILT = ("filt_x", "filt_y")
COL_MATCHED = ("matched_x", "matched_y")
COL_MATCHED_FLAG = "matched"
COL_EDGE = "edge_id"
COL_LATENCY = "match_ms"

UNMATCHED_POLICY = "hold_last"         # "hold_last" or "exclude"

# Reference (s_ref) construction
REF_INIT_SEARCH_M = 200.0   # first fix is searched within this far of the route
                            # start; None = search the whole route
REF_BACK_M = 50.0           # how far back from the previous fix to search
REF_AHEAD_M = 50.0          # + REF_MAX_SPEED_MPS * dt ahead of the previous fix
REF_MAX_SPEED_MPS = 12.0    # eBike top speed (~43 km/h); also used for jumps
REF_OUTLIER_M = 25.0        # raw fixes further than this from the route are
                            # treated as GPS outliers and interpolated over
REF_SMOOTH_FIXES = 5        # rolling-median window (odd); 1 = no smoothing

MATCH_SEARCH_M = 150.0      # matched points are located within +/- this of s_ref

# Metric thresholds
WITHIN_M = (5.0, 10.0)
EDGE_TOL_M = 10.0           # near an edge boundary, either edge counts as correct
MOVING_M = 2.0              # rider counts as moving if s_ref advanced this much
STUCK_M = 0.1               # matched point "didn't move" below this step
JUMP_MARGIN_M = 10.0        # jump if step > REF_MAX_SPEED_MPS * dt + this

# Overall ranking. Each metric is scored 100 (best group) .. 0 (worst group),
# averaged within its category, then the categories are combined with these
# weights. Weight 0 = shown in the table but not counted in 'overall'.
RANK_MODE = "score"   # "score": scaled by how big the gaps are
                      # "rank":  order only, ignores gap size
RANK_CATEGORIES = {
    "position":  (1.0, ["err_mean", "err_p95", "within_5m_pct", "cross_mean"]),
    "road":      (1.0, ["edge_pct", "route_pct", "nk_error"]),
    "behaviour": (1.0, ["stuck_pct", "jumps_per_km", "match_pct"]),
    "speed":     (0.0, ["latency_ms"]),
}

OUTPUT_DIR = "Data/accuracy"
WRITE_PER_FIX = True
# ==============================================================

if "SUMO_HOME" not in os.environ:
    raise EnvironmentError("SUMO_HOME is not set. Set it before running this script.")
_TOOLS = os.path.join(os.environ["SUMO_HOME"], "tools")
if _TOOLS not in sys.path:
    sys.path.append(_TOOLS)

import sumolib  # noqa: E402

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

_WITHIN_KEYS = [f"within_{int(m) if float(m).is_integer() else m}m_pct" for m in WITHIN_M]

# key, column header, number format, which direction is better
#   low  = smaller is better      high = bigger is better
#   abs  = closer to 0 is better  one  = closer to 1 is better
METRICS = [
    ("match_pct", "match%", ".1f", "high"),
    ("err_mean", "err_mean", ".2f", "low"),
    ("err_median", "err_med", ".2f", "low"),
    ("err_p95", "err_p95", ".2f", "low"),
    ("err_max", "err_max", ".1f", "low"),
    ("err_rmse", "err_rmse", ".2f", "low"),
    *[(k, k.replace("within_", "<=").replace("_pct", "%"), ".1f", "high") for k in _WITHIN_KEYS],
    ("along_bias", "lag", "+.2f", "abs"),
    ("along_mae", "along_mae", ".2f", "low"),
    ("cross_mean", "cross_mean", ".2f", "low"),
    ("cross_p95", "cross_p95", ".2f", "low"),
    ("edge_pct", "edge%", ".1f", "high"),
    ("route_pct", "route%", ".1f", "high"),
    ("nk_error", "NK_err", ".3f", "low"),
    ("stuck_pct", "stuck%", ".1f", "low"),
    ("jumps_per_km", "jumps/km", ".2f", "low"),
    ("path_ratio", "path_ratio", ".3f", "one"),
    ("latency_ms", "lat_ms", ".1f", "low"),
    ("latency_p95_ms", "lat_p95", ".1f", "low"),
]
METRIC_KEYS = [m[0] for m in METRICS]
METRIC_BY_KEY = {m[0]: m for m in METRICS}


# ------------------------------------------------------------ small helpers

def _resolve(path):
    return path if os.path.isabs(path) else os.path.join(SCRIPT_DIR, path)


def _to_float(value):
    if value in (None, ""):
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(v) else v


def _is_true(value):
    return str(value).strip().lower() in ("true", "1", "yes")


def _run_name(path):
    stem = os.path.splitext(os.path.basename(path))[0]
    return stem.replace("all_methods_", "", 1)


def _group_label(method, kalman_on):
    return f"{method}_kalman" if kalman_on else method


def _is_missing(v):
    return v is None or (isinstance(v, float) and math.isnan(v))


def _fmt(v, fmt):
    return "--" if _is_missing(v) else format(v, fmt)


def _pct(num, den):
    return 100.0 * num / den if den else None


def parse_sumocfg_for_netfile(sumocfg_path):
    root = ET.parse(sumocfg_path).getroot()
    input_tag = root.find("input")
    if input_tag is None:
        raise ValueError(f"No <input> section found in {sumocfg_path}")
    net_tag = input_tag.find("net-file")
    if net_tag is None or not net_tag.get("value"):
        raise ValueError(f"No usable <net-file> entry in {sumocfg_path}")
    base_dir = os.path.dirname(os.path.abspath(sumocfg_path))
    return os.path.abspath(os.path.join(base_dir, net_tag.get("value")))


def load_ground_truth_route(route_file, route_id=None):
    routes = ET.parse(route_file).getroot().findall(".//route")
    if not routes:
        raise ValueError(f"No <route> elements found in {route_file}")
    if route_id is not None:
        for r in routes:
            if r.get("id") == route_id:
                return r.get("edges", "").split()
        available = ", ".join(r.get("id", "<no id>") for r in routes)
        raise ValueError(f"No route '{route_id}' in {route_file}. Available: {available}")
    if len(routes) == 1:
        return routes[0].get("edges", "").split()
    available = ", ".join(r.get("id", "<no id>") for r in routes)
    raise ValueError(f"{route_file} defines {len(routes)} routes; set ROUTE_ID to one of: {available}")


def rolling_median(y, window):
    if window <= 1 or len(y) < 3:
        return y.copy()
    h = window // 2
    return np.array([np.median(y[max(0, i - h): i + h + 1]) for i in range(len(y))])


def isotonic_increasing(y):
    """Least-squares non-decreasing fit (pool adjacent violators)."""
    vals, wts = [], []
    for v in y:
        vals.append(float(v))
        wts.append(1)
        while len(vals) > 1 and vals[-2] > vals[-1]:
            w = wts[-2] + wts[-1]
            v = (vals[-2] * wts[-2] + vals[-1] * wts[-1]) / w
            vals[-2:] = [v]
            wts[-2:] = [w]
    return np.repeat(vals, wts)


# ---------------------------------------------------------- route geometry

class RouteLine:
    """The route as ONE ordered polyline (edge centrelines joined through
    the junction lanes between them), with chainage s along it."""

    def __init__(self, net, route_edges):
        pieces, missing = [], []
        prev_edge = None
        for edge_id in route_edges:
            try:
                edge = net.getEdge(edge_id)
            except Exception:
                missing.append(edge_id)
                prev_edge = None
                continue
            if prev_edge is not None:
                for conn in prev_edge.getConnections(edge):
                    via = conn.getViaLaneID()
                    if not via:
                        continue
                    try:
                        lane = net.getLane(via)
                        pieces.append((lane.getEdge().getID(), "junction",
                                       [tuple(p[:2]) for p in lane.getShape()]))
                    except Exception:
                        pass
                    break
            pieces.append((edge_id, "edge", [tuple(p[:2]) for p in edge.getShape()]))
            prev_edge = edge

        if missing:
            print(f"[WARN] {len(missing)} route edge(s) not in the network: "
                  f"{', '.join(missing[:10])}" + (" ..." if len(missing) > 10 else ""))

        pts, spans = [], []
        for edge_id, kind, shape in pieces:
            shape = [(float(x), float(y)) for x, y in shape]
            if pts and shape and math.hypot(shape[0][0] - pts[-1][0], shape[0][1] - pts[-1][1]) < 1e-6:
                shape = shape[1:]
            # the joining segment from the previous piece's end belongs to this piece
            start = len(pts) - 1 if pts else 0
            pts.extend(shape)
            spans.append((start, len(pts) - 1, edge_id, kind))
        if len(pts) < 2:
            raise ValueError("Ground-truth route has no usable geometry.")

        self.P = np.asarray(pts, dtype=float)
        self.seg = np.diff(self.P, axis=0)
        self.seglen = np.hypot(self.seg[:, 0], self.seg[:, 1])
        self.seg2 = self.seglen ** 2
        self.s = np.concatenate([[0.0], np.cumsum(self.seglen)])
        self.length = float(self.s[-1])
        self.nseg = len(self.seg)

        self.span_s0 = np.array([self.s[a] for a, _, _, _ in spans])
        self.span_s1 = np.array([self.s[b] for _, b, _, _ in spans])
        self.span_id = [e for _, _, e, _ in spans]
        self.span_kind = [k for _, _, _, k in spans]
        self.edge_ids = {e for e, k in zip(self.span_id, self.span_kind) if k == "edge"}

    def project(self, x, y, lo=None, hi=None):
        """Nearest point on the route with chainage in [lo, hi].
        Returns (s, distance)."""
        lo = 0.0 if lo is None else max(lo, 0.0)
        hi = self.length if hi is None else min(hi, self.length)
        if hi < lo:
            lo = hi
        i0 = max(int(np.searchsorted(self.s, lo, side="right")) - 1, 0)
        i0 = min(i0, self.nseg - 1)
        i1 = min(int(np.searchsorted(self.s, hi, side="left")), self.nseg)
        if i1 <= i0:
            i1 = i0 + 1
        A, AB, L2 = self.P[i0:i1], self.seg[i0:i1], self.seg2[i0:i1]
        t = ((x - A[:, 0]) * AB[:, 0] + (y - A[:, 1]) * AB[:, 1]) / np.where(L2 > 0, L2, 1.0)
        t = np.clip(np.where(L2 > 0, t, 0.0), 0.0, 1.0)
        d = np.hypot(A[:, 0] + t * AB[:, 0] - x, A[:, 1] + t * AB[:, 1] - y)
        k = int(np.argmin(d))
        s = self.s[i0 + k] + t[k] * self.seglen[i0 + k]
        return float(min(max(s, lo), hi)), float(d[k])

    def acceptable_edges(self, s, tol):
        """Edge IDs the rider could legitimately be on at chainage s, and
        whether a junction is within tol."""
        mask = (self.span_s0 <= s + tol) & (self.span_s1 >= s - tol)
        ids = {self.span_id[i] for i in np.flatnonzero(mask)}
        near_junction = any(self.span_kind[i] == "junction" for i in np.flatnonzero(mask))
        return ids, near_junction

    def covered_edges(self, s_lo, s_hi):
        """Route edges (in order, repeats kept) that a ride from s_lo to
        s_hi covered for at least half their length."""
        out = []
        for s0, s1, eid, kind in zip(self.span_s0, self.span_s1, self.span_id, self.span_kind):
            if kind != "edge":
                continue
            overlap = min(s1, s_hi) - max(s0, s_lo)
            if (s1 - s0) > 0 and overlap >= 0.5 * (s1 - s0):
                out.append(eid)
        return out


class RouteLanes:
    """Every lane of the route (plus junction lanes) for cross-track
    distance, so a point on any lane of the right road scores ~0."""

    def __init__(self, net, route_edges):
        shapes = []
        for edge_id in dict.fromkeys(route_edges):
            try:
                edge = net.getEdge(edge_id)
            except Exception:
                continue
            shapes.extend(lane.getShape() for lane in edge.getLanes())
        for a, b in zip(route_edges, route_edges[1:]):
            try:
                ea, eb = net.getEdge(a), net.getEdge(b)
            except Exception:
                continue
            for conn in ea.getConnections(eb):
                via = conn.getViaLaneID()
                if via:
                    try:
                        shapes.append(net.getLane(via).getShape())
                    except Exception:
                        pass
        starts, ends = [], []
        for shape in shapes:
            for p, q in zip(shape, shape[1:]):
                starts.append(p[:2])
                ends.append(q[:2])
        self.A = np.asarray(starts, dtype=float)
        self.AB = np.asarray(ends, dtype=float) - self.A
        self.AB2 = (self.AB ** 2).sum(axis=1)
        self._safe = np.where(self.AB2 > 0, self.AB2, 1.0)

    def distances(self, points, chunk=512):
        P = np.asarray(points, dtype=float).reshape(-1, 2)
        out = np.empty(len(P))
        A, AB = self.A[None], self.AB[None]
        for i in range(0, len(P), chunk):
            p = P[i:i + chunk, None, :]
            t = ((p - A) * AB).sum(axis=2) / self._safe
            t = np.clip(np.where(self.AB2 > 0, t, 0.0), 0.0, 1.0)
            c = A + t[..., None] * AB
            out[i:i + chunk] = np.hypot(c[..., 0] - p[..., 0], c[..., 1] - p[..., 1]).min(axis=1)
        return out


# ---------------------------------------------------------------- loading

def load_run(csv_path):
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        required = [COL_METHOD, COL_KALMAN, COL_FIX, *COL_RAW, *COL_MATCHED]
        missing = [c for c in required if c not in fields]
        if missing:
            raise ValueError(f"missing column(s) {missing}. Found: {fields}")
        has_time = COL_TIME in fields
        has_edge = COL_EDGE in fields
        has_flag = COL_MATCHED_FLAG in fields
        has_lat = COL_LATENCY in fields
        has_filt = all(c in fields for c in COL_FILT)

        groups = {}
        first_method = None
        raw_fixes, filt_fixes = {}, {}
        for row in reader:
            method = row[COL_METHOD]
            k_on = _is_true(row[COL_KALMAN])
            fix = int(float(row[COL_FIX]))
            t = _to_float(row[COL_TIME]) if has_time else None
            if t is None:
                t = float(fix)
            if first_method is None:
                first_method = method

            if method == first_method and not k_on:
                rx, ry = _to_float(row[COL_RAW[0]]), _to_float(row[COL_RAW[1]])
                if rx is not None and ry is not None:
                    raw_fixes[fix] = (t, rx, ry)
            if method == first_method and k_on and has_filt:
                fx, fy = _to_float(row[COL_FILT[0]]), _to_float(row[COL_FILT[1]])
                if fx is not None and fy is not None:
                    filt_fixes[fix] = (fx, fy)

            mx, my = _to_float(row[COL_MATCHED[0]]), _to_float(row[COL_MATCHED[1]])
            matched = (not has_flag or _is_true(row[COL_MATCHED_FLAG])) and mx is not None and my is not None
            groups.setdefault(_group_label(method, k_on), []).append({
                "fix": fix, "t": t, "matched": matched, "x": mx, "y": my,
                "edge": (row.get(COL_EDGE) or "") if has_edge else "",
                "latency": _to_float(row[COL_LATENCY]) if has_lat else None,
            })

    for rows in groups.values():
        rows.sort(key=lambda r: r["fix"])
    return {"groups": groups, "raw": raw_fixes, "filt": filt_fixes,
            "has_time": has_time, "has_edge": has_edge, "has_lat": has_lat}


# -------------------------------------------------------------- reference

def build_reference(route, raw_fixes):
    """raw_fixes: {fix: (t, x, y)} -> dict with arrays fix, t, s, raw_cross."""
    fixes = sorted(raw_fixes)
    t = np.array([raw_fixes[f][0] for f in fixes], dtype=float)
    s = np.full(len(fixes), np.nan)
    d_raw = np.full(len(fixes), np.nan)

    prev_s = prev_t = None
    for i, f in enumerate(fixes):
        _, x, y = raw_fixes[f]
        if prev_s is None:
            lo, hi = 0.0, (REF_INIT_SEARCH_M if REF_INIT_SEARCH_M is not None else route.length)
        else:
            dt = max(t[i] - prev_t, 1.0)
            lo, hi = prev_s - REF_BACK_M, prev_s + REF_AHEAD_M + REF_MAX_SPEED_MPS * dt
        si, di = route.project(x, y, lo, hi)
        d_raw[i] = di
        if di <= REF_OUTLIER_M:
            s[i] = si
            prev_s, prev_t = si, t[i]

    good = ~np.isnan(s)
    if good.sum() < 2:
        raise ValueError("fewer than 2 raw fixes lie near the route -- wrong route or "
                         "coordinate frame?")
    idx = np.arange(len(s))
    s = np.interp(idx, idx[good], s[good])
    s = isotonic_increasing(rolling_median(s, REF_SMOOTH_FIXES))
    return {"fix": fixes, "t": t, "s": s, "raw_cross_to_centreline": d_raw,
            "outlier_pct": 100.0 * (~good).mean()}


# ---------------------------------------------------------------- metrics

def _stats(arr):
    a = np.asarray(arr, dtype=float)
    if a.size == 0:
        return None
    return {"mean": float(a.mean()), "median": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max()),
            "rmse": float(np.sqrt((a ** 2).mean()))}


def newson_krumm(true_route, matched_route, net, cache):
    def length(e):
        if e not in cache:
            try:
                cache[e] = net.getEdge(e).getLength()
            except Exception:
                cache[e] = 0.0
        return cache[e]
    tc, mc = Counter(true_route), Counter(matched_route)
    d0 = sum(length(e) * n for e, n in tc.items())
    d_minus = sum(length(e) * max(n - mc.get(e, 0), 0) for e, n in tc.items())
    d_plus = sum(length(e) * max(n - tc.get(e, 0), 0) for e, n in mc.items())
    return (d_plus + d_minus) / d0 if d0 > 0 else None


def locate(route, xs, ys, s_ref):
    """Chainage of each point, searched within +/- MATCH_SEARCH_M of s_ref."""
    return np.array([route.project(x, y, sr - MATCH_SEARCH_M, sr + MATCH_SEARCH_M)[0]
                     for x, y, sr in zip(xs, ys, s_ref)])


def group_metrics(route, lanes, ref, rows, has_edge, net, len_cache):
    ref_s = dict(zip(ref["fix"], ref["s"]))
    rows = [r for r in rows if r["fix"] in ref_s]
    n_rows = len(rows)
    if n_rows == 0:
        return None, []

    seq, last = [], None
    for r in rows:
        if r["matched"]:
            last = r
            seq.append((r, r["x"], r["y"], r["edge"], False))
        elif UNMATCHED_POLICY == "hold_last" and last is not None:
            seq.append((r, last["x"], last["y"], last["edge"], True))
    if not seq:
        return None, []

    fixes = np.array([r["fix"] for r, *_ in seq])
    t = np.array([r["t"] for r, *_ in seq], dtype=float)
    xs = np.array([x for _, x, _, _, _ in seq])
    ys = np.array([y for _, _, y, _, _ in seq])
    edges = [e for _, _, _, e, _ in seq]
    held = [h for *_, h in seq]
    s_ref = np.array([ref_s[f] for f in fixes])

    cross = lanes.distances(np.column_stack([xs, ys]))
    s_m = locate(route, xs, ys, s_ref)
    along = s_m - s_ref
    err = np.hypot(along, cross)
    es = _stats(err)
    cs = _stats(cross)

    m = {
        "n": len(err),
        "match_pct": _pct(sum(r["matched"] for r in rows), n_rows),
        "err_mean": es["mean"], "err_median": es["median"], "err_p95": es["p95"],
        "err_max": es["max"], "err_rmse": es["rmse"],
        "along_bias": float(along.mean()), "along_mae": float(np.abs(along).mean()),
        "cross_mean": cs["mean"], "cross_p95": cs["p95"],
    }
    for key, lim in zip(_WITHIN_KEYS, WITHIN_M):
        m[key] = 100.0 * float((err <= lim).mean())

    edge_ok = [None] * len(edges)
    if has_edge:
        ok_count = 0
        for i, (e, sr) in enumerate(zip(edges, s_ref)):
            ids, near_j = route.acceptable_edges(sr, EDGE_TOL_M)
            ok = bool(e) and (e in ids or (e.startswith(":") and near_j))
            edge_ok[i] = ok
            ok_count += ok
        m["edge_pct"] = _pct(ok_count, len(edges))
        real = [e for e in edges if e and not e.startswith(":")]
        m["route_pct"] = _pct(sum(e in route.edge_ids for e in real), len(real))

        matched_seq = []
        for r in rows:
            e = r["edge"]
            if r["matched"] and e and not e.startswith(":") and (not matched_seq or matched_seq[-1] != e):
                matched_seq.append(e)
        truth = route.covered_edges(float(ref["s"].min()), float(ref["s"].max()))
        m["nk_error"] = newson_krumm(truth, matched_seq, net, len_cache)
    else:
        m["edge_pct"] = m["route_pct"] = m["nk_error"] = None

    step = np.hypot(np.diff(xs), np.diff(ys))
    ds_ref = np.diff(s_ref)
    dt = np.maximum(np.diff(t), 1.0)
    moving = ds_ref >= MOVING_M
    m["stuck_pct"] = _pct(int((moving & (step < STUCK_M)).sum()), int(moving.sum()))
    ridden_m = float(s_ref[-1] - s_ref[0])
    jumps = int((step > REF_MAX_SPEED_MPS * dt + JUMP_MARGIN_M).sum())
    m["jumps_per_km"] = jumps / (ridden_m / 1000.0) if ridden_m > 0 else None
    m["path_ratio"] = float(step.sum()) / ridden_m if ridden_m > 0 else None

    lat = [r["latency"] for r in rows if r["latency"] is not None]
    m["latency_ms"] = float(np.mean(lat)) if lat else None
    m["latency_p95_ms"] = float(np.percentile(lat, 95)) if lat else None

    per_fix = [(int(f), float(sr), float(a), float(c), float(e2), ed, ok, h)
               for f, sr, a, c, e2, ed, ok, h in zip(fixes, s_ref, along, cross, err, edges, edge_ok, held)]
    return m, per_fix


def input_metrics(route, lanes, ref, points_by_fix):
    """Baseline for an input stream (raw or Kalman-filtered fixes)."""
    ref_s = dict(zip(ref["fix"], ref["s"]))
    fixes = [f for f in sorted(points_by_fix) if f in ref_s]
    if not fixes:
        return None
    xy = np.array([points_by_fix[f] for f in fixes], dtype=float)
    s_ref = np.array([ref_s[f] for f in fixes])
    cross = lanes.distances(xy)
    along = locate(route, xy[:, 0], xy[:, 1], s_ref) - s_ref
    err = np.hypot(along, cross)
    return {"n": len(fixes), "err_mean": float(err.mean()), "err_p95": float(np.percentile(err, 95)),
            "along_bias": float(along.mean()), "along_mae": float(np.abs(along).mean()),
            "cross_mean": float(cross.mean()), "cross_p95": float(np.percentile(cross, 95))}


# ---------------------------------------------------------------- ranking

def _badness(v, direction):
    return {"low": v, "high": -v, "abs": abs(v), "one": abs(v - 1.0)}[direction]


def _scaled_ranks(bad):
    """{group: badness} -> {group: 0..1}, 0 = best; ties share a rank."""
    ordered = sorted(bad, key=bad.get)
    n = len(ordered)
    out, i = {}, 0
    while i < n:
        j = i
        while j + 1 < n and math.isclose(bad[ordered[j + 1]], bad[ordered[i]],
                                         rel_tol=1e-9, abs_tol=1e-9):
            j += 1
        r = (i + j) / 2
        for g in ordered[i:j + 1]:
            out[g] = r / (n - 1) if n > 1 else 0.0
        i = j + 1
    return out


def score_groups(metrics_by_group):
    """Returns (metric_scores, category_scores, overall), all 0..100,
    higher = better."""
    labels = [g for g, m in metrics_by_group.items() if m is not None]
    metric_scores = {g: {} for g in labels}
    for _, keys in RANK_CATEGORIES.values():
        for k in keys:
            if k not in METRIC_BY_KEY:
                continue
            direction = METRIC_BY_KEY[k][3]
            bad = {g: _badness(metrics_by_group[g][k], direction)
                   for g in labels if not _is_missing(metrics_by_group[g].get(k))}
            if not bad:
                continue
            if RANK_MODE == "rank":
                scaled = _scaled_ranks(bad)
            else:
                lo, hi = min(bad.values()), max(bad.values())
                scaled = {g: (b - lo) / (hi - lo) if hi > lo else 0.0 for g, b in bad.items()}
            for g, v in scaled.items():
                metric_scores[g][k] = 100.0 * (1.0 - v)

    cat_scores, overall = {g: {} for g in labels}, {}
    for g in labels:
        total = wsum = 0.0
        for cat, (w, keys) in RANK_CATEGORIES.items():
            vals = [metric_scores[g][k] for k in keys if k in metric_scores[g]]
            if not vals:
                continue
            cat_scores[g][cat] = statistics.mean(vals)
            if w > 0:
                total += w * cat_scores[g][cat]
                wsum += w
        overall[g] = total / wsum if wsum else None
    return metric_scores, cat_scores, overall


def _places(overall):
    order = sorted((g for g in overall if overall[g] is not None), key=lambda g: -overall[g])
    return {g: i for i, g in enumerate(order, 1)}


# ----------------------------------------------------------------- output

def _print_table(title, rows, keys, extra_first=None):
    """rows: list of (label, metrics-dict-or-None)."""
    heads = [METRIC_BY_KEY[k][1] for k in keys]
    widths = [max(8, len(h) + 1) for h in heads]
    head = f"{'group':<16}"
    if extra_first:
        head += f"{extra_first[0]:>7}"
    head += "".join(f"{h:>{w}}" for h, w in zip(heads, widths))
    line = "=" * len(head)
    print(f"\n{line}\n{title}\n{head}\n{'-' * len(head)}")
    for label, m in rows:
        out = f"{label:<16}"
        if extra_first:
            out += f"{_fmt(extra_first[1](label, m), extra_first[2]):>7}"
        for k, w in zip(keys, widths):
            v = None if m is None else m.get(k)
            out += f"{_fmt(v, METRIC_BY_KEY[k][2]):>{w}}"
        print(out)
    print(line)


def _write_csv(path, header, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def _cell(v):
    return "" if _is_missing(v) else v


def main():
    net_file = NET_FILE
    if net_file is None:
        cfg = _resolve(SUMOCFG)
        if not os.path.exists(cfg):
            raise FileNotFoundError(f"SUMO config not found: {cfg}")
        net_file = parse_sumocfg_for_netfile(cfg)
    route_file = _resolve(ROUTE_FILE)
    if not os.path.exists(route_file):
        raise FileNotFoundError(f"Route file not found: {route_file}")

    print(f"[INFO] Net file:   {net_file}")
    print(f"[INFO] Route file: {route_file}")
    print("[INFO] Loading network with sumolib...")
    net = sumolib.net.readNet(net_file, withInternal=True)
    route_edges = load_ground_truth_route(route_file, ROUTE_ID)
    route = RouteLine(net, route_edges)
    lanes = RouteLanes(net, route_edges)
    print(f"[INFO] Route: {len(route_edges)} edges, {route.length:.0f} m long")
    print(f"[INFO] Unmatched fixes: {UNMATCHED_POLICY}")

    by_group = {}       # label -> [metrics per run]
    per_run_rows = []   # (run, label, metrics)
    per_fix_rows = []
    len_cache = {}
    headline = ["match_pct", "err_mean", "err_p95", _WITHIN_KEYS[0], "along_bias",
                "cross_mean", "edge_pct", "nk_error", "stuck_pct", "jumps_per_km"]

    for path in RUN_FILES:
        abs_path = _resolve(path)
        run = _run_name(path)
        if not os.path.exists(abs_path):
            print(f"[WARN] run {run}: file not found at {abs_path} -- skipping.")
            continue
        try:
            data = load_run(abs_path)
            ref = build_reference(route, data["raw"])
        except ValueError as e:
            print(f"[WARN] run {run}: {e} -- skipping.")
            continue

        ridden = float(ref["s"][-1] - ref["s"][0])
        first_d = float(ref["raw_cross_to_centreline"][0])
        print(f"\n[INFO] run {run}: {len(ref['fix'])} fixes, covers route "
              f"{ref['s'][0]:.0f}-{ref['s'][-1]:.0f} m ({100 * ridden / route.length:.0f}% of route), "
              f"{ref['outlier_pct']:.1f}% raw fixes treated as outliers")
        if not data["has_time"]:
            print(f"[WARN] run {run}: no '{COL_TIME}' column -- using fix_seq as time (1 s per fix).")
        if ref["outlier_pct"] > 20 or np.nanmedian(ref["raw_cross_to_centreline"]) > 15:
            print(f"[WARN] run {run}: raw GPS sits far from the route. Check this ride followed "
                  f"{os.path.basename(ROUTE_FILE)} and that coordinates are SUMO network x/y.")
        if REF_INIT_SEARCH_M is not None and first_d > REF_OUTLIER_M:
            print(f"[WARN] run {run}: first fix is {first_d:.0f} m from the route start. If the ride "
                  f"didn't start there, set REF_INIT_SEARCH_M = None.")

        table = []
        for name, pts in (("input:raw_gps", {f: v[1:] for f, v in data["raw"].items()}),
                          ("input:kalman", data["filt"])):
            if pts:
                im = input_metrics(route, lanes, ref, pts)
                table.append((name, im))
                by_group.setdefault(name, []).append(im)
                per_run_rows.append((run, name, im))

        for label, rows in data["groups"].items():
            m, per_fix = group_metrics(route, lanes, ref, rows, data["has_edge"], net, len_cache)
            table.append((label, m))
            by_group.setdefault(label, []).append(m)
            per_run_rows.append((run, label, m))
            per_fix_rows.extend((run, label, *pf) for pf in per_fix)

        _print_table(f"Run {run}", table, headline)

    if not by_group:
        print("\n[WARN] No runs produced results.")
        return

    averaged, n_runs_used, err_sd = {}, {}, {}
    for label, ms in by_group.items():
        usable = [m for m in ms if m is not None]
        n_runs_used[label] = len(usable)
        if not usable:
            averaged[label] = None
            continue
        avg = {}
        for k in METRIC_KEYS + ["n"]:
            vals = [m.get(k) for m in usable if not _is_missing(m.get(k))]
            avg[k] = statistics.mean(vals) if vals else None
        averaged[label] = avg
        errs = [m["err_mean"] for m in usable if not _is_missing(m.get("err_mean"))]
        err_sd[label] = statistics.stdev(errs) if len(errs) > 1 else None

    n_runs = len({r for r, _, _ in per_run_rows})
    methods = {g: m for g, m in averaged.items() if not g.startswith("input:")}
    inputs = [(g, m) for g, m in averaged.items() if g.startswith("input:")]

    if inputs:
        _print_table(f"INPUT QUALITY, average of {n_runs} run(s)  (raw GPS is the reference "
                     f"source, so its along-track numbers are optimistic)",
                     inputs, ["err_mean", "err_p95", "along_bias", "along_mae",
                              "cross_mean", "cross_p95"])
    _print_table(f"POSITION ACCURACY, average of {n_runs} run(s)",
                 list(methods.items()),
                 ["match_pct", "err_mean", "err_median", "err_p95", "err_max", "err_rmse",
                  *_WITHIN_KEYS],
                 extra_first=("+/-", lambda g, m: err_sd.get(g), ".2f"))
    _print_table(f"ALONG / OFF ROUTE + ROAD CHOICE, average of {n_runs} run(s)",
                 list(methods.items()),
                 ["along_bias", "along_mae", "cross_mean", "cross_p95",
                  "edge_pct", "route_pct", "nk_error"])
    _print_table(f"BEHAVIOUR, average of {n_runs} run(s)",
                 list(methods.items()),
                 ["stuck_pct", "jumps_per_km", "path_ratio", "latency_ms", "latency_p95_ms"])

    # ---------------- overall ranking ----------------
    unknown = [k for _, ks in RANK_CATEGORIES.values() for k in ks if k not in METRIC_BY_KEY]
    if unknown:
        print(f"[WARN] unknown metric(s) in RANK_CATEGORIES ignored: {unknown}")

    metric_scores, cat_scores, overall = score_groups(methods)
    order = sorted((g for g in overall if overall[g] is not None), key=lambda g: -overall[g])

    # where each group lands when each run is scored on its own
    run_names = list(dict.fromkeys(r for r, _, _ in per_run_rows))
    run_places = {g: [] for g in order}
    for run in run_names:
        per_run = {g: m for r, g, m in per_run_rows if r == run and not g.startswith("input:")}
        places = _places(score_groups(per_run)[2])
        for g in order:
            run_places[g].append(places.get(g))

    cats = list(RANK_CATEGORIES)
    cat_heads = [f"{c}{'' if RANK_CATEGORIES[c][0] > 0 else '*'}" for c in cats]
    widths = [max(9, len(h) + 1) for h in cat_heads]
    head = (f"{'#':>3}  {'group':<16}{'overall':>9}"
            + "".join(f"{h:>{w}}" for h, w in zip(cat_heads, widths))
            + "   place in each run")
    weights = ", ".join(f"{c} x{RANK_CATEGORIES[c][0]:g}" for c in cats)
    print(f"\n{'=' * len(head)}\nOVERALL RANKING  (0-100, higher = better; "
          f"mode={RANK_MODE}; weights: {weights})\n{head}\n{'-' * len(head)}")
    for pos, g in enumerate(order, 1):
        line = f"{pos:>3}  {g:<16}{overall[g]:>9.1f}"
        line += "".join(f"{_fmt(cat_scores[g].get(c), '.1f'):>{w}}" for c, w in zip(cats, widths))
        line += "   " + " ".join("-" if p is None else str(p) for p in run_places[g])
        print(line)
    print("=" * len(head))
    if any(RANK_CATEGORIES[c][0] == 0 for c in cats):
        print("* weight 0: shown for reference, not counted in 'overall'")

    # ---------------- CSV outputs ----------------
    out_dir = _resolve(OUTPUT_DIR)
    _write_csv(os.path.join(out_dir, "accuracy_per_run.csv"),
               ["run", "group", "n"] + METRIC_KEYS,
               [[run, g, _cell(None if m is None else m.get("n"))]
                + [_cell(None if m is None else m.get(k)) for k in METRIC_KEYS]
                for run, g, m in per_run_rows])
    _write_csv(os.path.join(out_dir, "accuracy_summary.csv"),
               ["group", "runs", "avg_n", "err_mean_run_std"] + METRIC_KEYS,
               [[g, n_runs_used[g], _cell(None if m is None else m.get("n")), _cell(err_sd.get(g))]
                + [_cell(None if m is None else m.get(k)) for k in METRIC_KEYS]
                for g, m in averaged.items()])
    score_keys = [k for _, ks in RANK_CATEGORIES.values() for k in ks if k in METRIC_BY_KEY]
    _write_csv(os.path.join(out_dir, "accuracy_ranking.csv"),
               ["position", "group", "overall_score"] + [f"cat_{c}" for c in cats]
               + [f"place_{r}" for r in run_names] + [f"score_{k}" for k in score_keys],
               [[i, g, overall[g]] + [_cell(cat_scores[g].get(c)) for c in cats]
                + [_cell(p) for p in run_places[g]]
                + [_cell(metric_scores[g].get(k)) for k in score_keys]
                for i, g in enumerate(order, 1)])
    if WRITE_PER_FIX:
        _write_csv(os.path.join(out_dir, "accuracy_per_fix.csv"),
                   ["run", "group", "fix_seq", "s_ref_m", "along_m", "cross_m", "err_m",
                    "edge_id", "edge_correct", "held_unmatched"],
                   [[*r[:8], _cell(r[8]), r[9]] for r in per_fix_rows])
    print(f"\n[INFO] CSVs written to {out_dir}")


if __name__ == "__main__":
    main()