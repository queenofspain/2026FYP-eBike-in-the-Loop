"""
edge_distance_error.py
======================================================================
Edge-distance error for SUMO map-matching output.

For every matched point (matched_x, matched_y) this measures the
shortest ("perpendicular" / cross-track) distance from that point to the
GROUND-TRUTH ROUTE -- i.e. to the nearest point on any of the edges
listed in the SUMO route file. A matched point sitting right on the
route scores ~0 m; a point the matcher snapped onto a parallel path or
the wrong side street scores the gap to the route. The phone's GPS
error is not involved at all, which is why these numbers should be much
smaller than the GPS-to-matched distances from positional_error.py.

Results are summarised as max / min / mean / standard deviation per
method, per run, and then averaged across runs -- same layout as
positional_error.py.

HOW THE EDGE COORDINATES ARE OBTAINED
--------------------------------------
A SUMO route file only lists edge IDs, not coordinates. The coordinates
come from the network file (net.xml):

  1. The route's edge IDs are read from the .rou.xml.
  2. Each edge's polyline (x, y shape points) is read straight out of
     the .net.xml with a plain XML parser -- no SUMO_HOME / sumolib /
     traci needed. net.xml coordinates are in the same x/y frame as the
     raw_x/raw_y/matched_x/matched_y columns (SUMO's own network frame).
  3. The nearest point on each polyline SEGMENT is found by projecting
     the matched point onto the segment and clamping to its ends, so
     the distance is perpendicular where possible and measured to the
     segment end where the projection falls off it.

GEOMETRY OPTIONS (see GEOMETRY below)
--------------------------------------
  "lane" (default) -- every lane of every route edge. A bike that is
        correctly on the route edge is then ~0 m away regardless of
        which lane the matcher picked.
  "edge" -- the edge's centre-line only (its `shape` attribute, falling
        back to its lanes if the edge has none). A correct match on a
        lane offset from the centre-line then reads as a small constant
        offset (about half a lane width per lane of separation), which
        is lane position, not a matching error.

JUNCTIONS
---------
Matched points often land on internal junction lanes (edge IDs starting
with ":"), which are not part of a <route> definition. With
INCLUDE_JUNCTION_CONNECTORS = True (default) the internal lanes that
connect consecutive route edges (taken from the net's <connection>
elements) are treated as part of the route, so a bike turning through a
junction on the correct route isn't penalised. Set it to False to
measure to the route edges only.

Rows the matcher skipped (matched=False) or that are missing a matched
coordinate are left out of the stats rather than counted as zero error.

Needs numpy. No command-line arguments: edit the CONFIG block and re-run.
----------------------------------------------------------------------
"""

import csv
import os
import statistics
import xml.etree.ElementTree as ET

import numpy as np

# ---------- EDIT THIS ----------
# Point straight at a .net.xml, OR leave NET_FILE as None and point
# SUMOCFG at the .sumocfg -- the net file is then read out of it.
NET_FILE = None
SUMOCFG = r"Monash_Campus/2026-08-25-19-43-30/osm.sumocfg"  # update to match your scenario

ROUTE_FILE = "Monash_Campus/2026-08-25-19-43-30/full_campus_v3.rou.xml"
# Which <route id="..."> counts as ground truth. Leave as None to
# auto-pick when the file defines exactly one route; set it explicitly
# if the file ever defines more than one.
ROUTE_ID = None

# One path per run. Paths are relative to this script's own directory
# unless you use an absolute path. Add or remove runs freely.
RUN_FILES = [
    "C:/Uni/Final Year Project/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-152039.csv",
    "C:/Uni/Final Year Project/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-152040.csv",
    "C:/Uni/Final Year Project/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-155952.csv",
    "C:/Uni/Final Year Project/2026FYP-eBike-in-the-Loop/Data/all_methods_20261005-155953.csv",
]

MATCHED_COLUMNS = ("matched_x", "matched_y")

# "lane" or "edge" -- see GEOMETRY OPTIONS above.
GEOMETRY = "lane"
INCLUDE_JUNCTION_CONNECTORS = True

# Where to write the CSVs. Set either to None to skip it.
SUMMARY_OUTPUT = "Data/edge_distance_summary.csv"       # averaged over runs
PER_RUN_OUTPUT = "Data/edge_distance_per_run.csv"       # one row per run x method
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


def _open_xml(path):
    """Open an XML file for ET, transparently handling gzip. SUMO's OSM
    web wizard writes the network as osm.net.xml.gz, and the .sumocfg
    points at that; ET can't read it directly (it fails with "not
    well-formed (invalid token): line 1, column 0"). The gzip magic bytes
    are checked rather than the file extension, so it works either way."""
    f = open(path, "rb")
    magic = f.read(2)
    f.seek(0)
    if magic == b"\x1f\x8b":
        f.close()
        return gzip.open(path, "rb")
    return f


# ----------------------------------------------------------------------
# SUMO file parsing
# ----------------------------------------------------------------------
def parse_sumocfg_for_netfile(sumocfg_path):
    """Read the .sumocfg XML and return the absolute path to its
    <net-file>, resolved relative to the config's own directory."""
    root = ET.parse(sumocfg_path).getroot()

    input_tag = root.find("input")
    if input_tag is None:
        raise ValueError(f"No <input> section found in {sumocfg_path}")
    net_tag = input_tag.find("net-file")
    if net_tag is None:
        raise ValueError(f"No <net-file> entry found in {sumocfg_path}")
    net_value = net_tag.get("value")
    if not net_value:
        raise ValueError(f"net-file has no value in {sumocfg_path}")

    base_dir = os.path.dirname(os.path.abspath(sumocfg_path))
    return os.path.abspath(os.path.join(base_dir, net_value))


def load_ground_truth_route(route_file, route_id=None):
    """Parse a SUMO .rou.xml and return the ground-truth route as an
    ordered list of edge IDs. Picks the sole <route> automatically when
    the file only defines one; raises with the available IDs if it
    defines several and none was named."""
    root = ET.parse(route_file).getroot()
    routes = root.findall(".//route")
    if not routes:
        raise ValueError(f"No <route> elements found in {route_file}")

    if route_id is not None:
        for r in routes:
            if r.get("id") == route_id:
                return r.get("edges", "").split()
        available = ", ".join(r.get("id", "<no id>") for r in routes)
        raise ValueError(f"No route with id '{route_id}' in {route_file}. "
                         f"Available: {available}")

    if len(routes) == 1:
        return routes[0].get("edges", "").split()

    available = ", ".join(r.get("id", "<no id>") for r in routes)
    raise ValueError(f"{route_file} defines {len(routes)} routes; set ROUTE_ID "
                     f"to one of: {available}")


def _parse_shape(text):
    """'x1,y1 x2,y2 ...' (optionally x,y,z) -> [(x, y), ...]."""
    points = []
    for token in (text or "").split():
        parts = token.split(",")
        if len(parts) >= 2:
            points.append((float(parts[0]), float(parts[1])))
    return points


def load_network(net_file):
    """Read the geometry we need out of a .net.xml.

    Returns a dict with:
      edge_shapes  : {edge_id: [(x, y), ...]}   edge centre-line (only
                     for edges that carry a `shape` attribute)
      edge_lanes   : {edge_id: [lane_id, ...]}
      lane_shapes  : {lane_id: [(x, y), ...]}
      connections  : {from_edge: [(to_edge, via_lane), ...]}  (only
                     connections that have a `via`, i.e. pass through an
                     internal junction lane)
    """
    edge_shapes, edge_lanes, lane_shapes = {}, {}, {}
    connections = {}

    # iterparse + clear() keeps memory flat on big city networks.
    for _, elem in ET.iterparse(net_file, events=("end",)):
        if elem.tag == "edge":
            edge_id = elem.get("id")
            shape = _parse_shape(elem.get("shape"))
            if shape:
                edge_shapes[edge_id] = shape
            lanes = []
            for lane in elem.findall("lane"):
                lane_id = lane.get("id")
                lane_shapes[lane_id] = _parse_shape(lane.get("shape"))
                lanes.append(lane_id)
            edge_lanes[edge_id] = lanes
            elem.clear()
        elif elem.tag == "connection":
            via = elem.get("via")
            if via:
                connections.setdefault(elem.get("from"), []).append(
                    (elem.get("to"), via))
            elem.clear()

    return {"edge_shapes": edge_shapes, "edge_lanes": edge_lanes,
            "lane_shapes": lane_shapes, "connections": connections}


def _connector_lanes(net, from_edge, to_edge):
    """Internal junction lanes used to get from from_edge to to_edge.
    Follows chains of internal lanes (junctions with internal
    sub-nodes) until the connection lands on to_edge."""
    conns = net["connections"]
    frontier = [via for to, via in conns.get(from_edge, []) if to == to_edge]
    seen, out = set(), []
    while frontier:
        lane = frontier.pop()
        if lane in seen:
            continue
        seen.add(lane)
        out.append(lane)
        internal_edge = lane.rsplit("_", 1)[0]
        frontier.extend(via for to, via in conns.get(internal_edge, [])
                        if to == to_edge)
    return out


def build_route_segments(net, route_edges, geometry="lane",
                         include_connectors=True):
    """Turn the route's edge IDs into line segments (x1, y1, x2, y2).

    Returns (A, B, segment_edge_ids, report) where A and B are (S, 2)
    arrays of segment start/end points and report is a dict of counts
    for printing. Edge IDs the network doesn't know are skipped and
    listed in report['missing_edges']."""
    if geometry not in ("lane", "edge"):
        raise ValueError(f"GEOMETRY must be 'lane' or 'edge', got {geometry!r}")

    polylines = []        # (owner_id, [(x, y), ...])
    missing_edges = []
    no_connector_pairs = 0
    n_connector_lanes = 0

    for edge_id in dict.fromkeys(route_edges):      # unique, keeps order
        if edge_id not in net["edge_lanes"]:
            missing_edges.append(edge_id)
            continue
        if geometry == "edge" and edge_id in net["edge_shapes"]:
            polylines.append((edge_id, net["edge_shapes"][edge_id]))
        else:
            for lane_id in net["edge_lanes"][edge_id]:
                polylines.append((edge_id, net["lane_shapes"][lane_id]))

    if include_connectors:
        seen_pairs = set()
        for a, b in zip(route_edges, route_edges[1:]):
            if (a, b) in seen_pairs:
                continue
            seen_pairs.add((a, b))
            lanes = _connector_lanes(net, a, b)
            if not lanes:
                no_connector_pairs += 1
            for lane_id in lanes:
                shape = net["lane_shapes"].get(lane_id)
                if shape:
                    polylines.append((lane_id.rsplit("_", 1)[0], shape))
                    n_connector_lanes += 1

    starts, ends, owners = [], [], []
    for owner, shape in polylines:
        if len(shape) == 1:                 # degenerate: treat as a point
            starts.append(shape[0]); ends.append(shape[0]); owners.append(owner)
        for p, q in zip(shape, shape[1:]):
            starts.append(p); ends.append(q); owners.append(owner)

    if not starts:
        raise ValueError("The route produced no geometry -- none of its edges "
                         "were found in the network file.")

    report = {
        "missing_edges": missing_edges,
        "n_polylines": len(polylines),
        "n_connector_lanes": n_connector_lanes,
        "no_connector_pairs": no_connector_pairs,
    }
    return np.asarray(starts, float), np.asarray(ends, float), owners, report


# ----------------------------------------------------------------------
# Distance calculation
# ----------------------------------------------------------------------
def nearest_distances(points, A, B, chunk=500):
    """Shortest distance from each point to the set of segments A->B.

    points: (N, 2); A, B: (S, 2). The point is projected onto each
    segment and the projection clamped to [0, 1], so the result is the
    perpendicular distance where the foot of the perpendicular lies on
    the segment and the distance to the nearer endpoint otherwise.
    Returns an (N,) array."""
    points = np.asarray(points, float).reshape(-1, 2)
    AB = B - A
    L2 = np.einsum("ij,ij->i", AB, AB)
    safe_L2 = np.where(L2 > 0, L2, 1.0)             # avoid /0 on point-segments

    out = np.empty(len(points))
    for i in range(0, len(points), chunk):
        P = points[i:i + chunk]
        AP = P[:, None, :] - A[None, :, :]                       # (n, S, 2)
        t = np.einsum("nsj,sj->ns", AP, AB) / safe_L2[None, :]
        t = np.clip(np.where(L2[None, :] > 0, t, 0.0), 0.0, 1.0)
        closest = A[None, :, :] + t[:, :, None] * AB[None, :, :]
        d = np.linalg.norm(P[:, None, :] - closest, axis=2)      # (n, S)
        out[i:i + chunk] = d.min(axis=1)
    return out


# ----------------------------------------------------------------------
# Reading the run files
# ----------------------------------------------------------------------
def load_run(csv_path, matched_cols, A, B):
    """Read one run file and return {label: info}, where label is e.g.
    "native" or "native_kalman" (in order of first appearance) and info is
    {"distances": [...], "skipped_unmatched": int, "skipped_missing": int}.

    distances are the shortest distances (meters) from each matched
    point to the route geometry, for every row of that method/kalman
    group that was matched and has both matched coordinates."""
    match_x_col, match_y_col = matched_cols

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []

        required = ["method", "kalman", match_x_col, match_y_col]
        missing = [c for c in required if c not in fieldnames]
        if missing:
            raise ValueError(f"{csv_path} is missing column(s) {missing}. "
                             f"Found columns: {fieldnames}")

        groups = {}
        points = {}
        for row in reader:
            label = _group_label(row["method"], row["kalman"])
            g = groups.setdefault(label, {"distances": [],
                                          "skipped_unmatched": 0,
                                          "skipped_missing": 0})
            pts = points.setdefault(label, [])

            if "matched" in row and not _is_true(row["matched"]):
                g["skipped_unmatched"] += 1
                continue

            mx, my = _to_float(row[match_x_col]), _to_float(row[match_y_col])
            if mx is None or my is None:
                g["skipped_missing"] += 1
                continue

            pts.append((mx, my))

    for label, pts in points.items():
        if pts:
            groups[label]["distances"] = nearest_distances(
                np.asarray(pts), A, B).tolist()

    return groups


def summarize(distances):
    if not len(distances):
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
    net_file = NET_FILE
    if net_file is None:
        sumocfg_abs = _resolve(SUMOCFG)
        if not os.path.exists(sumocfg_abs):
            raise FileNotFoundError(f"SUMO config not found: {sumocfg_abs}")
        net_file = parse_sumocfg_for_netfile(sumocfg_abs)
    elif not os.path.isabs(net_file):
        net_file = _resolve(net_file)

    route_file_abs = _resolve(ROUTE_FILE)
    if not os.path.exists(route_file_abs):
        raise FileNotFoundError(f"Route file not found: {route_file_abs}")
    if not os.path.exists(net_file):
        raise FileNotFoundError(f"Net file not found: {net_file}")

    print(f"[INFO] Net file:    {net_file}")
    print(f"[INFO] Route file:  {route_file_abs}")

    route_edges = load_ground_truth_route(route_file_abs, ROUTE_ID)
    print(f"[INFO] Ground-truth route: {len(route_edges)} edges "
          f"({len(set(route_edges))} unique)")

    print("[INFO] Reading network geometry...")
    net = load_network(net_file)
    A, B, _, report = build_route_segments(
        net, route_edges, GEOMETRY, INCLUDE_JUNCTION_CONNECTORS)

    if report["missing_edges"]:
        shown = ", ".join(report["missing_edges"][:10])
        more = " ..." if len(report["missing_edges"]) > 10 else ""
        print(f"[WARN] {len(report['missing_edges'])} route edge(s) not found "
              f"in the network and skipped: {shown}{more}")
    if INCLUDE_JUNCTION_CONNECTORS and report["no_connector_pairs"]:
        print(f"[WARN] No junction connector found for "
              f"{report['no_connector_pairs']} consecutive edge pair(s).")

    xs = np.concatenate([A[:, 0], B[:, 0]])
    ys = np.concatenate([A[:, 1], B[:, 1]])
    print(f"[INFO] Route geometry ({GEOMETRY}): {len(A)} segments from "
          f"{report['n_polylines']} polylines"
          + (f", {report['n_connector_lanes']} junction connector lane(s)"
             if INCLUDE_JUNCTION_CONNECTORS else ""))
    print(f"[INFO] Route bounding box: x {xs.min():.1f}..{xs.max():.1f}, "
          f"y {ys.min():.1f}..{ys.max():.1f}")

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
            groups = load_run(abs_path, MATCHED_COLUMNS, A, B)
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

    # Sanity check on coordinate frames: distances of this size mean the
    # matched points and the network aren't in the same x/y frame.
    overall_max = max((s["max"] for _, _, s in per_run_rows if s), default=0.0)
    if overall_max > 1000:
        print(f"\n[WARN] Largest distance is {overall_max:.0f} m. If that "
              "isn't plausible, check that the net file matches the one the "
              "runs were recorded on (same x/y frame).")

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