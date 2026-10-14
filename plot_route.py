"""Plot the route defined in a SUMO route file (.rou.xml) using the geometry
from a SUMO network file (.net.xml). No axes, transparent background.

Usage:
    python plot_sumo_route.py [net_file] [route_file]
"""

import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np

# ----------------------------------------------------------------------
# SETTINGS - edit these to change inputs and how the route looks
# ----------------------------------------------------------------------

NET_FILE = "Monash_Campus/2026-08-25-19-43-30/osm.net.xml"
ROUTE_FILE = "Monash_Campus/2026-08-25-19-43-30/full_campus_v3.rou.xml"
ROUTE_ID = None             # None = first <route> in the file, or e.g. "full_campus_v3"
OUTPUT_FILE = "sumo_route.png"   # .png / .svg / .pdf ; None to skip saving
SHOW_PLOT = True

# If two consecutive edges end/start further apart than this (metres), the
# line is broken there instead of drawing a straight jump across the gap.
GAP_BREAK_M = 100.0

FIGURE_SIZE = (8, 8)
BACKGROUND = "none"         # "none" = transparent; or e.g. "white"
TRANSPARENT = True
DPI = 300

# Any matplotlib Line2D property can go in here.
ROUTE_STYLE = {
    "color": "#000000",
    "linewidth": 3,
    "linestyle": "-",           # "-", "--", "-.", ":", or (0, (5, 2)) for custom dashes
    "alpha": 1.0,
    "solid_capstyle": "round",  # "round", "butt", "projecting"
    "solid_joinstyle": "round",
    # "dash_capstyle": "round",         # used when linestyle is dashed
}

# Optional outline/"halo" drawn underneath the route (set to None to disable)
OUTLINE_STYLE = None

# Optional start / end markers (set either to None to disable)
START_MARKER = {"marker": "o", "color": "#27d427", "markersize": 10, "zorder": 5}
END_MARKER = {"marker": "s", "color": "#CB1515", "markersize": 10, "zorder": 5}
# ----------------------------------------------------------------------


def read_route_edges(route_file, route_id=None):
    """Return the list of edge IDs of the chosen <route> (whitespace-separated)."""
    for _, elem in ET.iterparse(route_file):
        if elem.tag == "route" and (route_id is None or elem.get("id") == route_id):
            edges = (elem.get("edges") or "").split()
            if edges:
                return edges
    raise SystemExit(f"No route{'' if route_id is None else ' ' + repr(route_id)} "
                     f"with edges found in {route_file}")


def parse_shape(shape_str):
    return np.array([[float(v) for v in p.split(",")[:2]] for p in shape_str.split()])


def allows_bike(lane):
    allow, disallow = lane.get("allow"), lane.get("disallow")
    if allow is not None:
        return "bicycle" in allow.split()
    return "bicycle" not in (disallow or "").split()


def read_edge_shapes(net_file, wanted):
    """Map edge ID -> (N,2) array, using the first bicycle-permitted lane's shape."""
    shapes = {}
    for _, elem in ET.iterparse(net_file):
        if elem.tag == "edge":
            eid = elem.get("id")
            if eid in wanted and eid not in shapes:
                lanes = elem.findall("lane")
                if lanes:
                    lane = next((l for l in lanes if allows_bike(l)), lanes[0])
                    shapes[eid] = parse_shape(lane.get("shape"))
            elem.clear()
    return shapes


def build_polyline(edges, shapes):
    """Concatenate edge shapes in route order; NaN row breaks the line at big gaps."""
    pts, missing, gaps = [], [], []
    prev_end = None
    for eid in edges:
        shp = shapes.get(eid)
        if shp is None:
            missing.append(eid)
            continue
        if prev_end is not None:
            d = float(np.hypot(*(shp[0] - prev_end)))
            if d > GAP_BREAK_M:
                gaps.append((eid, d))
                pts.append(np.array([[np.nan, np.nan]]))
        pts.append(shp)
        prev_end = shp[-1]
    return np.vstack(pts), missing, gaps


def main():
    net_file = sys.argv[1] if len(sys.argv) > 1 else NET_FILE
    route_file = sys.argv[2] if len(sys.argv) > 2 else ROUTE_FILE

    edges = read_route_edges(route_file, ROUTE_ID)
    shapes = read_edge_shapes(net_file, set(edges))
    xy, missing, gaps = build_polyline(edges, shapes)

    print(f"{len(edges)} edges in route, {len(shapes)} unique edges found in net")
    if missing:
        print(f"WARNING: {len(missing)} edge(s) not in net file: {sorted(set(missing))}")
    for eid, d in gaps:
        print(f"WARNING: {d:.0f} m gap before edge {eid} (line broken here)")

    x, y = xy[:, 0], xy[:, 1]
    valid = ~np.isnan(x)

    fig, ax = plt.subplots(figsize=FIGURE_SIZE, facecolor=BACKGROUND)
    ax.set_facecolor(BACKGROUND)

    if OUTLINE_STYLE:
        ax.plot(x, y, zorder=1, **OUTLINE_STYLE)
    ax.plot(x, y, zorder=2, **ROUTE_STYLE)

    if START_MARKER:
        ax.plot(x[valid][0], y[valid][0], linestyle="none", **START_MARKER)
    if END_MARKER:
        ax.plot(x[valid][-1], y[valid][-1], linestyle="none", **END_MARKER)

    ax.set_aspect("equal")  # keep the route's true shape
    ax.axis("off")          # no axes, ticks, or frame
    fig.tight_layout(pad=0.5)

    if OUTPUT_FILE:
        fig.savefig(OUTPUT_FILE, dpi=DPI, transparent=TRANSPARENT,
                    facecolor=fig.get_facecolor(), bbox_inches="tight")
        print(f"Saved {OUTPUT_FILE}")
    if SHOW_PLOT:
        plt.show()


if __name__ == "__main__":
    main()