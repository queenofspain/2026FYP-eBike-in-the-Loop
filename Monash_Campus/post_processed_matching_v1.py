# """
# Post-processing GPS-to-road map matcher for SUMO.

# Instead of pulling live GPS data from a phone/Flask server and driving a
# vehicle in real time (as the original live-tracking script did), this script
# takes a CSV of already-recorded GPS points (sim_time, lat, lon) and runs each
# point through SUMO's native map-matching routine
# (traci.simulation.convertRoad -- the same algorithm used internally by
# moveToXY / the live script) to snap it onto the road network.

# For every input point it writes out:
#   - the original sim_time, lat, lon
#   - whether a match was found
#   - the matched edge ID, lane index, and position-along-lane (these are what
#     you need for Newson-Krumm style error metrics)
#   - the matched point's coordinates, both in SUMO x/y and back-projected
#     lat/lon (i.e. the point on the road nearest the raw GPS fix)
#   - the straight-line distance between the raw GPS point and the matched
#     point ("matching error" in meters), which is often useful context

# No real-time stepping, vehicle spawning, or speed/heading control is needed
# here since we're only doing geometric map matching on a static network.
# """

# import argparse
# import csv
# import os
# import sys
# import math

# # ---------- DEFAULT SETTINGS (overridable via CLI args) ----------
# # Defaults are resolved relative to this script's own location, not the
# # current working directory -- so the script works the same whether you run
# # it from this folder, a parent folder, or via an IDE "Run" button.
# SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# DEFAULT_SUMO_CFG = os.path.join(SCRIPT_DIR, "2026-03-11-17-20-46", "osm.sumocfg")
# DEFAULT_INPUT_CSV = os.path.join(SCRIPT_DIR, "Data/gps_data_processed.csv")
# DEFAULT_OUTPUT_CSV = os.path.join(SCRIPT_DIR, "Data/matched_output.csv")

# DEFAULT_LAT_COL = "lat"
# DEFAULT_LON_COL = "lon"

# # Same matching radius the live script used for moveToXY / convertRoad
# MATCH_THRESHOLD = 100.0
# # -------------------------------------------------------------------

# if "SUMO_HOME" not in os.environ:
#     raise EnvironmentError("SUMO_HOME is not set. Set it before running this script.")

# SUMO_HOME = os.environ["SUMO_HOME"]
# TOOLS = os.path.join(SUMO_HOME, "tools")
# if TOOLS not in sys.path:
#     sys.path.append(TOOLS)

# import traci  # noqa: E402
# import sumolib  # noqa: E402


# def parse_sumocfg_for_netfile(sumocfg_path: str) -> str:
#     import xml.etree.ElementTree as ET

#     tree = ET.parse(sumocfg_path)
#     root = tree.getroot()

#     input_tag = root.find("input")
#     if input_tag is None:
#         raise ValueError(f"No <input> section found in {sumocfg_path}")

#     net_tag = input_tag.find("net-file")
#     if net_tag is None:
#         raise ValueError(f"No <net-file> entry found in {sumocfg_path}")

#     net_value = net_tag.get("value")
#     if not net_value:
#         raise ValueError(f"net-file has no value in {sumocfg_path}")

#     base_dir = os.path.dirname(os.path.abspath(sumocfg_path))
#     return os.path.abspath(os.path.join(base_dir, net_value))


# def load_gps_rows(csv_path: str, lat_col: str, lon_col: str):
#     """Read a generic GPS CSV and return a list of dicts with the raw row
#     plus parsed time/lat/lon. Extra columns in the CSV are preserved and
#     passed through to the output file untouched."""
#     rows = []
#     with open(csv_path, "r", newline="") as f:
#         reader = csv.DictReader(f)
#         if reader.fieldnames is None:
#             raise ValueError(f"Could not read header row from {csv_path}")

#         missing = [c for c in (lat_col, lon_col) if c not in reader.fieldnames]
#         if missing:
#             raise ValueError(
#                 f"Input CSV is missing expected column(s) {missing}. "
#                 f"Found columns: {reader.fieldnames}. "
#                 f"Use --lat-col/--lon-col to point at the right columns."
#             )

#         for raw_row in reader:
#             try:
#                 lat = float(raw_row[lat_col])
#                 lon = float(raw_row[lon_col])
#             except (TypeError, ValueError):
#                 print(f"[WARN] Skipping unparsable row: {raw_row}")
#                 continue

#             rows.append({"raw": raw_row, "lat": lat, "lon": lon})

#     return rows


# def match_point(net, lon: float, lat: float, match_threshold: float):
#     """
#     Run one GPS point through SUMO's native map-matching (convertRoad),
#     then compute the actual snapped point on the matched lane using the
#     static network geometry (sumolib), and project that back to lat/lon.

#     Returns a dict with match results. matched=False if no edge was found
#     within match_threshold.
#     """
#     result = {
#         "matched": False,
#         "edge_id": "",
#         "lane_index": "",
#         "lane_pos": "",
#         "raw_x": "",
#         "raw_y": "",
#         "matched_x": "",
#         "matched_y": "",
#         "matched_lat": "",
#         "matched_lon": "",
#         "match_error_m": "",
#     }

#     try:
#         raw_x, raw_y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
#     except traci.TraCIException as e:
#         print(f"[WARN] convertGeo failed for ({lat}, {lon}): {e}")
#         return result

#     result["raw_x"] = raw_x
#     result["raw_y"] = raw_y

#     try:
#         edge_id, lane_pos, lane_index = traci.simulation.convertRoad(
#             lon, lat, isGeo=True
#         )
#     except traci.TraCIException as e:
#         print(f"[WARN] convertRoad failed for ({lat}, {lon}): {e}")
#         return result

#     if not edge_id or edge_id.startswith(":"):
#         # No edge found, or only an internal junction edge matched
#         return result

#     try:
#         lane = net.getLane(f"{edge_id}_{lane_index}")
#         shape = lane.getShape()
#         matched_x, matched_y = sumolib.geomhelper.positionAtShapeOffset(shape, lane_pos)
#     except Exception as e:
#         print(f"[WARN] Could not compute snapped geometry for edge {edge_id}: {e}")
#         # We still have a matched edge even if we can't get exact XY
#         result["matched"] = True
#         result["edge_id"] = edge_id
#         result["lane_index"] = lane_index
#         result["lane_pos"] = lane_pos
#         return result

#     matched_lon, matched_lat = net.convertXY2LonLat(matched_x, matched_y)

#     match_error_m = math.hypot(matched_x - raw_x, matched_y - raw_y)

#     result.update(
#         {
#             "matched": True,
#             "edge_id": edge_id,
#             "lane_index": lane_index,
#             "lane_pos": lane_pos,
#             "matched_x": matched_x,
#             "matched_y": matched_y,
#             "matched_lat": matched_lat,
#             "matched_lon": matched_lon,
#             "match_error_m": match_error_m,
#         }
#     )
#     return result


# def main():
#     parser = argparse.ArgumentParser(
#         description="Map-match a CSV of recorded GPS points onto a SUMO network."
#     )
#     parser.add_argument("--sumocfg", default=DEFAULT_SUMO_CFG, help="Path to .sumocfg file")
#     parser.add_argument("--input", default=DEFAULT_INPUT_CSV, help="Input GPS CSV path")
#     parser.add_argument("--output", default=DEFAULT_OUTPUT_CSV, help="Output CSV path")
#     parser.add_argument("--lat-col", default=DEFAULT_LAT_COL, help="Name of the latitude column")
#     parser.add_argument("--lon-col", default=DEFAULT_LON_COL, help="Name of the longitude column")
#     parser.add_argument(
#         "--match-threshold",
#         type=float,
#         default=MATCH_THRESHOLD,
#         help="Max search radius (m) for map matching",
#     )
#     args = parser.parse_args()

#     sumocfg_abs = os.path.abspath(args.sumocfg)
#     if not os.path.exists(sumocfg_abs):
#         raise FileNotFoundError(f"SUMO config not found: {sumocfg_abs}")

#     net_file = parse_sumocfg_for_netfile(sumocfg_abs)

#     print(f"[INFO] SUMO config: {sumocfg_abs}")
#     print(f"[INFO] Net file:    {net_file}")
#     print(f"[INFO] Input CSV:   {args.input}")
#     print(f"[INFO] Output CSV:  {args.output}")

#     print("[INFO] Loading network geometry with sumolib...")
#     net = sumolib.net.readNet(net_file)

#     print("[INFO] Loading GPS points...")
#     gps_rows = load_gps_rows(args.input, args.lat_col, args.lon_col)
#     print(f"[INFO] Loaded {len(gps_rows)} GPS points.")

#     print("[INFO] Starting headless SUMO for native map matching...")
#     traci.start(["sumo", "-c", sumocfg_abs, "--start", "--no-step-log", "true"])

#     output_rows = []
#     matched_count = 0

#     try:
#         for i, row in enumerate(gps_rows):
#             match = match_point(net, row["lon"], row["lat"], args.match_threshold)
#             if match["matched"]:
#                 matched_count += 1

#             out_row = dict(row["raw"])  # preserve any extra original columns
#             out_row.update(
#                 {
#                     args.lat_col: row["lat"],
#                     args.lon_col: row["lon"],
#                     "matched": match["matched"],
#                     "edge_id": match["edge_id"],
#                     "lane_index": match["lane_index"],
#                     "lane_pos": match["lane_pos"],
#                     "raw_x": match["raw_x"],
#                     "raw_y": match["raw_y"],
#                     "matched_x": match["matched_x"],
#                     "matched_y": match["matched_y"],
#                     "matched_lat": match["matched_lat"],
#                     "matched_lon": match["matched_lon"],
#                     "match_error_m": match["match_error_m"],
#                 }
#             )
#             output_rows.append(out_row)

#             if (i + 1) % 50 == 0 or (i + 1) == len(gps_rows):
#                 print(f"[INFO] Processed {i + 1}/{len(gps_rows)} points "
#                       f"({matched_count} matched so far)")
#     finally:
#         traci.close()

#     if not output_rows:
#         print("[WARN] No output rows produced; nothing written.")
#         return

#     fieldnames = list(output_rows[0].keys())
#     with open(args.output, "w", newline="") as f:
#         writer = csv.DictWriter(f, fieldnames=fieldnames)
#         writer.writeheader()
#         writer.writerows(output_rows)

#     print(
#         f"[DONE] Wrote {len(output_rows)} rows to {args.output} "
#         f"({matched_count}/{len(output_rows)} points matched to an edge)."
#     )


# if __name__ == "__main__":
#     main()

"""
Post-processing GPS-to-road map matcher for SUMO.

Instead of pulling live GPS data from a phone/Flask server and driving a
vehicle in real time (as the original live-tracking script did), this script
takes a CSV of already-recorded GPS points (lat, lon) and runs each
point through SUMO's native map-matching routine
(traci.simulation.convertRoad -- the same algorithm used internally by
moveToXY / the live script) to snap it onto the road network.

For every input point it writes out:
  - the original lat, lon
  - whether a match was found
  - the matched edge ID, lane index, and position-along-lane (these are what
    you need for Newson-Krumm style error metrics)
  - whether the match landed on a junction/internal edge (is_internal_edge)
  - the matched point's coordinates, both in SUMO x/y and back-projected
    lat/lon (i.e. the point on the road nearest the raw GPS fix)
  - the straight-line distance between the raw GPS point and the matched
    point ("matching error" in meters), which is often useful context
  - if a point didn't match, why (unmatched_reason)

No real-time stepping, vehicle spawning, or speed/heading control is needed
here since we're only doing geometric map matching on a static network.
"""

import argparse
import csv
import os
import sys
import math

# ---------- DEFAULT SETTINGS (overridable via CLI args) ----------
# Defaults are resolved relative to this script's own location, not the
# current working directory -- so the script works the same whether you run
# it from this folder, a parent folder, or via an IDE "Run" button.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_SUMO_CFG = os.path.join(SCRIPT_DIR, "2026-03-11-17-20-46", "osm.sumocfg")
DEFAULT_INPUT_CSV = os.path.join(SCRIPT_DIR, "Data/gps_data_processed.csv")
DEFAULT_OUTPUT_CSV = os.path.join(SCRIPT_DIR, "Data/matched_output.csv")

DEFAULT_LAT_COL = "lat"
DEFAULT_LON_COL = "lon"

# Same matching radius the live script used for moveToXY / convertRoad
MATCH_THRESHOLD = 100.0
# -------------------------------------------------------------------

if "SUMO_HOME" not in os.environ:
    raise EnvironmentError("SUMO_HOME is not set. Set it before running this script.")

SUMO_HOME = os.environ["SUMO_HOME"]
TOOLS = os.path.join(SUMO_HOME, "tools")
if TOOLS not in sys.path:
    sys.path.append(TOOLS)

import traci  # noqa: E402
import sumolib  # noqa: E402


def parse_sumocfg_for_netfile(sumocfg_path: str) -> str:
    import xml.etree.ElementTree as ET

    tree = ET.parse(sumocfg_path)
    root = tree.getroot()

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


def load_gps_rows(csv_path: str, lat_col: str, lon_col: str):
    """Read a generic GPS CSV and return a list of dicts with the raw row
    plus parsed lat/lon. Extra columns in the CSV are preserved and
    passed through to the output file untouched."""
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Could not read header row from {csv_path}")

        missing = [c for c in (lat_col, lon_col) if c not in reader.fieldnames]
        if missing:
            raise ValueError(
                f"Input CSV is missing expected column(s) {missing}. "
                f"Found columns: {reader.fieldnames}. "
                f"Use --lat-col/--lon-col to point at the right columns."
            )

        for raw_row in reader:
            try:
                lat = float(raw_row[lat_col])
                lon = float(raw_row[lon_col])
            except (TypeError, ValueError):
                print(f"[WARN] Skipping unparsable row: {raw_row}")
                continue

            rows.append({"raw": raw_row, "lat": lat, "lon": lon})

    return rows


def match_point(net, lon: float, lat: float, match_threshold: float):
    """
    Run one GPS point through SUMO's native map-matching (convertRoad),
    then compute the actual snapped point on the matched lane using the
    static network geometry (sumolib), and project that back to lat/lon.

    Returns a dict with match results. matched=False if no edge was found
    within match_threshold.
    """
    result = {
        "matched": False,
        "edge_id": "",
        "lane_index": "",
        "lane_pos": "",
        "is_internal_edge": False,
        "raw_x": "",
        "raw_y": "",
        "matched_x": "",
        "matched_y": "",
        "matched_lat": "",
        "matched_lon": "",
        "match_error_m": "",
        "unmatched_reason": "",
    }

    try:
        raw_x, raw_y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
    except traci.TraCIException as e:
        print(f"[WARN] convertGeo failed for ({lat}, {lon}): {e}")
        result["unmatched_reason"] = "convertGeo_failed"
        return result

    result["raw_x"] = raw_x
    result["raw_y"] = raw_y

    try:
        edge_id, lane_pos, lane_index = traci.simulation.convertRoad(
            lon, lat, isGeo=True
        )
    except traci.TraCIException as e:
        print(f"[WARN] convertRoad failed for ({lat}, {lon}): {e}")
        result["unmatched_reason"] = "convertRoad_failed"
        return result

    if not edge_id:
        # No edge found at all within SUMO's internal search radius
        result["unmatched_reason"] = "no_edge_found"
        return result

    # NOTE: edges starting with ':' are internal junction/connector edges
    # (SUMO's auto-generated intersection geometry). These are still valid
    # matches -- a GPS fix taken while turning through an intersection will
    # legitimately snap here. We keep them (flagged via is_internal_edge)
    # rather than discarding them, since dropping them creates artificial
    # gaps right at every turn/crossing.
    result["is_internal_edge"] = edge_id.startswith(":")

    try:
        lane = net.getLane(f"{edge_id}_{lane_index}")
        shape = lane.getShape()
        matched_x, matched_y = sumolib.geomhelper.positionAtShapeOffset(shape, lane_pos)
    except Exception as e:
        print(f"[WARN] Could not compute snapped geometry for edge {edge_id}: {e}")
        # We still have a matched edge even if we can't get exact XY
        result["matched"] = True
        result["edge_id"] = edge_id
        result["lane_index"] = lane_index
        result["lane_pos"] = lane_pos
        result["unmatched_reason"] = "geometry_lookup_failed"
        return result

    matched_lon, matched_lat = net.convertXY2LonLat(matched_x, matched_y)

    match_error_m = math.hypot(matched_x - raw_x, matched_y - raw_y)

    result.update(
        {
            "matched": True,
            "edge_id": edge_id,
            "lane_index": lane_index,
            "lane_pos": lane_pos,
            "matched_x": matched_x,
            "matched_y": matched_y,
            "matched_lat": matched_lat,
            "matched_lon": matched_lon,
            "match_error_m": match_error_m,
        }
    )
    return result


def main():
    parser = argparse.ArgumentParser(
        description="Map-match a CSV of recorded GPS points onto a SUMO network."
    )
    parser.add_argument("--sumocfg", default=DEFAULT_SUMO_CFG, help="Path to .sumocfg file")
    parser.add_argument("--input", default=DEFAULT_INPUT_CSV, help="Input GPS CSV path")
    parser.add_argument("--output", default=DEFAULT_OUTPUT_CSV, help="Output CSV path")
    parser.add_argument("--lat-col", default=DEFAULT_LAT_COL, help="Name of the latitude column")
    parser.add_argument("--lon-col", default=DEFAULT_LON_COL, help="Name of the longitude column")
    parser.add_argument(
        "--match-threshold",
        type=float,
        default=MATCH_THRESHOLD,
        help="Max search radius (m) for map matching",
    )
    args = parser.parse_args()

    sumocfg_abs = os.path.abspath(args.sumocfg)
    if not os.path.exists(sumocfg_abs):
        raise FileNotFoundError(f"SUMO config not found: {sumocfg_abs}")

    net_file = parse_sumocfg_for_netfile(sumocfg_abs)

    print(f"[INFO] SUMO config: {sumocfg_abs}")
    print(f"[INFO] Net file:    {net_file}")
    print(f"[INFO] Input CSV:   {args.input}")
    print(f"[INFO] Output CSV:  {args.output}")

    print("[INFO] Loading network geometry with sumolib...")
    # withInternal=True is required so junction-internal edges (the short
    # connector edges inside intersections, IDs starting with ':') have
    # usable lane geometry -- otherwise every match that lands on one during
    # a turn silently fails to produce coordinates.
    net = sumolib.net.readNet(net_file, withInternal=True)

    print("[INFO] Loading GPS points...")
    gps_rows = load_gps_rows(args.input, args.lat_col, args.lon_col)
    print(f"[INFO] Loaded {len(gps_rows)} GPS points.")

    print("[INFO] Starting headless SUMO for native map matching...")
    traci.start(["sumo", "-c", sumocfg_abs, "--start", "--no-step-log", "true"])

    output_rows = []
    matched_count = 0

    try:
        for i, row in enumerate(gps_rows):
            match = match_point(net, row["lon"], row["lat"], args.match_threshold)
            if match["matched"]:
                matched_count += 1

            out_row = dict(row["raw"])  # preserve any extra original columns
            out_row.update(
                {
                    args.lat_col: row["lat"],
                    args.lon_col: row["lon"],
                    "matched": match["matched"],
                    "edge_id": match["edge_id"],
                    "lane_index": match["lane_index"],
                    "lane_pos": match["lane_pos"],
                    "is_internal_edge": match["is_internal_edge"],
                    "raw_x": match["raw_x"],
                    "raw_y": match["raw_y"],
                    "matched_x": match["matched_x"],
                    "matched_y": match["matched_y"],
                    "matched_lat": match["matched_lat"],
                    "matched_lon": match["matched_lon"],
                    "match_error_m": match["match_error_m"],
                    "unmatched_reason": match["unmatched_reason"],
                }
            )
            output_rows.append(out_row)

            if (i + 1) % 50 == 0 or (i + 1) == len(gps_rows):
                print(f"[INFO] Processed {i + 1}/{len(gps_rows)} points "
                      f"({matched_count} matched so far)")
    finally:
        traci.close()

    if not output_rows:
        print("[WARN] No output rows produced; nothing written.")
        return

    internal_count = sum(1 for r in output_rows if r["is_internal_edge"])
    reason_counts = {}
    for r in output_rows:
        if not r["matched"]:
            reason = r["unmatched_reason"] or "unknown"
            reason_counts[reason] = reason_counts.get(reason, 0) + 1

    fieldnames = list(output_rows[0].keys())
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(output_rows)

    print(
        f"[DONE] Wrote {len(output_rows)} rows to {args.output} "
        f"({matched_count}/{len(output_rows)} points matched to an edge, "
        f"{internal_count} of which landed on a junction/internal edge)."
    )
    if reason_counts:
        print("[INFO] Breakdown of unmatched points:")
        for reason, count in sorted(reason_counts.items(), key=lambda kv: -kv[1]):
            print(f"         {reason}: {count}")


if __name__ == "__main__":
    main()