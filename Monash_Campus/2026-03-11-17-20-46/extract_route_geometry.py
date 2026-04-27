import xml.etree.ElementTree as ET
import traci
import csv
import os

# ---------------- SETTINGS ----------------
ROUTE_FILE = r"C:\Uni\Final Year Project\Code\Edited Code\Monash_Campus\2026-03-11-17-20-46\around_n1_route.rou.xml"
OUTPUT_FILE = r"C:\Uni\Final Year Project\Code\Edited Code\Monash_Campus\2026-03-11-17-20-46\route_geometry.csv"
SUMO_CFG = r"C:\Uni\Final Year Project\Code\Edited Code\Monash_Campus\2026-03-11-17-20-46\osm.sumocfg"

# ----------------------------------------


def load_edges_from_routefile(route_file):
    """
    Extract ordered edge list from .rou.xml
    """
    tree = ET.parse(route_file)
    root = tree.getroot()

    route = root.find("route")
    if route is None:
        raise ValueError("No <route> tag found in file")

    edges = route.get("edges").split()
    return edges


def extract_route_geometry(edge_list):
    route_points = []

    for edge in edge_list:
        try:
            lane_id = f"{edge}_0"
            shape = traci.lane.getShape(lane_id)

            if not shape:
                continue

            if len(route_points) > 0 and route_points[-1] == shape[0]:
                route_points.extend(shape[1:])
            else:
                route_points.extend(shape)

        except traci.TraCIException:
            print(f"[WARN] Could not read lane shape: {edge}_0")

    return route_points


def save_geometry(points, output_file):
    """
    Save polyline to CSV
    """
    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        writer.writerows(points)


def main():

    # Start SUMO ONLY for geometry access
    traci.start([
        "sumo",
        "-c", SUMO_CFG
    ])

    edges = load_edges_from_routefile(ROUTE_FILE)
    print(f"[INFO] Loaded {len(edges)} edges")

    route_points = extract_route_geometry(edges)
    print(f"[INFO] Extracted {len(route_points)} route points")

    save_geometry(route_points, OUTPUT_FILE)
    print(f"[INFO] Saved to {OUTPUT_FILE}")

    traci.close()


if __name__ == "__main__":
    main()