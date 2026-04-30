import os
import sys
import time
import requests
import xml.etree.ElementTree as ET
import csv

# ---------- USER SETTINGS ----------

SUMO_CFG = r"2026-03-11-17-20-46/osm.sumocfg"

FLASK_LATEST_URL = "http://localhost:5000/latest"

VEHICLE_ID_KEEP0 = "ebike_keep0"
VEHICLE_ID_KEEP1 = "ebike_keep1"
VEHICLE_TYPE_ID = "bike_live"

ROUTE_ID = "block_route"

POLL_INTERVAL = 1.0
STALE_DATA_SECONDS = 5.0
SUMO_STEP_LENGTH = 1.0
MATCH_THRESHOLD = 100.0

LOG_FILE = "position_log.csv"

if "SUMO_HOME" not in os.environ:
    raise EnvironmentError("SUMO_HOME is not set.")

SUMO_HOME = os.environ["SUMO_HOME"]
TOOLS = os.path.join(SUMO_HOME, "tools")
if TOOLS not in sys.path:
    sys.path.append(TOOLS)

import traci


def parse_sumocfg_for_netfile(sumocfg_path: str) -> str:
    tree = ET.parse(sumocfg_path)
    root = tree.getroot()
    input_tag = root.find("input")
    net_tag = input_tag.find("net-file")
    net_value = net_tag.get("value")
    base_dir = os.path.dirname(os.path.abspath(sumocfg_path))
    return os.path.abspath(os.path.join(base_dir, net_value))


def get_latest_phone_data(url: str):
    try:
        r = requests.get(url, timeout=2)
        r.raise_for_status()
        data = r.json()
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def phone_data_is_valid(data) -> bool:
    return bool(data) and data.get("lat") is not None and data.get("lon") is not None


def phone_data_is_fresh(data, stale_seconds: float) -> bool:
    received = data.get("server_received_at")
    if not received:
        return False
    from datetime import datetime
    t = received.replace("Z", "+00:00")
    dt = datetime.fromisoformat(t)
    now = datetime.now(dt.tzinfo) if dt.tzinfo else datetime.now()
    return (now - dt).total_seconds() <= stale_seconds


def ensure_vehicle_type():
    if VEHICLE_TYPE_ID not in traci.vehicletype.getIDList():
        try:
            traci.vehicletype.copy("DEFAULT_BIKETYPE", VEHICLE_TYPE_ID)
        except traci.TraCIException:
            traci.vehicletype.copy("DEFAULT_VEHTYPE", VEHICLE_TYPE_ID)
            traci.vehicletype.setVehicleClass(VEHICLE_TYPE_ID, "bicycle")


def vehicle_exists(vehID):
    return vehID in traci.vehicle.getIDList()


def spawn_vehicle_if_missing(vehID):
    if vehicle_exists(vehID):
        return True

    if ROUTE_ID not in traci.route.getIDList():
        print(f"[ERROR] Route '{ROUTE_ID}' not loaded.")
        return False

    traci.vehicle.add(
        vehID=vehID,
        routeID=ROUTE_ID,
        typeID=VEHICLE_TYPE_ID,
        depart="now",
        departLane="best",
        departPos="base",
        departSpeed="0"
    )

    traci.vehicle.setSpeedMode(vehID, 0)
    traci.vehicle.setSpeed(vehID, 0.0)

    print(f"[INFO] Spawned {vehID}")
    return True


def move_bike(vehID, keep_route, x, y, edge_id, lane_index, angle, speed):
    try:
        traci.vehicle.moveToXY(
            vehID=vehID,
            edgeID=edge_id,
            laneIndex=lane_index,
            x=x,
            y=y,
            angle=angle,
            keepRoute=keep_route,
            matchThreshold=MATCH_THRESHOLD
        )
        traci.vehicle.setSpeed(vehID, max(0.0, float(speed or 0.0)))
    except traci.TraCIException:
        pass


def move_vehicles_to_phone_position(lat, lon, speed_mps, course_deg_raw, log_writer):

    if not spawn_vehicle_if_missing(VEHICLE_ID_KEEP0):
        return
    if not spawn_vehicle_if_missing(VEHICLE_ID_KEEP1):
        return

    try:
        x, y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
        edge_id, pos, lane_index = traci.simulation.convertRoad(lon, lat, isGeo=True)
    except traci.TraCIException:
        return

    angle = traci.constants.INVALID_DOUBLE_VALUE
    try:
        if course_deg_raw is not None:
            angle = float(course_deg_raw) % 360.0
    except Exception:
        pass

    # Move both bikes
    move_bike(VEHICLE_ID_KEEP0, 0, x, y, edge_id, lane_index, angle, speed_mps)
    move_bike(VEHICLE_ID_KEEP1, 1, x, y, edge_id, lane_index, angle, speed_mps)

    sim_time = traci.simulation.getTime()

    try:
        k0_x, k0_y = traci.vehicle.getPosition(VEHICLE_ID_KEEP0)
    except traci.TraCIException:
        k0_x, k0_y = None, None

    try:
        k1_x, k1_y = traci.vehicle.getPosition(VEHICLE_ID_KEEP1)
    except traci.TraCIException:
        k1_x, k1_y = None, None

    log_writer.writerow([
        sim_time,
        lat,
        lon,
        x, y,
        k0_x, k0_y,
        k1_x, k1_y
    ])

    print(f"[OK] GPS ({x:.1f},{y:.1f}) | keep0=({k0_x:.1f},{k0_y:.1f}) | keep1=({k1_x:.1f},{k1_y:.1f})")


def main():
    traci.start([
        "sumo-gui",
        "-c", SUMO_CFG,
        "--step-length", str(SUMO_STEP_LENGTH),
        "--start",
        "--end", "1000000"
    ])

    ensure_vehicle_type()

    log_file = open(LOG_FILE, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "sim_time",
        "phone_lat",
        "phone_lon",
        "gps_x",
        "gps_y",
        "keep0_sumo_x",
        "keep0_sumo_y",
        "keep1_sumo_x",
        "keep1_sumo_y"
    ])

    last_seen_timestamp = None
    last_poll_time = 0.0

    try:
        while True:
            step_start = time.time()
            traci.simulationStep()

            now = time.time()
            if now - last_poll_time >= POLL_INTERVAL:
                last_poll_time = now
                data = get_latest_phone_data(FLASK_LATEST_URL)

                if phone_data_is_valid(data) and phone_data_is_fresh(data, STALE_DATA_SECONDS):
                    phone_timestamp = data.get("phone_timestamp")

                    if phone_timestamp != last_seen_timestamp:
                        last_seen_timestamp = phone_timestamp

                        lat = float(data["lat"])
                        lon = float(data["lon"])
                        speed_mps = float(data.get("speed_mps", 0.0) or 0.0)
                        course_deg = data.get("course_deg")

                        move_vehicles_to_phone_position(
                            lat, lon, speed_mps, course_deg, log_writer
                        )

            elapsed = time.time() - step_start
            time.sleep(max(0.0, SUMO_STEP_LENGTH - elapsed))

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        log_file.close()
        traci.close()


if __name__ == "__main__":
    main()