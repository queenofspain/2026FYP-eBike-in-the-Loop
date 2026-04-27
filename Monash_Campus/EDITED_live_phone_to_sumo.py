import os
import sys
import time
import requests
import xml.etree.ElementTree as ET
import csv

# ---------- USER SETTINGS ----------

# ---------- Monash Clayton ---------
SUMO_CFG = r"2026-03-11-17-20-46/osm.sumocfg"

# ---------- Jordan's house ---------
# SUMO_CFG = r"2026-04-25-22-15-28/osm.sumocfg"

FLASK_LATEST_URL = "http://localhost:5000/latest"

VEHICLE_ID = "ebike0"
VEHICLE_TYPE_ID = "bike_live"

POLL_INTERVAL = 1.0
STALE_DATA_SECONDS = 5.0
SUMO_STEP_LENGTH = 1.0
MATCH_THRESHOLD = 100.0
# ----------------------------------

# ---------- Data Files ----------
LOG_FILE = "position_log.csv"
# ----------------------------------

if "SUMO_HOME" not in os.environ:
    raise EnvironmentError("SUMO_HOME is not set. Set it before running this script.")

SUMO_HOME = os.environ["SUMO_HOME"]
TOOLS = os.path.join(SUMO_HOME, "tools")
if TOOLS not in sys.path:
    sys.path.append(TOOLS)

import traci


def parse_sumocfg_for_netfile(sumocfg_path: str) -> str:
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


def get_latest_phone_data(url: str):
    try:
        r = requests.get(url, timeout=2)
        r.raise_for_status()
        data = r.json()
        return data if isinstance(data, dict) else None
    except Exception as e:
        print(f"[WARN] Could not get latest data from Flask: {e}")
        return None


def phone_data_is_valid(data) -> bool:
    return bool(data) and data.get("lat") is not None and data.get("lon") is not None


def phone_data_is_fresh(data, stale_seconds: float) -> bool:
    received = data.get("server_received_at")
    if not received:
        return False

    try:
        from datetime import datetime
        t = received.replace("Z", "+00:00")
        dt = datetime.fromisoformat(t)
        now = datetime.now(dt.tzinfo) if dt.tzinfo else datetime.now()
        return (now - dt).total_seconds() <= stale_seconds
    except Exception:
        return False


def ensure_vehicle_type():
    try:
        existing = set(traci.vehicletype.getIDList())
        if VEHICLE_TYPE_ID not in existing:
            try:
                traci.vehicletype.copy("DEFAULT_BIKETYPE", VEHICLE_TYPE_ID)
                traci.vehicletype.setMaxSpeed(VEHICLE_TYPE_ID, 12.0)
            except traci.TraCIException:
                traci.vehicletype.copy("DEFAULT_VEHTYPE", VEHICLE_TYPE_ID)
                traci.vehicletype.setVehicleClass(VEHICLE_TYPE_ID, "bicycle")
    except Exception as e:
        print(f"[WARN] Could not ensure vehicle type: {e}")


def vehicle_exists() -> bool:
    try:
        return VEHICLE_ID in traci.vehicle.getIDList()
    except Exception:
        return False


def spawn_vehicle_if_missing():
    if vehicle_exists():
        return True

    # route_id = f"route_{VEHICLE_ID}"

    # edge_ids = traci.edge.getIDList()
    # usable_edges = [e for e in edge_ids if not e.startswith(":")]

    # if not usable_edges:
    #     print("[ERROR] No usable edges found in network.")
    #     return False

    # first_edge = usable_edges[0]

    try:
        route_id = "block_route"
        if route_id not in traci.route.getIDList():
            # traci.route.add(route_id, [first_edge])
            print(f"[ERROR] Route '{route_id}' not loaded from .rou.xml")
            return False

        traci.vehicle.add(
            vehID=VEHICLE_ID,
            routeID=route_id,
            typeID=VEHICLE_TYPE_ID,
            depart="now",
            departLane="best",
            departPos="base",
            departSpeed="0"
        )

        traci.vehicle.setSpeedMode(VEHICLE_ID, 0)
        traci.vehicle.setSpeed(VEHICLE_ID, 0.0)

        print(f"[INFO] Spawned {VEHICLE_ID} on route {route_id}")
        return True

    except traci.TraCIException as e:
        print(f"[WARN] Could not spawn {VEHICLE_ID}: {e}")
        return False


def parse_course_deg(course_deg_raw):
    """
    Return a valid angle in degrees, or INVALID_DOUBLE_VALUE if unusable.
    """
    try:
        if course_deg_raw is None:
            return traci.constants.INVALID_DOUBLE_VALUE

        course_deg = float(course_deg_raw)

        if course_deg < 0:
            return traci.constants.INVALID_DOUBLE_VALUE

        # Normalize to 0..360
        course_deg = course_deg % 360.0
        return course_deg
    except Exception:
        return traci.constants.INVALID_DOUBLE_VALUE


def move_vehicle_to_phone_position(lat: float, lon: float, speed_mps: float | None, course_deg_raw, log_writer):
    """
    Convert GPS to SUMO coordinates and move the controlled bike.
    Respawns vehicle if it disappeared.
    Uses phone course as the vehicle angle when available.
    Prints both phone and SUMO speed/course.
    """
    if not spawn_vehicle_if_missing():
        return

    try:
        x, y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
        edge_id, pos, lane_index = traci.simulation.convertRoad(lon, lat, isGeo=True)
    except traci.TraCIException as e:
        print(f"[WARN] Geo conversion failed: {e}")
        return

    angle_to_use = parse_course_deg(course_deg_raw)

    try:
        traci.vehicle.moveToXY(
            vehID=VEHICLE_ID,
            edgeID=edge_id,
            laneIndex=lane_index,
            x=x,
            y=y,
            angle=angle_to_use,
            keepRoute=1,
            matchThreshold=MATCH_THRESHOLD
        )
    except traci.TraCIException as e:
        print(f"[WARN] moveToXY failed: {e}")
        return

    try:
        traci.vehicle.setSpeed(VEHICLE_ID, max(0.0, float(speed_mps or 0.0)))
    except traci.TraCIException:
        pass

    # --- Get SUMO internal position ---
    try:
        sumo_x, sumo_y = traci.vehicle.getPosition(VEHICLE_ID)
    except traci.TraCIException:
        return

    # Log positional data
    sim_time = traci.simulation.getTime()

    log_writer.writerow([
        sim_time,
        lat,
        lon,
        x,          # converted GPS coordinate
        y,
        sumo_x,     # SUMO internal position
        sumo_y
    ])

    # Read back SUMO's current state
    try:
        sumo_speed_mps = traci.vehicle.getSpeed(VEHICLE_ID)
    except traci.TraCIException:
        sumo_speed_mps = None

    try:
        sumo_angle_deg = traci.vehicle.getAngle(VEHICLE_ID)
    except traci.TraCIException:
        sumo_angle_deg = None

    phone_speed_mps = float(speed_mps or 0.0)
    phone_speed_kmh = phone_speed_mps * 3.6

    try:
        phone_course_deg = None if course_deg_raw is None else float(course_deg_raw)
    except Exception:
        phone_course_deg = None

    if sumo_speed_mps is not None:
        sumo_speed_kmh = sumo_speed_mps * 3.6
        sumo_speed_str = f"{sumo_speed_mps:.2f} m/s ({sumo_speed_kmh:.2f} km/h)"
    else:
        sumo_speed_str = "N/A"

    sumo_angle_str = f"{sumo_angle_deg:.1f} deg" if sumo_angle_deg is not None else "N/A"
    phone_course_str = f"{phone_course_deg:.1f} deg" if phone_course_deg is not None else "N/A"

    print(
        f"[OK] {VEHICLE_ID} -> edge={edge_id}, lane={lane_index}, xy=({x:.1f}, {y:.1f}) | "
        f"PHONE speed={phone_speed_mps:.2f} m/s ({phone_speed_kmh:.2f} km/h), "
        f"PHONE course={phone_course_str} | "
        f"SUMO speed={sumo_speed_str}, SUMO angle={sumo_angle_str}"
    )


def main():
    sumocfg_abs = os.path.abspath(SUMO_CFG)
    if not os.path.exists(sumocfg_abs):
        raise FileNotFoundError(f"SUMO config not found: {sumocfg_abs}")

    net_file = parse_sumocfg_for_netfile(sumocfg_abs)

    print(f"[INFO] SUMO config: {sumocfg_abs}")
    print(f"[INFO] Net file:    {net_file}")
    print("[INFO] Starting SUMO...")

    traci.start([
        "sumo-gui",
        "-c", sumocfg_abs,
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
        "sumo_x",
        "sumo_y"
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

                if not phone_data_is_valid(data):
                    print("[WARN] No valid phone data yet")
                elif not phone_data_is_fresh(data, STALE_DATA_SECONDS):
                    print("[WARN] Latest phone data is stale; ignoring")
                else:
                    phone_timestamp = data.get("phone_timestamp")

                    if phone_timestamp != last_seen_timestamp:
                        last_seen_timestamp = phone_timestamp

                        lat = float(data["lat"])
                        lon = float(data["lon"])

                        try:
                            speed_mps = float(data.get("speed_mps", 0.0) or 0.0)
                        except Exception:
                            speed_mps = 0.0

                        course_deg = data.get("course_deg")

                        move_vehicle_to_phone_position(lat, lon, speed_mps, course_deg, log_writer)

            elapsed = time.time() - step_start
            time.sleep(max(0.0, SUMO_STEP_LENGTH - elapsed))

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        try:
            log_file.close()
            traci.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()