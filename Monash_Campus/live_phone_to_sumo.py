import os
import sys
import time
import requests
import xml.etree.ElementTree as ET
from datetime import datetime

# ---------- USER SETTINGS ----------
SUMO_CFG = r"2026-03-11-17-20-46/osm.sumocfg"
FLASK_LATEST_URL = "http://localhost:5000/latest"
FLASK_FEEDBACK_URL = "http://localhost:5000/feedback/update"

VEHICLE_ID = "ebike0"
VEHICLE_TYPE_ID = "bike_live"

POLL_INTERVAL = 1.0
STALE_DATA_SECONDS = 5.0
SUMO_STEP_LENGTH = 1.0
MATCH_THRESHOLD = 100.0
# ----------------------------------

if "SUMO_HOME" not in os.environ:
    raise EnvironmentError("SUMO_HOME is not set. Set it before running this script.")

SUMO_HOME = os.environ["SUMO_HOME"]
TOOLS = os.path.join(SUMO_HOME, "tools")
if TOOLS not in sys.path:
    sys.path.append(TOOLS)

import traci


def post_sumo_feedback(feedback: dict):
    try:
        requests.post(FLASK_FEEDBACK_URL, json=feedback, timeout=1)
    except Exception as e:
        print(f"[WARN] Could not post SUMO feedback to Flask: {e}")


def phone_data_age_seconds(data) -> float | None:
    received = data.get("server_received_at") if data else None
    if not received:
        return None

    try:
        t = received.replace("Z", "+00:00")
        dt = datetime.fromisoformat(t)
        now = datetime.now(dt.tzinfo) if dt.tzinfo else datetime.now()
        return max(0.0, (now - dt).total_seconds())
    except Exception:
        return None


def make_status_feedback(level: str, message: str, matched: bool = False, phone_data=None):
    return {
        "vehicle_id": VEHICLE_ID,
        "sim_time": safe_traci_call(lambda: traci.simulation.getTime()),
        "matched": matched,
        "phone_data_age_s": phone_data_age_seconds(phone_data),
        "feedback_level": level,
        "feedback_message": message,
        "bridge_timestamp": datetime.now().isoformat()
    }


def safe_traci_call(fn, default=None):
    try:
        return fn()
    except Exception:
        return default


def feedback_level_for_state(speed_mps, allowed_speed_mps, leader_gap_m, phone_age_s, accuracy_m):
    if phone_age_s is not None and phone_age_s > STALE_DATA_SECONDS:
        return "danger", "Phone GPS data is stale"

    if accuracy_m is not None and accuracy_m > 10:
        return "warn", "GPS accuracy is poor"

    if leader_gap_m is not None and leader_gap_m < 5:
        return "danger", "Vehicle very close ahead"

    if leader_gap_m is not None and leader_gap_m < 10:
        return "warn", "Traffic ahead"

    if allowed_speed_mps and speed_mps and speed_mps > allowed_speed_mps:
        return "warn", "Above SUMO lane speed"

    return "ok", "Riding normally"


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

    route_id = f"route_{VEHICLE_ID}"

    edge_ids = traci.edge.getIDList()
    usable_edges = [e for e in edge_ids if not e.startswith(":")]

    if not usable_edges:
        print("[ERROR] No usable edges found in network.")
        return False

    first_edge = usable_edges[0]

    try:
        if route_id not in traci.route.getIDList():
            traci.route.add(route_id, [first_edge])

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

        print(f"[INFO] Spawned {VEHICLE_ID} on edge {first_edge}")
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


def move_vehicle_to_phone_position(lat: float, lon: float, speed_mps: float | None, course_deg_raw, phone_data=None):
    """
    Convert GPS to SUMO coordinates and move the controlled bike.
    Respawns vehicle if it disappeared.
    Uses phone course as the vehicle angle when available.
    Prints both phone and SUMO speed/course.
    """
    if not spawn_vehicle_if_missing():
        return make_status_feedback("danger", "Could not spawn rider in SUMO", False, phone_data)

    try:
        x, y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
        edge_id, pos, lane_index = traci.simulation.convertRoad(lon, lat, isGeo=True)
    except traci.TraCIException as e:
        print(f"[WARN] Geo conversion failed: {e}")
        return make_status_feedback("danger", "GPS position could not be converted to SUMO", False, phone_data)

    angle_to_use = parse_course_deg(course_deg_raw)

    try:
        traci.vehicle.moveToXY(
            vehID=VEHICLE_ID,
            edgeID=edge_id,
            laneIndex=lane_index,
            x=x,
            y=y,
            angle=angle_to_use,
            keepRoute=0,
            matchThreshold=MATCH_THRESHOLD
        )
    except traci.TraCIException as e:
        print(f"[WARN] moveToXY failed: {e}")
        return make_status_feedback("danger", "GPS position did not match the SUMO map", False, phone_data)

    try:
        traci.vehicle.setSpeed(VEHICLE_ID, max(0.0, float(speed_mps or 0.0)))
    except traci.TraCIException:
        pass

    # Read back SUMO's current state
    try:
        sumo_speed_mps = traci.vehicle.getSpeed(VEHICLE_ID)
    except traci.TraCIException:
        sumo_speed_mps = None

    try:
        sumo_angle_deg = traci.vehicle.getAngle(VEHICLE_ID)
    except traci.TraCIException:
        sumo_angle_deg = None

    road_id = safe_traci_call(lambda: traci.vehicle.getRoadID(VEHICLE_ID), edge_id)
    lane_id = safe_traci_call(lambda: traci.vehicle.getLaneID(VEHICLE_ID))
    lane_position_m = safe_traci_call(lambda: traci.vehicle.getLanePosition(VEHICLE_ID))
    allowed_speed_mps = safe_traci_call(lambda: traci.vehicle.getAllowedSpeed(VEHICLE_ID))
    acceleration_mps2 = safe_traci_call(lambda: traci.vehicle.getAcceleration(VEHICLE_ID))
    distance_m = safe_traci_call(lambda: traci.vehicle.getDistance(VEHICLE_ID))
    sim_time = safe_traci_call(lambda: traci.simulation.getTime())
    leader = safe_traci_call(lambda: traci.vehicle.getLeader(VEHICLE_ID, 30.0))
    leader_id = leader[0] if leader else None
    leader_gap_m = leader[1] if leader else None

    traffic_edge_id = road_id or edge_id
    traffic_vehicle_count = None
    traffic_mean_speed_mps = None
    halting_count = None
    travel_time_s = None
    if traffic_edge_id and not str(traffic_edge_id).startswith(":"):
        traffic_vehicle_count = safe_traci_call(lambda: traci.edge.getLastStepVehicleNumber(traffic_edge_id))
        traffic_mean_speed_mps = safe_traci_call(lambda: traci.edge.getLastStepMeanSpeed(traffic_edge_id))
        halting_count = safe_traci_call(lambda: traci.edge.getLastStepHaltingNumber(traffic_edge_id))
        travel_time_s = safe_traci_call(lambda: traci.edge.getTraveltime(traffic_edge_id))

    phone_speed_mps = float(speed_mps or 0.0)
    phone_speed_kmh = phone_speed_mps * 3.6

    try:
        phone_course_deg = None if course_deg_raw is None else float(course_deg_raw)
    except Exception:
        phone_course_deg = None

    try:
        accuracy_m = None if not phone_data else float(phone_data.get("accuracy_m"))
    except Exception:
        accuracy_m = None

    phone_age_s = phone_data_age_seconds(phone_data)
    level, message = feedback_level_for_state(
        sumo_speed_mps,
        allowed_speed_mps,
        leader_gap_m,
        phone_age_s,
        accuracy_m
    )

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

    return {
        "vehicle_id": VEHICLE_ID,
        "sim_time": sim_time,
        "matched": True,
        "edge_id": edge_id,
        "road_id": road_id,
        "lane_id": lane_id,
        "lane_index": lane_index,
        "lane_position_m": lane_position_m,
        "sumo_x": x,
        "sumo_y": y,
        "sumo_speed_mps": sumo_speed_mps,
        "sumo_speed_kmh": None if sumo_speed_mps is None else sumo_speed_mps * 3.6,
        "allowed_speed_mps": allowed_speed_mps,
        "angle_deg": sumo_angle_deg,
        "acceleration_mps2": acceleration_mps2,
        "distance_m": distance_m,
        "traffic_vehicle_count": traffic_vehicle_count,
        "traffic_mean_speed_mps": traffic_mean_speed_mps,
        "halting_count": halting_count,
        "travel_time_s": travel_time_s,
        "leader_id": leader_id,
        "leader_gap_m": leader_gap_m,
        "phone_speed_mps": phone_speed_mps,
        "phone_speed_kmh": phone_speed_kmh,
        "phone_course_deg": phone_course_deg,
        "phone_data_age_s": phone_age_s,
        "phone_accuracy_m": accuracy_m,
        "feedback_level": level,
        "feedback_message": message,
        "bridge_timestamp": datetime.now().isoformat()
    }


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
                    post_sumo_feedback(make_status_feedback("warn", "Waiting for phone GPS data", False, data))
                elif not phone_data_is_fresh(data, STALE_DATA_SECONDS):
                    print("[WARN] Latest phone data is stale; ignoring")
                    post_sumo_feedback(make_status_feedback("danger", "Phone GPS data is stale", False, data))
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

                        feedback = move_vehicle_to_phone_position(lat, lon, speed_mps, course_deg, data)
                        if feedback:
                            post_sumo_feedback(feedback)

            elapsed = time.time() - step_start
            time.sleep(max(0.0, SUMO_STEP_LENGTH - elapsed))

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        try:
            traci.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
