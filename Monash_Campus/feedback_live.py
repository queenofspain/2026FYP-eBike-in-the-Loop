"""Run the existing five-method GPS bridge with an additive advice publisher.

The original live_phone_to_sumo.py remains the authority for GPS handling,
matching, SUMO placement, and CSV logs. This runner observes successful
moveToXY calls and reads road context afterward.
"""

import math
import os

import requests

import live_phone_to_sumo as bridge
from rider_road import collect


BASE_URL = os.environ.get("EBIKE_FEEDBACK_SERVER_URL",
                          f"http://127.0.0.1:{os.environ.get('EBIKE_PORT', '5000')}").rstrip("/")
NETWORK = None
LAST_SPEED = None
LAST_FIX_TIME = None


def publish(path, payload, quiet=False):
    try:
        response = requests.post(BASE_URL + path, json=payload, timeout=1.0)
        response.raise_for_status()
        return True
    except requests.RequestException as error:
        if not quiet:
            print(f"[feedback] Could not publish to {path}: {error}", flush=True)
        return False


def _finite(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError, OverflowError):
        return None


def main():
    global NETWORK, LAST_SPEED, LAST_FIX_TIME
    # parse_args does not prompt; the bridge still owns method/Kalman selection.
    args = bridge.parse_args()
    config = os.path.abspath(os.path.join(bridge.SCRIPT_DIR, args.cfg))
    network_path = bridge.parse_sumocfg_for_netfile(config)
    import sumolib
    NETWORK = sumolib.net.readNet(network_path)

    original_get = bridge.get_latest_phone_data
    original_move = bridge.move_vehicle_to_phone_position

    def get_with_heartbeat(url):
        data = original_get(url)
        valid = bridge.phone_data_is_valid(data) and bridge.phone_data_is_fresh(data, bridge.STALE_DATA_SECONDS)
        publish("/api/feedback/bridge", {"status": "running" if valid else "waiting"}, quiet=True)
        return data

    def move_with_feedback(lat, lon, speed_mps, course_deg_raw, fix_time, accuracy_m):
        global LAST_SPEED, LAST_FIX_TIME
        moved = False
        original_move_to_xy = bridge.traci.vehicle.moveToXY

        def observed_move(*move_args, **move_kwargs):
            nonlocal moved
            result = original_move_to_xy(*move_args, **move_kwargs)
            moved = True
            return result

        bridge.traci.vehicle.moveToXY = observed_move
        try:
            result = original_move(lat, lon, speed_mps, course_deg_raw, fix_time, accuracy_m)
        finally:
            bridge.traci.vehicle.moveToXY = original_move_to_xy
        if not moved:
            return result

        try:
            speed = _finite(speed_mps)
            course = _finite(course_deg_raw)
            elapsed = fix_time - LAST_FIX_TIME if fix_time is not None and LAST_FIX_TIME is not None else None
            acceleration = ((speed - LAST_SPEED) / elapsed if speed is not None and LAST_SPEED is not None
                            and elapsed is not None and 0.1 <= elapsed <= 10 else None)
            if speed is not None:
                LAST_SPEED = speed
            if fix_time is not None:
                LAST_FIX_TIME = fix_time
            if course is None:
                course = _finite(bridge.traci.vehicle.getAngle(bridge.VEHICLE_ID))
            sample = collect(bridge.traci, NETWORK, bridge.VEHICLE_ID,
                             speed_mps=speed, course_deg=course, fix_time=fix_time)
            sample.update(source="live", accel_mps2=acceleration, match_method=bridge.METHOD_KEY)
            publish("/api/feedback/telemetry", sample)
        except Exception as error:
            print(f"[feedback] Road data unavailable for this fix: {error}", flush=True)
        return result

    bridge.get_latest_phone_data = get_with_heartbeat
    bridge.move_vehicle_to_phone_position = move_with_feedback
    publish("/api/feedback/bridge", {"status": "starting"})
    try:
        bridge.main()
    except Exception:
        publish("/api/feedback/bridge", {"status": "error"}, quiet=True)
        raise
    finally:
        publish("/api/feedback/bridge", {"status": "stopped"}, quiet=True)


if __name__ == "__main__":
    main()
