"""Small SUMO-only rider advice demo on the current campus map."""

import argparse
import os
import sys
import time
from pathlib import Path

import requests

from rider_road import collect
from rider_route import upcoming_intersection, upcoming_light


HERE = Path(__file__).resolve().parent
SCENARIO = HERE / "2026-08-25-19-43-30"
SERVER = os.environ.get("EBIKE_FEEDBACK_SERVER_URL", "http://127.0.0.1:5000").rstrip("/")
VEHICLE_ID = "simBike"


def post(path, payload):
    try:
        response = requests.post(SERVER + path, json=payload, timeout=2)
        return response.json()
    except (requests.RequestException, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--max-time", type=float, default=120)
    args = parser.parse_args()
    if "SUMO_HOME" not in os.environ:
        raise EnvironmentError("SUMO_HOME is required")
    sys.path.append(str(Path(os.environ["SUMO_HOME"]) / "tools"))
    import sumolib
    import traci

    network_file = SCENARIO / "osm.net.xml"
    route_file = SCENARIO / "full_campus_v3.rou.xml"
    network = sumolib.net.readNet(str(network_file))
    command = ["sumo", "-n", str(network_file), "-r", str(route_file),
               "--ignore-route-errors", "true", "--step-length", "1",
               "--end", str(args.max_time), "--no-step-log", "true"]
    state = "finished"
    started = False
    try:
        traci.start(command)
        started = True
        while traci.simulation.getTime() < args.max_time:
            tick = time.monotonic()
            traci.simulationStep()
            if VEHICLE_ID not in traci.vehicle.getIDList():
                if traci.simulation.getTime() > 1:
                    break
                continue
            sample = collect(traci, network, VEHICLE_ID,
                             speed_mps=traci.vehicle.getSpeed(VEHICLE_ID),
                             course_deg=traci.vehicle.getAngle(VEHICLE_ID))
            sample.update(upcoming_intersection(traci, network, VEHICLE_ID))
            sample.update(upcoming_light(traci, VEHICLE_ID))
            sample.update(source="demo", run_id=args.run_id,
                          accel_mps2=traci.vehicle.getAcceleration(VEHICLE_ID),
                          route_source="known_sumo_route")
            response = post("/api/feedback/telemetry", sample)
            if response and response.get("stop"):
                state = "stopped"
                break
            time.sleep(max(0.0, 1.0 - (time.monotonic() - tick)))
    except Exception:
        state = "error"
        raise
    finally:
        if started:
            try:
                traci.close()
            except Exception:
                pass
        post("/api/feedback/demo/state", {"run_id": args.run_id, "status": state})


if __name__ == "__main__":
    main()
