import argparse
import time
from datetime import datetime

import requests


DEFAULT_URL = "http://localhost:5000/feedback/update"


SCENARIOS = [
    {
        "name": "OK normal riding",
        "feedback_level": "ok",
        "feedback_message": "Riding normally",
        "sumo_speed_kmh": 14.0,
        "allowed_speed_mps": 7.0,
        "leader_gap_m": 22.0,
        "phone_data_age_s": 0.8,
        "phone_accuracy_m": 6.0,
    },
    {
        "name": "WARN poor GPS accuracy",
        "feedback_level": "warn",
        "feedback_message": "GPS accuracy is poor",
        "sumo_speed_kmh": 13.0,
        "allowed_speed_mps": 7.0,
        "leader_gap_m": 20.0,
        "phone_data_age_s": 0.9,
        "phone_accuracy_m": 14.0,
    },
    {
        "name": "WARN traffic ahead",
        "feedback_level": "warn",
        "feedback_message": "Traffic ahead",
        "sumo_speed_kmh": 12.0,
        "allowed_speed_mps": 7.0,
        "leader_gap_m": 7.5,
        "phone_data_age_s": 0.7,
        "phone_accuracy_m": 5.0,
    },
    {
        "name": "WARN above SUMO lane speed",
        "feedback_level": "warn",
        "feedback_message": "Above SUMO lane speed",
        "sumo_speed_kmh": 28.0,
        "allowed_speed_mps": 6.0,
        "leader_gap_m": 18.0,
        "phone_data_age_s": 0.8,
        "phone_accuracy_m": 5.0,
    },
    {
        "name": "DANGER vehicle very close ahead",
        "feedback_level": "danger",
        "feedback_message": "Vehicle very close ahead",
        "sumo_speed_kmh": 10.0,
        "allowed_speed_mps": 7.0,
        "leader_gap_m": 4.0,
        "phone_data_age_s": 0.8,
        "phone_accuracy_m": 5.0,
    },
    {
        "name": "DANGER stale phone GPS",
        "feedback_level": "danger",
        "feedback_message": "Phone GPS data is stale",
        "sumo_speed_kmh": 0.0,
        "allowed_speed_mps": 7.0,
        "leader_gap_m": 20.0,
        "phone_data_age_s": 6.0,
        "phone_accuracy_m": 5.0,
    },
]


def build_payload(scenario, index):
    speed_kmh = scenario["sumo_speed_kmh"]
    return {
        "vehicle_id": "ebike0",
        "sim_time": index,
        "matched": True,
        "edge_id": f"scenario_edge_{index}",
        "road_id": f"scenario_edge_{index}",
        "lane_id": f"scenario_edge_{index}_0",
        "lane_index": 0,
        "lane_position_m": 40.0 + index * 10.0,
        "sumo_x": 1000.0 + index,
        "sumo_y": 2000.0 + index,
        "sumo_speed_mps": speed_kmh / 3.6,
        "sumo_speed_kmh": speed_kmh,
        "allowed_speed_mps": scenario["allowed_speed_mps"],
        "angle_deg": (80 + index * 20) % 360,
        "traffic_vehicle_count": 1 if scenario["leader_gap_m"] < 10 else 0,
        "traffic_mean_speed_mps": 4.5,
        "halting_count": 0,
        "leader_id": "scenario_vehicle_ahead" if scenario["leader_gap_m"] < 30 else None,
        "leader_gap_m": scenario["leader_gap_m"],
        "phone_data_age_s": scenario["phone_data_age_s"],
        "phone_accuracy_m": scenario["phone_accuracy_m"],
        "feedback_level": scenario["feedback_level"],
        "feedback_message": scenario["feedback_message"],
        "bridge_timestamp": datetime.now().isoformat(),
    }


def post_payload(url, payload):
    response = requests.post(url, json=payload, timeout=2)
    response.raise_for_status()


def main():
    parser = argparse.ArgumentParser(description="Send no-SUMO rider feedback test scenarios to Flask.")
    parser.add_argument("--url", default=DEFAULT_URL, help="Feedback update URL.")
    parser.add_argument("--delay", type=float, default=3.0, help="Seconds between scenarios.")
    parser.add_argument("--loop", action="store_true", help="Repeat scenarios until stopped.")
    args = parser.parse_args()

    print("[test] Open http://localhost:5000/feedback")
    print("[test] Sending controlled rider feedback scenarios.")

    while True:
        for index, scenario in enumerate(SCENARIOS, start=1):
            payload = build_payload(scenario, index)
            post_payload(args.url, payload)
            print(f"[test] {scenario['feedback_level'].upper()}: {scenario['name']}")
            time.sleep(args.delay)

        if not args.loop:
            break

    print("[test] Scenario test complete.")


if __name__ == "__main__":
    main()
