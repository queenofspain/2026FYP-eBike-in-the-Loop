import math
import time
from datetime import datetime

import requests


FEEDBACK_URL = "http://localhost:5000/feedback/update"


def main():
    tick = 0
    print("[demo] Posting fake SUMO rider feedback to Flask.")
    print("[demo] Open http://localhost:5000/feedback")

    while True:
        speed_kmh = 12 + 8 * math.sin(tick / 6)
        leader_gap = 18 + 12 * math.sin(tick / 8)
        level = "ok"
        message = "Riding normally"

        if leader_gap < 5:
            level = "danger"
            message = "Vehicle very close ahead"
        elif leader_gap < 10:
            level = "warn"
            message = "Traffic ahead"

        payload = {
            "vehicle_id": "ebike0",
            "sim_time": tick,
            "matched": True,
            "edge_id": "demo_edge_42",
            "road_id": "demo_edge_42",
            "lane_id": "demo_edge_42_0",
            "lane_index": 0,
            "lane_position_m": 80 + tick,
            "sumo_speed_mps": speed_kmh / 3.6,
            "sumo_speed_kmh": speed_kmh,
            "allowed_speed_mps": 25 / 3.6,
            "angle_deg": (90 + tick * 3) % 360,
            "traffic_vehicle_count": 3,
            "traffic_mean_speed_mps": 4.5,
            "halting_count": 0,
            "leader_id": "demo_car_1" if leader_gap < 30 else None,
            "leader_gap_m": max(2, leader_gap),
            "phone_data_age_s": 0.7,
            "phone_accuracy_m": 8.0,
            "feedback_level": level,
            "feedback_message": message,
            "bridge_timestamp": datetime.now().isoformat()
        }

        try:
            requests.post(FEEDBACK_URL, json=payload, timeout=1).raise_for_status()
            print(f"[demo] {level.upper()}: {message} speed={speed_kmh:.1f} km/h")
        except Exception as e:
            print(f"[demo] Could not post feedback: {e}")

        tick += 1
        time.sleep(1)


if __name__ == "__main__":
    main()
