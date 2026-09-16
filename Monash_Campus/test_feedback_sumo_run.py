"""End-to-end rider advice test using SUMO data only; no phone or ngrok.

Run from Monash_Campus with: python -B test_feedback_sumo_run.py
"""

import os
import logging
import threading
import time

import requests
from werkzeug.serving import make_server

import feedback_server


def main():
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    http_server = make_server("127.0.0.1", 0, feedback_server.app, threaded=True)
    port = http_server.server_port
    os.environ["EBIKE_PORT"] = str(port)
    base_url = f"http://127.0.0.1:{port}"
    thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    thread.start()

    samples = []
    advice_codes = set()
    final_status = None
    print(f"Feedback test server: {base_url}", flush=True)
    try:
        response = requests.post(base_url + "/api/feedback/demo/start",
                                 json={"max_time": 12}, timeout=5)
        response.raise_for_status()
        print("Started SUMO-only demo on the current campus route.", flush=True)

        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            status = requests.get(base_url + "/api/feedback/status", timeout=5).json()
            if status["freshness"]["fresh"]:
                sample = status["telemetry"]
                if not samples or sample["time_s"] != samples[-1]["time_s"]:
                    samples.append(sample)
                    advice_codes.add(status["advice"]["code"])
                    print(f"SUMO {sample['time_s']:>4.0f}s | speed {sample['speed_mps'] * 3.6:>5.1f} km/h"
                          f" | limit {status['road_context']['speed_limit_kmh']:>5.1f} km/h"
                          f" | advice: {status['advice']['action']}", flush=True)
            elif status["status"] in {"finished", "stopped", "error"}:
                final_status = status
                break
            time.sleep(0.35)

        assert len(samples) >= 3, f"Expected several fresh SUMO samples, got {len(samples)}"
        assert all(sample["source"] == "demo" for sample in samples)
        assert any(sample["speed_limit_mps"] is not None for sample in samples)
        assert any(sample["intersection_status"] == "ahead" for sample in samples)
        assert any(sample["traffic_light_status"] == "ahead" for sample in samples)
        assert "approach" in advice_codes, f"No approach advice; saw {sorted(advice_codes)}"
        assert final_status is not None and final_status["status"] == "finished", final_status
        assert final_status["telemetry"] is None, "Finished ride still displays current telemetry"
        print(f"PASS: {len(samples)} fresh SUMO samples, intersection and traffic-light context,"
              " approach advice, and clean finish.", flush=True)
    finally:
        try:
            requests.post(base_url + "/api/feedback/demo/stop", json={}, timeout=2)
        except requests.RequestException:
            pass
        http_server.shutdown()
        thread.join(timeout=3)


if __name__ == "__main__":
    main()
