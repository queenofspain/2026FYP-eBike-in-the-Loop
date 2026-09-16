"""Rider advice routes added to the existing GPS Flask app without editing it."""

import os
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

from flask import jsonify, render_template, request

from rider_advice import inactive_advice, number, road_context, select_advice
from server import app


HERE = Path(__file__).resolve().parent
FRESH_SECONDS = 5.0
lock = threading.RLock()
latest_sample = None
previous_advice = None
bridge_state = "stopped"
bridge_seen = 0.0
demo_process = None
demo_run_id = None
demo_state = "idle"
demo_stop = False


def _local_writer():
    # Bridge and demo both POST to localhost. Keep tunnel visitors read-only.
    return request.remote_addr in {"127.0.0.1", "::1"}


def _sample_from(payload):
    if not isinstance(payload, dict) or not isinstance(payload.get("edge_id"), str) or not payload["edge_id"]:
        return None
    sample = {"edge_id": payload["edge_id"], "lane_id": payload.get("lane_id"),
              "route_source": payload.get("route_source")}
    for key in ("time_s", "speed_mps", "accel_mps2", "angle_deg", "speed_limit_mps",
                "intersection_distance_m", "traffic_light_distance_m"):
        sample[key] = number(payload.get(key))
    for prefix in ("intersection", "traffic_light"):
        status = payload.get(prefix + "_status")
        sample[prefix + "_status"] = status if status in {"ahead", "inside", "none", "unknown"} else "unknown"
        sample[prefix + "_id"] = payload.get(prefix + "_id")
    sample["traffic_light_state"] = payload.get("traffic_light_state")
    return sample


@app.route("/feedback")
def feedback_page():
    return render_template("rider_feedback.html")


@app.route("/api/feedback/bridge", methods=["POST"])
def feedback_bridge():
    global bridge_state, bridge_seen, latest_sample, previous_advice
    if not _local_writer():
        return jsonify({"error": "Local bridge only"}), 403
    payload = request.get_json(silent=True)
    state = payload.get("status") if isinstance(payload, dict) else None
    if state not in {"starting", "waiting", "running", "stopped", "error"}:
        return jsonify({"error": "Invalid bridge status"}), 400
    with lock:
        bridge_state, bridge_seen = state, time.time()
        if state in {"starting", "stopped", "error"} and latest_sample and latest_sample["source"] == "live":
            latest_sample = None
            previous_advice = None
    return jsonify({"ok": True})


@app.route("/api/feedback/telemetry", methods=["POST"])
def feedback_telemetry():
    global latest_sample, previous_advice, bridge_state, bridge_seen, demo_state
    if not _local_writer():
        return jsonify({"error": "Local producer only"}), 403
    payload = request.get_json(silent=True)
    sample = _sample_from(payload)
    if sample is None or payload.get("source") not in {"live", "demo"}:
        return jsonify({"error": "Invalid telemetry"}), 400
    source = payload["source"]
    with lock:
        if source == "demo":
            if payload.get("run_id") != demo_run_id or demo_stop:
                return jsonify({"ok": False, "stop": True}), 409
            demo_state = "running"
        else:
            if demo_process is not None and demo_process.poll() is None:
                return jsonify({"error": "Demo ride is active"}), 409
            bridge_state, bridge_seen = "running", time.time()
        sample["source"] = source
        sample["sampled_at"] = time.time()
        latest_sample = sample
        previous_advice = select_advice(sample, previous_advice)
        return jsonify({"ok": True, "stop": demo_stop if source == "demo" else False})


@app.route("/api/feedback/status")
def feedback_status():
    with lock:
        now = time.time()
        fresh = (latest_sample is not None and 0 <= now - latest_sample["sampled_at"] <= FRESH_SECONDS)
        live_active = now - bridge_seen <= FRESH_SECONDS and bridge_state in {"starting", "waiting", "running"}
        demo_exit = demo_process.poll() if demo_process is not None else None
        demo_active = demo_process is not None and demo_exit is None
        if fresh:
            sample = dict(latest_sample)
            state = "running"
            source = sample["source"]
            advice = previous_advice or select_advice(sample)
        else:
            sample = None
            source = "demo" if demo_process is not None and demo_state in {"starting", "running", "finished", "stopped", "error"} else "live"
            if demo_state in {"finished", "stopped"}:
                state = demo_state
            elif demo_active:
                state = "stale" if demo_state == "running" else "starting"
            elif live_active:
                state = "stale" if bridge_state == "running" else "starting"
            elif demo_exit is not None and demo_state in {"starting", "running"}:
                state = "error"
            elif bridge_state == "error":
                state = "error"
            elif latest_sample is not None:
                state = "stale"
            else:
                state = "idle"
            advice = inactive_advice(state)
        return jsonify({"status": state, "source": source, "telemetry": sample,
                        "road_context": road_context(sample), "advice": advice,
                        "freshness": {"fresh": fresh,
                                      "age_s": max(0.0, now - latest_sample["sampled_at"]) if latest_sample else None}})


@app.route("/api/feedback/demo/start", methods=["POST"])
def start_demo():
    global demo_process, demo_run_id, demo_state, demo_stop, latest_sample, previous_advice
    payload = request.get_json(silent=True) or {}
    duration = number(payload.get("max_time", 120)) if isinstance(payload, dict) else None
    if duration is None or not 10 <= duration <= 900:
        return jsonify({"error": "max_time must be from 10 to 900 seconds"}), 400
    with lock:
        if demo_process is not None and demo_process.poll() is None:
            return jsonify({"error": "A demo ride is already running"}), 409
        if time.time() - bridge_seen <= FRESH_SECONDS and bridge_state in {"starting", "waiting", "running"}:
            return jsonify({"error": "Stop the live SUMO bridge before starting a demo"}), 409
        demo_run_id = str(uuid.uuid4())
        demo_state, demo_stop = "starting", False
        latest_sample = previous_advice = None
        try:
            child_env = os.environ.copy()
            child_env["EBIKE_FEEDBACK_SERVER_URL"] = f"http://127.0.0.1:{os.environ.get('EBIKE_PORT', '5000')}"
            demo_process = subprocess.Popen(
                [sys.executable, str(HERE / "feedback_demo.py"), "--run-id", demo_run_id,
                 "--max-time", str(duration)],
                cwd=HERE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                env=child_env,
            )
        except OSError as error:
            demo_state = "error"
            return jsonify({"error": str(error)}), 500
    return jsonify({"ok": True, "run_id": demo_run_id})


@app.route("/api/feedback/demo/stop", methods=["POST"])
def stop_demo():
    global demo_stop, demo_state, latest_sample, previous_advice
    with lock:
        demo_stop = True
        demo_state = "stopped"
        if latest_sample and latest_sample["source"] == "demo":
            latest_sample = previous_advice = None
    return jsonify({"ok": True})


@app.route("/api/feedback/demo/state", methods=["POST"])
def update_demo_state():
    global demo_state, latest_sample, previous_advice
    if not _local_writer():
        return jsonify({"error": "Local demo only"}), 403
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict) or payload.get("run_id") != demo_run_id:
        return jsonify({"error": "Unknown demo run"}), 409
    state = payload.get("status")
    if state not in {"finished", "stopped", "error"}:
        return jsonify({"error": "Invalid demo state"}), 400
    with lock:
        demo_state = state
        if latest_sample and latest_sample["source"] == "demo":
            latest_sample = previous_advice = None
    return jsonify({"ok": True})


if __name__ == "__main__":
    app.run(host=os.environ.get("EBIKE_HOST", "0.0.0.0"),
            port=int(os.environ.get("EBIKE_PORT", "5000")), debug=False,
            use_reloader=False)
