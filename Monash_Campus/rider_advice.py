"""Road-only rider advice. Distances and thresholds are simulation prototypes."""

import math


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) and result > -1e9 else None
    except (TypeError, ValueError, OverflowError):
        return None


def positive(value):
    result = number(value)
    return result if result is not None and result >= 0 else None


def road_context(sample):
    sample = sample or {}
    def feature(prefix):
        distance = positive(sample.get(prefix + "_distance_m"))
        status = sample.get(prefix + "_status", "unknown")
        if status not in {"ahead", "none", "inside"}:
            status = "unknown"
        if status == "ahead" and distance is None:
            status = "unknown"
        return {"status": status, "distance_m": distance if status == "ahead" else None,
                "id": sample.get(prefix + "_id")}

    intersection = feature("intersection")
    light = feature("traffic_light")
    # Keep yielding/off states distinct; never translate them into plain green.
    signals = {"r": "Red", "R": "Red", "y": "Yellow", "Y": "Yellow",
               "G": "Green", "g": "Green · yield", "s": "Stop · yield",
               "u": "Red + yellow", "o": "Off · yield", "O": "Off"}
    light["signal"] = signals.get(sample.get("traffic_light_state")) if light["status"] == "ahead" else None
    limit = positive(sample.get("speed_limit_mps"))
    return {"speed_limit_kmh": limit * 3.6 if limit is not None else None,
            "limit_on_junction": str(sample.get("edge_id", "")).startswith(":"),
            "intersection": intersection, "traffic_light": light}


def advice(code, level, action, reason, **details):
    return dict(code=code, level=level, action=action, reason=reason, **details)


def select_advice(sample, previous=None):
    """Select one action; optional signal colors do not control advice in this stage."""
    context = road_context(sample)
    speed = positive(sample.get("speed_mps"))
    limit = context["speed_limit_kmh"]
    previous = previous or {}
    excess = speed * 3.6 - limit if speed is not None and limit is not None else None
    # Different entry/exit tolerances prevent threshold flicker.
    if excess is not None and excess > (0.36 if previous.get("code") == "overspeed" else 1.08):
        return advice("overspeed", "warning", "Reduce speed",
                      f"{excess:.1f} km/h above the {limit:.0f} km/h road limit.", limit_kmh=limit)

    lookahead = max(30.0, min(80.0, (speed or 0) * 6.0))
    candidates = []
    for key, label in (("traffic_light", "Upcoming traffic light"), ("intersection", "Upcoming intersection")):
        feature = context[key]
        same = previous.get("source") == key and previous.get("feature_id") == feature["id"]
        if feature["status"] == "ahead" and feature["distance_m"] <= lookahead + (10 if same else 0):
            candidates.append((feature["distance_m"], key, label, feature))
    if candidates:
        candidates.sort(key=lambda item: (item[0], item[1] != "traffic_light"))
        chosen = candidates[0]
        # Retain the same nearby feature on almost-equal distances.
        for candidate in candidates:
            if (candidate[1] == previous.get("source") and candidate[3]["id"] == previous.get("feature_id")
                    and candidate[0] <= chosen[0] + 5):
                chosen = candidate
                break
        distance, key, label, feature = chosen
        stopped = speed is not None and speed < 0.2 and distance <= 5 and key == "traffic_light"
        return advice("approach", "caution", "Check the traffic light" if stopped else "Approach carefully",
                      f"{label} · {distance:.0f} m ahead", source=key, feature_id=feature["id"], distance_m=distance)
    if context["intersection"]["status"] == "inside":
        return advice("crossing", "caution", "Continue carefully", "Travelling through an intersection.")
    if speed is None or limit is None or any(context[k]["status"] == "unknown" for k in ("intersection", "traffic_light")):
        return advice("incomplete", "neutral", "Stay attentive", "Some road information is unavailable.")
    if speed < 0.2:
        return advice("stationary", "neutral", "Check before moving", "The simulated bicycle is stationary.")
    return advice("steady", "good", "Keep a steady pace", f"Within the {limit:.0f} km/h road limit.")


def inactive_advice(status):
    messages = {
        "idle": ("Start GPS or a demo ride", "Your current advice will appear here."),
        "starting": ("Waiting for rider data", "Advice appears after a fresh SUMO position is available."),
        "stopped": ("Ride stopped", "Start another ride when you are ready."),
        "finished": ("Ride complete", "Start another ride when you are ready."),
        "error": ("Advice unavailable", "The SUMO ride encountered an error."),
        "stale": ("Advice unavailable", "Waiting for fresh position data from the active ride."),
    }
    action, reason = messages.get(status, messages["stale"])
    return advice(status, "neutral", action, reason)
