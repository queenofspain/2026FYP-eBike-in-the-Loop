"""
live_phone_to_sumo.py
======================================================================
eBike-in-the-Loop live bridge.

This is the TraCI-side half of the pipeline. It does three jobs on a loop:

    1. Poll the Flask server for the newest GPS fix the phone posted.
    2. Convert that lat/lon into SUMO network (x, y) coordinates and hand
       it to the ST-Matching map-matcher.
    3. Move the virtual eBike to the matched position inside a running
       SUMO simulation via traci.vehicle.moveToXY().

Data flow for one update:

    phone GPS --(HTTP POST)--> Flask /update
                                  |
                                  v
    this script --(HTTP GET)--> Flask /latest --> convertGeo() --> (x, y)
                                  |
                                  v
                   STMatcher.match(x, y, timestamp, speed, course)
                                  |
                                  v
                        traci.vehicle.moveToXY(...)  --> eBike moves

The matcher already decides the final on-edge position, so the vehicle is
placed with keepRoute=2 (exact placement, no re-snapping by SUMO).

NO FALLBACK TO NATIVE MATCHING. If the matcher returns nothing, the update
is SKIPPED entirely and the bike holds its last position. Falling back to
SUMO's own snapping would silently mix native results into the method's
CMP score and make the five-method comparison meaningless. A skipped fix
is visible in the logs; a contaminated one is not.

TIMESTAMPS: unlike the topological matcher, ST-Matching needs the elapsed
time between fixes for its temporal term, so the phone's own fix time is
parsed and passed through. Phone time is used rather than arrival time
because the temporal term is about how fast the RIDER moved, not how fast
the network delivered the packet.
----------------------------------------------------------------------
"""

import os
import sys
import time
import requests
import xml.etree.ElementTree as ET
from datetime import datetime

# ---------- USER SETTINGS ----------
# Path to the SUMO scenario config, relative to this script's own directory.
SUMO_CFG = r"2026-03-11-17-20-46/osm.sumocfg"
# Endpoint on the Flask server that returns the most recent phone fix.
FLASK_LATEST_URL = "http://localhost:5000/latest"

# ID used for the controlled eBike inside SUMO, and the vehicle type we
# create for it (see ensure_vehicle_type).
VEHICLE_ID = "ebike0"
VEHICLE_TYPE_ID = "bike_live"

POLL_INTERVAL = 1.0        # seconds between polls of the Flask server
STALE_DATA_SECONDS = 5.0   # ignore phone fixes older than this
SUMO_STEP_LENGTH = 1.0     # simulation seconds advanced per step
MATCH_THRESHOLD = 100.0    # moveToXY search radius (m) for candidate edges

# ---- ST-Matching parameters ----
# These are the knobs to sweep when tuning. They are named here rather than
# buried in the constructor call so a parameter sweep is a one-line edit.
ST_SEARCH_RADIUS = 50.0    # m; hard cutoff for candidate edge lookup
ST_SIGMA = 20.0            # m; GPS error std for the observation probability
ST_WINDOW_SIZE = 8         # fixes held in the fixed-lag Viterbi window
ST_MAX_CANDIDATES = 5      # per fix; transition cost is quadratic in this
ST_TEMPORAL_MODE = "lou"   # "lou" (faithful) | "ratio" (corrected) | "off"
ST_SPEED_REFERENCE = None  # m/s; set to rescale car speed limits for a bike
ST_VCLASS = None           # e.g. "bicycle" to reject disallowed edges
# ----------------------------------

# SUMO ships its Python TraCI library under $SUMO_HOME/tools, so that path
# has to be on sys.path before "import traci" can succeed. Fail loudly and
# early if the environment variable was never set.
if "SUMO_HOME" not in os.environ:
    raise EnvironmentError("SUMO_HOME is not set. Set it before running this script.")

SUMO_HOME = os.environ["SUMO_HOME"]
TOOLS = os.path.join(SUMO_HOME, "tools")
if TOOLS not in sys.path:
    sys.path.append(TOOLS)

import traci
# The ST-Matching map-matcher lives in st_matching.py alongside this file.
from STMatching import STMatcher

# Built once in main(), read inside move_vehicle_to_phone_position().
MATCHER = None

# Anchor relative paths to THIS FILE's directory, not the working directory,
# so the script runs correctly regardless of where it was launched from.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def parse_sumocfg_for_netfile(sumocfg_path: str) -> str:
    """
    Read the .sumocfg XML and return the absolute path to its <net-file>.

    The matcher needs the .net.xml directly (it loads the network with
    sumolib, independently of TraCI), but the scenario only names the net
    file inside the config. This digs it out and resolves it relative to
    the config's own directory so it works regardless of launch dir.
    """
    tree = ET.parse(sumocfg_path)
    root = tree.getroot()

    # <input> holds the file references (net-file, route-files, etc.).
    input_tag = root.find("input")
    if input_tag is None:
        raise ValueError(f"No <input> section found in {sumocfg_path}")

    # <net-file value="..."/> is the entry we actually want.
    net_tag = input_tag.find("net-file")
    if net_tag is None:
        raise ValueError(f"No <net-file> entry found in {sumocfg_path}")

    net_value = net_tag.get("value")
    if not net_value:
        raise ValueError(f"net-file has no value in {sumocfg_path}")

    # The path in the config is relative to the config file itself, not to
    # wherever this script was launched, so anchor it to the config's dir.
    base_dir = os.path.dirname(os.path.abspath(sumocfg_path))
    return os.path.abspath(os.path.join(base_dir, net_value))


def get_latest_phone_data(url: str):
    """
    GET the latest phone fix from Flask.

    Returns the parsed JSON dict on success, or None on any failure
    (network error, non-200 status, non-dict body). Never raises, so a
    transient Flask hiccup can't kill the simulation loop.
    """
    try:
        r = requests.get(url, timeout=2)
        r.raise_for_status()
        data = r.json()
        return data if isinstance(data, dict) else None
    except Exception as e:
        print(f"[WARN] Could not get latest data from Flask: {e}")
        return None


def phone_data_is_valid(data) -> bool:
    """True only if we have a dict with both a latitude and a longitude."""
    return bool(data) and data.get("lat") is not None and data.get("lon") is not None


def phone_data_is_fresh(data, stale_seconds: float) -> bool:
    """
    True if the fix was received by the server within the last
    `stale_seconds`. Guards against driving the eBike off an old fix if
    the phone stops sending (signal loss, app backgrounded, etc.).

    Uses server_received_at (set by Flask when the POST arrived) rather
    than the phone's own timestamp, to avoid clock-skew between devices.
    """
    received = data.get("server_received_at")
    if not received:
        return False

    try:
        # Normalise a trailing "Z" to an explicit +00:00 offset so
        # fromisoformat() can parse it.
        t = received.replace("Z", "+00:00")
        dt = datetime.fromisoformat(t)
        # Compare in the same tz-awareness as the parsed timestamp.
        now = datetime.now(dt.tzinfo) if dt.tzinfo else datetime.now()
        return (now - dt).total_seconds() <= stale_seconds
    except Exception:
        # Any parse failure is treated as "not fresh" rather than crashing.
        return False


def phone_timestamp_seconds(ts_raw):
    """
    Convert the phone's ISO-8601 fix time into epoch seconds (float).

    ST-Matching divides by the elapsed time between fixes, so this feeds the
    temporal term directly. The phone's OWN timestamp is used rather than
    the server arrival time: network jitter would otherwise show up as the
    rider changing speed. Returns None if unparseable, in which case the
    matcher falls back to its nominal_dt.
    """
    if not ts_raw:
        return None
    try:
        return datetime.fromisoformat(str(ts_raw).replace("Z", "+00:00")).timestamp()
    except Exception:
        return None


def ensure_vehicle_type():
    """
    Make sure the VEHICLE_TYPE_ID exists in the running simulation.

    Preferred path: clone SUMO's built-in DEFAULT_BIKETYPE and cap its top
    speed at 12 m/s. If that bike type isn't present in this build/network,
    fall back to cloning DEFAULT_VEHTYPE and forcing its vehicle class to
    "bicycle" so it still behaves like a bike on the network.
    """
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
    """True if our controlled eBike is currently in the simulation."""
    try:
        return VEHICLE_ID in traci.vehicle.getIDList()
    except Exception:
        return False


def spawn_vehicle_if_missing():
    """
    Create the eBike if it isn't in the simulation yet.

    moveToXY needs a vehicle to exist before it can be moved, and SUMO
    needs every vehicle to have a route to be added at all. We don't have
    a real route (the eBike is driven entirely by GPS), so we give it a
    throwaway single-edge route just to satisfy that requirement; its
    actual position is overridden every step by moveToXY.

    Returns True if the vehicle exists (or was successfully spawned).
    """
    if vehicle_exists():
        return True

    route_id = f"route_{VEHICLE_ID}"

    # Pull all edges and drop internal junction edges (their IDs start with
    # ":"), which aren't valid as a route's starting edge.
    edge_ids = traci.edge.getIDList()
    usable_edges = [e for e in edge_ids if not e.startswith(":")]

    if not usable_edges:
        print("[ERROR] No usable edges found in network.")
        return False

    # Any real edge will do as the dummy route's single edge.
    first_edge = usable_edges[0]

    try:
        # Register the one-edge route once; reuse it on later respawns.
        if route_id not in traci.route.getIDList():
            traci.route.add(route_id, [first_edge])

        # Add the eBike, parked at the start of that edge, stationary.
        traci.vehicle.add(
            vehID=VEHICLE_ID,
            routeID=route_id,
            typeID=VEHICLE_TYPE_ID,
            depart="now",
            departLane="best",
            departPos="base",
            departSpeed="0"
        )

        # setSpeedMode(0) disables all of SUMO's built-in safety checks
        # (car-following, junction, red-light, etc.) so the bike goes
        # exactly where GPS/moveToXY puts it instead of braking itself.
        traci.vehicle.setSpeedMode(VEHICLE_ID, 0)
        traci.vehicle.setSpeed(VEHICLE_ID, 0.0)

        print(f"[INFO] Spawned {VEHICLE_ID} on edge {first_edge}")
        return True

    except traci.TraCIException as e:
        print(f"[WARN] Could not spawn {VEHICLE_ID}: {e}")
        return False


def parse_course_deg(course_deg_raw):
    """
    Return a valid heading in degrees, or INVALID_DOUBLE_VALUE if unusable.

    The phone client sends course_deg = 0 (or negative) when a real heading
    isn't available, so anything negative is rejected. Valid values are
    normalised into the 0..360 range. INVALID_DOUBLE_VALUE is SUMO's
    sentinel that tells moveToXY "no angle supplied, work it out yourself".

    NOTE: ST-Matching itself does not use heading -- Lou's formulation has
    no heading term. The angle is still parsed and passed to moveToXY so the
    bike is drawn facing the right way in the GUI.
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


def _fmt(value, spec=".3f"):
    """Format a float for the log line, or '--' if it's None."""
    # The ST components are legitimately None on the first fix of a route and
    # whenever the Viterbi chain restarts, so the logger has to tolerate it.
    return format(value, spec) if value is not None else "--"


def move_vehicle_to_phone_position(lat, lon, speed_mps, course_deg_raw, fix_time):
    """
    Convert one GPS fix to SUMO coordinates, run ST-Matching, and move the
    controlled bike there. Respawns the vehicle first if it disappeared.

    If the matcher returns None the update is skipped -- see the module
    docstring on why there is deliberately no native-matching fallback.
    """
    # Nothing to move if the vehicle can't be (re)created.
    if not spawn_vehicle_if_missing():
        return

    try:
        # Project lat/lon into the network's local Cartesian frame. Note
        # convertGeo takes (lon, lat) order with fromGeo=True.
        x, y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
    except traci.TraCIException as e:
        print(f"[WARN] Geo conversion failed: {e}")
        return

    # angle_to_use is what we hand to moveToXY (may be INVALID_DOUBLE_VALUE).
    angle_to_use = parse_course_deg(course_deg_raw)

    # The matcher wants a plain float or None for heading, so translate
    # SUMO's INVALID sentinel back into None before calling it.
    course_for_match = angle_to_use
    if course_for_match == traci.constants.INVALID_DOUBLE_VALUE:
        course_for_match = None

    # Run ST-Matching on the converted point. Wrapped in a broad try/except
    # so a matcher bug skips one fix rather than killing the loop.
    # match_ms is the per-fix matching latency -- ST-Matching runs Viterbi
    # over a window with shortest-path queries per candidate pair, so this is
    # the expensive method and the number the real-time claim rests on.
    match_start = time.perf_counter()
    try:
        result = MATCHER.match(
            x, y,
            timestamp=fix_time,
            speed_mps=speed_mps,
            course_deg=course_for_match,
        )
    except Exception:
        import traceback
        print("[ERROR] Matcher raised:")
        traceback.print_exc()
        result = None
    match_ms = (time.perf_counter() - match_start) * 1000.0

    # No match -> skip this fix entirely. The bike holds its last position.
    if result is None:
        print(f"[SKIP] No candidate edge within {ST_SEARCH_RADIUS:.0f} m "
              f"of ({x:.1f}, {y:.1f}) -- update skipped ({match_ms:.1f} ms)")
        return

    # Matcher succeeded: use its chosen on-edge (x, y) and edge id.
    move_x, move_y = result["x"], result["y"]
    edge_id = result["edge_id"]
    lane_index = 0

    # keepRoute=2 places the bike at exactly (move_x, move_y) without
    # re-snapping, so the matcher's decision survives. Anything else lets
    # SUMO's native matching override it at precisely the junctions this
    # project is about.
    try:
        traci.vehicle.moveToXY(
            vehID=VEHICLE_ID,
            edgeID=edge_id,
            laneIndex=lane_index,
            x=move_x,
            y=move_y,
            angle=angle_to_use,
            keepRoute=2,
            matchThreshold=MATCH_THRESHOLD
        )
    except traci.TraCIException as e:
        print(f"[WARN] moveToXY failed: {e}")
        return

    # Push the phone's reported speed onto the vehicle (clamped >= 0). This
    # is cosmetic for the digital twin; position comes from moveToXY.
    try:
        traci.vehicle.setSpeed(VEHICLE_ID, max(0.0, float(speed_mps or 0.0)))
    except traci.TraCIException:
        pass

    # ---- Read back SUMO's current state for the comparison print-out ----
    try:
        sumo_speed_mps = traci.vehicle.getSpeed(VEHICLE_ID)
    except traci.TraCIException:
        sumo_speed_mps = None

    try:
        sumo_angle_deg = traci.vehicle.getAngle(VEHICLE_ID)
    except traci.TraCIException:
        sumo_angle_deg = None

    # Phone-side speed in both units for the log line.
    phone_speed_mps = float(speed_mps or 0.0)
    phone_speed_kmh = phone_speed_mps * 3.6

    # Phone-side heading (raw, before the INVALID/normalise handling).
    try:
        phone_course_deg = None if course_deg_raw is None else float(course_deg_raw)
    except Exception:
        phone_course_deg = None

    # Format SUMO speed, or "N/A" if the read-back failed.
    if sumo_speed_mps is not None:
        sumo_speed_kmh = sumo_speed_mps * 3.6
        sumo_speed_str = f"{sumo_speed_mps:.2f} m/s ({sumo_speed_kmh:.2f} km/h)"
    else:
        sumo_speed_str = "N/A"

    sumo_angle_str = f"{sumo_angle_deg:.1f} deg" if sumo_angle_deg is not None else "N/A"
    phone_course_str = f"{phone_course_deg:.1f} deg" if phone_course_deg is not None else "N/A"

    # How far the matcher moved the point off the raw GPS reading. Large
    # values are not automatically errors -- correcting noise is the whole
    # point -- but a persistent large offset is worth investigating.
    correction_m = ((move_x - x) ** 2 + (move_y - y) ** 2) ** 0.5

    # ST score components. trans and ft are the two terms that the header
    # comment in st_matching.py predicts will be uninformative at 1 Hz;
    # watching them here is the quickest sanity check of that prediction.
    comps = result.get("components", {})

    # One line per update. xy is the MATCHED position actually sent to
    # moveToXY; raw is the unmodified converted GPS point.
    print(
        f"[OK] {VEHICLE_ID} -> edge={edge_id}, "
        f"xy=({move_x:.1f}, {move_y:.1f}), raw=({x:.1f}, {y:.1f}), "
        f"corr={correction_m:.2f} m | "
        f"win={result.get('window_len', '--')} "
        f"obs={_fmt(comps.get('obs'), '.5f')} "
        f"trans={_fmt(comps.get('trans'))} "
        f"fs={_fmt(comps.get('fs'), '.5f')} "
        f"ft={_fmt(comps.get('ft'))} | "
        f"match={match_ms:.1f} ms | "
        f"PHONE speed={phone_speed_mps:.2f} m/s ({phone_speed_kmh:.2f} km/h), "
        f"PHONE course={phone_course_str} | "
        f"SUMO speed={sumo_speed_str}, SUMO angle={sumo_angle_str}"
    )


def main():
    # MATCHER is assigned here and read inside move_vehicle_to_phone_position,
    # so it has to be declared global before first assignment.
    global MATCHER

    # Resolve and validate the SUMO config path up front, anchored to this
    # script's directory so the launch dir doesn't matter.
    sumocfg_abs = os.path.abspath(os.path.join(SCRIPT_DIR, SUMO_CFG))
    if not os.path.exists(sumocfg_abs):
        raise FileNotFoundError(f"SUMO config not found: {sumocfg_abs}")

    # Extract the .net.xml path the matcher needs from the config.
    net_file = parse_sumocfg_for_netfile(sumocfg_abs)

    # Build the matcher once (it loads the whole network with sumolib, which
    # is relatively expensive) and reuse it for every fix.
    print("[INFO] Loading network into ST-Matching matcher...")
    MATCHER = STMatcher(
        net_file,
        search_radius=ST_SEARCH_RADIUS,
        sigma=ST_SIGMA,
        window_size=ST_WINDOW_SIZE,
        max_candidates=ST_MAX_CANDIDATES,
        temporal_mode=ST_TEMPORAL_MODE,
        speed_reference=ST_SPEED_REFERENCE,
        nominal_dt=POLL_INTERVAL,
        vclass=ST_VCLASS,
    )
    print(f"[INFO] ST-Matching ready "
          f"(window={ST_WINDOW_SIZE}, candidates={ST_MAX_CANDIDATES}, "
          f"temporal={ST_TEMPORAL_MODE}).")

    print(f"[INFO] SUMO config: {sumocfg_abs}")
    print(f"[INFO] Net file:    {net_file}")
    print("[INFO] Starting SUMO...")

    # Launch SUMO under TraCI's control:
    #   sumo-gui        -> visual front-end (use "sumo" for headless)
    #   --start         -> begin stepping immediately, no manual play
    #   --delay         -> GUI pacing; handled by SUMO so its event loop
    #                      keeps running (a manual sleep() starves the GUI)
    #   --end 1000000   -> effectively never auto-terminate
    traci.start([
        "sumo-gui",
        "-c", sumocfg_abs,
        "--step-length", str(SUMO_STEP_LENGTH),
        "--delay", str(SUMO_STEP_LENGTH * 1000.0),
        "--start",
        "--end", "1000000"
    ])

    # The vehicle type must exist before the first spawn attempt.
    ensure_vehicle_type()

    # last_seen_timestamp: dedupes fixes so we only act on genuinely new data.
    # last_poll_time: rate-limits how often we hit the Flask server.
    last_seen_timestamp = None
    last_poll_time = 0.0

    try:
        # ---- Main real-time loop ----
        while True:
            # Advance the simulation by one step every iteration. SUMO's
            # --delay handles the pacing, so there is no sleep() here.
            traci.simulationStep()

            now = time.time()
            # Only poll Flask at most once per POLL_INTERVAL, independent of
            # the simulation step rate.
            if now - last_poll_time >= POLL_INTERVAL:
                last_poll_time = now

                data = get_latest_phone_data(FLASK_LATEST_URL)

                # Skip anything missing coordinates or too old to trust.
                if not phone_data_is_valid(data):
                    print("[WARN] No valid phone data yet")
                elif not phone_data_is_fresh(data, STALE_DATA_SECONDS):
                    print("[WARN] Latest phone data is stale; ignoring")
                else:
                    phone_timestamp = data.get("phone_timestamp")

                    # Only move the bike when this is a fix we haven't
                    # already processed (same timestamp => same fix).
                    if phone_timestamp != last_seen_timestamp:
                        last_seen_timestamp = phone_timestamp

                        lat = float(data["lat"])
                        lon = float(data["lon"])

                        # Speed may be missing/None; coerce to 0.0 safely.
                        try:
                            speed_mps = float(data.get("speed_mps", 0.0) or 0.0)
                        except Exception:
                            speed_mps = 0.0

                        course_deg = data.get("course_deg")

                        # Fix time in epoch seconds for the temporal term.
                        fix_time = phone_timestamp_seconds(phone_timestamp)

                        # Do the actual convert -> match -> move for this fix.
                        move_vehicle_to_phone_position(
                            lat, lon, speed_mps, course_deg, fix_time
                        )

    except KeyboardInterrupt:
        # Ctrl+C is the intended way to stop; exit the loop cleanly.
        print("\n[INFO] Stopped by user.")
    finally:
        # Always try to close the TraCI connection so SUMO shuts down.
        try:
            traci.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()