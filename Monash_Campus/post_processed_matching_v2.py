"""
post_processed_matching_v2.py
======================================================================
Post-processing GPS-to-road map matcher for SUMO -- UNIFIED MULTI-METHOD
RUNNER (offline counterpart of live_phone_to_sumo.py).

This is post_processed_matching_v1.py's job (take a CSV of already
recorded GPS points and map-match every point onto a SUMO network) done
with live_phone_to_sumo.py's engine: the same METHODS registry, the same
method/Kalman selection UI, and the same matcher/Kalman-filter machinery,
just fed from a CSV instead of a live Flask poll and with nothing
vehicle- or GUI-related left in (there's no bike to animate here).

    python post_processed_matching_v2.py --method st --kalman
    python post_processed_matching_v2.py --method native --input Data/ride.csv
    python post_processed_matching_v2.py                 (prompts for method + kalman)

Methods (identical registry to live_phone_to_sumo.py):
    native  -- SUMO's own convertRoad snapping (the geometric baseline)
    topo    -- TopologicalMatcher   (topological.py)
    fuzzy   -- FuzzyMatcher         (FuzzyLogic.py)
    hmm     -- HMMMatcher           (HMM.py)
    st      -- STMatcher            (STMatching.py)

For every input point this writes out:
  - the original lat, lon (and any other columns the input CSV had)
  - which method/Kalman setting produced this row
  - whether a match was found, and if not, why (unmatched_reason)
  - the matched edge ID, lane index, and lane position (Newson-Krumm
    style error metrics need these)
  - whether the match landed on a junction/internal edge (is_internal_edge)
  - raw / Kalman-filtered / matched coordinates (SUMO x/y), plus the
    matched point back-projected to lat/lon
  - the matcher's own diagnostics where it has them: score, score
    components, ST-Matching's Viterbi window length, and per-point
    matching latency (match_ms)

WHY ONE SCRIPT INSTEAD OF FIVE (SAME REASONING AS THE LIVE SCRIPT)
-------------------------------------------------------------------
Every difference between five separate post-processing scripts that is
NOT the matcher itself is a confound in the CMP comparison. Holding
method selection, Kalman pre-filtering, CSV I/O and logging in one file
means the five methods are run under identical conditions on the exact
same recorded ride, and a fix to the harness applies to all of them.

WHY THIS ISN'T JUST live_phone_to_sumo.py WITH A CSV BOLTED ON
-------------------------------------------------------------------
The live script's job per fix is "poll -> convert -> match -> move a
vehicle -> read back where SUMO actually put it". The last two steps
only exist because there's a bike being animated in a running
simulation. Here there is no vehicle, no moveToXY, no keepRoute, no
simulationStep(): traci is only kept around because convertGeo() and
(for the native method) convertRoad() need a loaded network to talk to.
Concretely, dropped from the live script: ensure_vehicle_type,
spawn_vehicle_if_missing, move_vehicle_to_phone_position's moveToXY/
getPosition/getSpeed/getAngle read-back, the Flask polling helpers, and
the GUI/--no-gui/--delay/--step-length plumbing. Added, from
post_processed_matching_v1.py: CSV loading/writing, and (since native
matching no longer has a moveToXY snap to read back afterwards) using
sumolib to look up the actual on-lane point from the edge/lane/pos that
convertRoad returns -- see snap_to_lane_geometry().

TIMESTAMPS: if the CSV has a timestamp column, points are sorted into
chronological order and consecutive-fix elapsed time is computed from it
and fed to whichever matcher/Kalman filter wants it (ST-Matching and the
HMM's temporal terms, the Kalman filter's dt). Without one, everything
still runs, just with a nominal fixed dt (NOMINAL_DT) standing in for
elapsed time between fixes -- the same graceful-degradation behaviour
the live script has when a phone fix has no parseable timestamp.
----------------------------------------------------------------------
"""

import os
import sys
import csv
import time
import math
import inspect
import argparse
import traceback
import xml.etree.ElementTree as ET
from datetime import datetime

# ---------- USER SETTINGS ----------
# Path to the SUMO scenario config, relative to this script's own directory.
# NOTE: the two reference scripts point at different dated scenario folders
# ("2026-03-11-17-20-46" in post_processed_matching_v1.py vs
# "2026-08-25-19-43-30" in live_phone_to_sumo.py). This defaults to the
# live script's (more recent) folder -- update it, or pass --sumocfg, if
# that's not the scenario this CSV was recorded against.
# SUMO_CFG = r"2026-08-25-19-43-30/osm.sumocfg"
SUMO_CFG = r"2026-03-11-17-20-46/osm.sumocfg"

DEFAULT_INPUT_CSV = "Data/gps_data_processed.csv"
OUTPUT_DIR = "Data"

DEFAULT_LAT_COL = "lat"
DEFAULT_LON_COL = "lon"
DEFAULT_TIMESTAMP_COL = "phone_timestamp"
DEFAULT_SPEED_COL = "speed_mps"
DEFAULT_COURSE_COL = "course_deg"
DEFAULT_ACCURACY_COL = "accuracy_m"

# Standing in for the live script's POLL_INTERVAL: used as the nominal
# elapsed-time-between-fixes whenever a point has no usable timestamp, for
# both the Kalman filter's dt and ST-Matching's nominal_dt.
NOMINAL_DT = 1.0

# Recorded logs can have bigger gaps than a live 1 Hz poll ever would
# (paused recording, a lost fix, etc.), so the Kalman filter's dt is capped
# higher than the live script's STALE_DATA_SECONDS to avoid needlessly
# discarding a filter state over an ordinary gap in the data.
MAX_KALMAN_DT = 30.0

# Kept for CLI/documentation parity with the live script's MATCH_THRESHOLD.
# NOTE: traci.simulation.convertRoad has no search-radius parameter (only
# moveToXY does, and nothing here calls moveToXY), so this value is
# informational only -- it does not change native matching's behaviour.
MATCH_THRESHOLD = 100.0
# ------------------------------------

# SUMO ships its Python TraCI library under $SUMO_HOME/tools, so that path
# has to be on sys.path before "import traci" can succeed.
if "SUMO_HOME" not in os.environ:
    raise EnvironmentError("SUMO_HOME is not set. Set it before running this script.")

SUMO_HOME = os.environ["SUMO_HOME"]
TOOLS = os.path.join(SUMO_HOME, "tools")
if TOOLS not in sys.path:
    sys.path.append(TOOLS)

import traci  # noqa: E402
import sumolib  # noqa: E402

# Anchor relative paths to THIS FILE's directory, not the working directory.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# The script's own directory must also be importable, so
# "from topological import ..." etc. work regardless of launch dir.
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


# ======================================================================
# METHOD REGISTRY -- identical to live_phone_to_sumo.py
# ======================================================================
# One entry per map-matching method. This is the ONLY place that knows
# anything method-specific. See live_phone_to_sumo.py for the full
# rationale behind each field; unchanged here so a tuning change made for
# the live comparison applies to the offline one too.
METHODS = {
    "native": {
        "module": None,
        "cls": None,
        "kwargs": {},
        "label": "Geometric (SUMO native convertRoad)",
    },
    "topo": {
        "module": "topological",
        "cls": "TopologicalMatcher",
        "kwargs": {
            "search_radius": 50.0,
            "sigma_prox": 15.0,
            "weights": (0.40, 0.30, 0.20, 0.10),
            "min_speed_for_heading": 0.5,
            "vclass": None,
        },
        "label": "Topological (weighted, Velaga et al.)",
    },
    "fuzzy": {
        "module": "FuzzyLogic",
        "cls": "FuzzyMatcher",
        "kwargs": {
            "search_radius": 50.0,
            "dist_half_width": 2.5,
            "angle_small_break": 25.0,
            "angle_large_break": 65.0,
            "junction_threshold": 5.0,
            "min_speed_for_switch": 0.5,
            "confirm_count": 2,
            "output_low": 10.0,
            "output_average": 50.0,
            "output_high": 100.0,
            "vclass": None,
        },
        "label": "Fuzzy logic (Ren & Karimi)",
    },
    "hmm": {
        "module": "HMM",
        "cls": "HMMMatcher",
        "kwargs": {
            "search_radius": 50.0,
            "sigma_default": 4.07,
            "beta": 0.2,
            "max_candidates": 5,
            "use_accuracy": True,
            "min_sigma": 1.0,
            "vclass": None,
        },
        "label": "Hidden Markov Model (Newson & Krumm)",
    },
    "st": {
        "module": "STMatching",
        "cls": "STMatcher",
        "kwargs": {
            "search_radius": 50.0,
            "sigma": 20.0,
            "window_size": 8,
            "max_candidates": 5,
            "temporal_mode": "lou",
            "speed_reference": None,
            "nominal_dt": NOMINAL_DT,
            "vclass": None,
        },
        "label": "ST-Matching (Lou et al.)",
    },
}

# Accepted on the command line in addition to the canonical keys above.
METHOD_ALIASES = {
    "geometric": "native",
    "base": "native",
    "sumo": "native",
    "topological": "topo",
    "fuzzylogic": "fuzzy",
    "stmatching": "st",
    "st-matching": "st",
}

# ---- Run-time state, all set once in main() and read by process_point ----
METHOD_KEY = None
METHOD_CFG = None
MATCHER = None
MATCH_PARAMS = set()
KALMAN = None
USED_KWARGS = {}
_LAST_FIX_TIME = None


def resolve_method(name):
    """Normalise a user-supplied method name to a registry key. Case- and
    alias-insensitive, and raises with the full list of valid names rather
    than failing obscurely later."""
    key = str(name).strip().lower()
    key = METHOD_ALIASES.get(key, key)
    if key not in METHODS:
        valid = ", ".join(METHODS.keys())
        raise ValueError(f"Unknown method '{name}'. Choose one of: {valid}")
    return key


def prompt_for_method():
    """Interactive fallback when --method is omitted. Accepts either the
    menu number or the method name; re-prompts on bad input."""
    keys = list(METHODS.keys())

    print("\n" + "=" * 62)
    print(" Post-processed map matching -- select method")
    print("=" * 62)
    for i, k in enumerate(keys, start=1):
        print(f"  {i}. {k:<8} {METHODS[k]['label']}")
    print("=" * 62)

    while True:
        choice = input("Method [number or name]: ").strip()

        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(keys):
                return keys[idx - 1]
            print(f"  -> pick 1..{len(keys)}")
            continue

        try:
            return resolve_method(choice)
        except ValueError as e:
            print(f"  -> {e}")


def prompt_for_kalman():
    """Interactive on/off choice for the Kalman pre-filter, shown right
    after the method menu whenever neither --kalman nor --no-kalman was
    given. Defaults to OFF on a bare Enter, same as the live script, so the
    unfiltered CMP number stays the safer default."""
    print("\n" + "=" * 62)
    print(" Kalman pre-filter (smooths GPS before matching)")
    print("=" * 62)
    print("  1. off   raw GPS straight into the matcher  [default]")
    print("  2. on    constant-velocity filter first")
    print("=" * 62)

    while True:
        choice = input("Kalman [1/2, y/n, Enter=off]: ").strip().lower()

        if choice == "":
            return False
        if choice in ("1", "n", "no", "off", "false"):
            return False
        if choice in ("2", "y", "yes", "on", "true"):
            return True
        print("  -> answer 1/2, y/n, or press Enter for off")


def _resolve_class(module, preferred, method_name):
    """Find the matcher class inside an imported module: the registry name
    if it exists there, otherwise the sole class in the module that defines
    `method_name` (reported as a substitution). Fails loudly if there's
    zero or several plausible classes rather than guessing wrong."""
    if hasattr(module, preferred):
        return getattr(module, preferred)

    found = [
        obj for name, obj in vars(module).items()
        if inspect.isclass(obj)
        and obj.__module__ == module.__name__
        and callable(getattr(obj, method_name, None))
    ]

    if len(found) == 1:
        print(f"[WARN] '{module.__name__}' has no class '{preferred}'; "
              f"using '{found[0].__name__}' instead "
              f"(the only class with a {method_name}() method).")
        return found[0]

    names = ", ".join(sorted(c.__name__ for c in found)) or "none"
    raise AttributeError(
        f"'{module.__name__}' has no class '{preferred}', and the fallback "
        f"could not pick one unambiguously.\n"
        f"       Classes with a {method_name}() method: {names}\n"
        f"       Set the real name in the METHODS registry."
    )


def build_matcher(method_key, net_file):
    """Import and instantiate the matcher class for `method_key`. Returns
    None for the native baseline. Constructor kwargs are filtered against
    the class's actual signature (dropped ones are printed, not silently
    ignored) since the five matchers don't all take the same tuning
    parameters."""
    cfg = METHODS[method_key]

    if cfg["module"] is None:
        return None

    try:
        module = __import__(cfg["module"], fromlist=[cfg["cls"]])
    except ImportError as e:
        raise ImportError(
            f"Could not import '{cfg['module']}' for method '{method_key}': {e}\n"
            f"       Expected {cfg['module']}.py alongside this script in\n"
            f"       {SCRIPT_DIR}"
        ) from e

    cls = _resolve_class(module, cfg["cls"], "match")

    accepted = set(inspect.signature(cls.__init__).parameters)
    kwargs = {k: v for k, v in cfg["kwargs"].items() if k in accepted}
    dropped = sorted(set(cfg["kwargs"]) - set(kwargs))
    if dropped:
        print(f"[WARN] {cfg['cls']} does not accept: {', '.join(dropped)} "
              f"-- these settings were IGNORED for this run.")

    global MATCH_PARAMS, USED_KWARGS
    MATCH_PARAMS = set(inspect.signature(cls.match).parameters)
    USED_KWARGS = kwargs

    print(f"[INFO] Loading network into {cfg['cls']}...")
    return cls(net_file, **kwargs)


def build_kalman():
    """Build the optional Kalman pre-filter used by --kalman. Same module
    (kalman_filter.py) and same reasoning as the live script: a missing
    module is a hard error, not a silent disable, since a run labelled
    'kalman' that quietly didn't filter anything would be worse than no
    run at all."""
    try:
        from kalman_filter import KalmanFilter
    except ImportError as e:
        raise ImportError(
            f"--kalman requested but kalman_filter.py could not be imported: {e}\n"
            f"       Expected it alongside this script in {SCRIPT_DIR}"
        ) from e

    return KalmanFilter(
        process_noise=1.0,
        sigma_default=4.07,
        use_accuracy=True,
        min_sigma=1.0,
        min_dt=0.05,
        max_dt=MAX_KALMAN_DT,
    )


def parse_sumocfg_for_netfile(sumocfg_path: str) -> str:
    """Read the .sumocfg XML and return the absolute path to its
    <net-file>, resolved relative to the config's own directory."""
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


def parse_course_deg(course_deg_raw):
    """Return a valid heading in degrees, or None if unusable (negative /
    missing / unparseable). Mirrors the live script's parse_course_deg,
    minus the INVALID_DOUBLE_VALUE sentinel translation -- there's no
    moveToXY call here to hand that sentinel to, so this returns the plain
    float-or-None form directly."""
    try:
        if course_deg_raw in (None, ""):
            return None
        course_deg = float(course_deg_raw)
        if course_deg < 0:
            return None
        return course_deg % 360.0
    except Exception:
        return None


def parse_fix_time(value):
    """Convert a CSV timestamp value into epoch seconds (float). Accepts
    either a bare epoch-seconds number or an ISO-8601 string (as the
    phone/Flask side of the live script produces). Returns None if
    unparseable or empty, in which case callers fall back to NOMINAL_DT."""
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        pass
    try:
        return datetime.fromisoformat(str(value).replace("Z", "+00:00")).timestamp()
    except Exception:
        return None


def _fmt(value, spec=".3f"):
    """Format one value: floats to `spec`, anything else as-is. Matchers
    report non-numeric score components too (FuzzyMatcher's mode string,
    HMMMatcher's candidate count), so this tolerates those rather than
    raising on a bad format spec."""
    if value is None or value == "":
        return "--"
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, (int, float)):
        try:
            return format(value, spec)
        except (ValueError, TypeError):
            return str(value)
    return str(value)


def _fmt_components(comps):
    """Render a matcher's score components as 'name=value' pairs, built
    from whatever keys it returned since the five methods report different
    terms."""
    if not comps:
        return ""
    return " ".join(f"{k}={_fmt(v)}" for k, v in comps.items())


def native_match(x, y):
    """Baseline 'matcher': ask SUMO which edge this point is on via
    convertRoad. Returns a dict shaped like the real matchers' results (so
    process_point doesn't need to know which method is running), plus a
    'lane_pos' entry the custom matchers don't need but native matching
    does -- see snap_to_lane_geometry() for why: unlike the live script,
    there's no moveToXY here to do the actual on-lane snapping, so that
    has to be done independently via sumolib using edge_id/lane_index/
    lane_pos.

    Called on the (possibly Kalman-filtered) NETWORK coordinates with
    isGeo=False, not on the raw lat/lon, so a --kalman native run looks up
    its edge from the same position it reports as matched -- the same
    reasoning the live script uses.
    """
    try:
        edge_id, lane_pos, lane_index = traci.simulation.convertRoad(x, y, isGeo=False)
    except traci.TraCIException as e:
        print(f"[WARN] convertRoad failed: {e}")
        return None

    if not edge_id:
        return None

    return {
        "x": x,
        "y": y,
        "edge_id": edge_id,
        "lane_index": lane_index,
        "lane_pos": lane_pos,
        "raw_dist": None,
        "score": None,
        "components": {},
        "window_len": None,
    }


def snap_to_lane_geometry(net, edge_id, lane_index, lane_pos):
    """Look up the actual on-lane (x, y) for an edge/lane/pos using the
    static network geometry (sumolib), same technique
    post_processed_matching_v1.py used. Only needed for native matching:
    the custom matchers already return a proper on-edge (x, y) directly
    from their own match() call. Returns None on failure."""
    try:
        lane = net.getLane(f"{edge_id}_{lane_index}")
        shape = lane.getShape()
        return sumolib.geomhelper.positionAtShapeOffset(shape, lane_pos)
    except Exception as e:
        print(f"[WARN] Could not compute snapped geometry for edge {edge_id}: {e}")
        return None


def process_point(net, lat, lon, speed_mps, course_deg_raw, fix_time, accuracy_m):
    """
    Convert one recorded GPS point to SUMO coordinates, run the selected
    matcher (optionally after Kalman pre-filtering), and return a dict of
    everything needed for the output CSV.

    This is move_vehicle_to_phone_position() from the live script with the
    vehicle taken out: no spawning, no moveToXY, no reading back a
    simulated position, because there's no animated bike to place -- only
    the geometry of "where does this point match to" is wanted here.
    """
    global _LAST_FIX_TIME

    row = {
        "matched": False,
        "edge_id": "",
        "lane_index": "",
        "lane_pos": "",
        "is_internal_edge": False,
        "raw_x": "",
        "raw_y": "",
        "filt_x": "",
        "filt_y": "",
        "matched_x": "",
        "matched_y": "",
        "matched_lat": "",
        "matched_lon": "",
        "match_error_raw_m": "",
        "match_error_filt_m": "",
        "raw_dist": "",
        "score": "",
        "components": None,
        "window_len": "",
        "match_ms": "",
        "unmatched_reason": "",
    }

    try:
        x, y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
    except traci.TraCIException as e:
        print(f"[WARN] convertGeo failed for ({lat}, {lon}): {e}")
        row["unmatched_reason"] = "convertGeo_failed"
        return row

    row["raw_x"], row["raw_y"] = x, y

    # Heading, sanitised once up front. Computed before the Kalman filter
    # runs since the filter seeds its initial velocity from it.
    course_for_match = parse_course_deg(course_deg_raw)

    # ---- Optional Kalman pre-filter -----------------------------------
    # Applied to the converted (x, y), in metres, same as the live script,
    # so the constant-velocity model's units match. Runs identically ahead
    # of all five methods.
    filt_x, filt_y = x, y
    if KALMAN is not None:
        if fix_time is not None and _LAST_FIX_TIME is not None:
            dt = fix_time - _LAST_FIX_TIME
        else:
            dt = NOMINAL_DT

        try:
            kf = KALMAN.update(
                x, y, dt,
                speed_mps=speed_mps,
                course_deg=course_for_match,
                accuracy_m=accuracy_m,
            )
            filt_x, filt_y = kf["x"], kf["y"]
        except Exception as e:
            print(f"[WARN] Kalman filter failed, using raw point: {e}")
            filt_x, filt_y = x, y

    if fix_time is not None:
        _LAST_FIX_TIME = fix_time

    row["filt_x"], row["filt_y"] = filt_x, filt_y

    # ---- Run the selected matcher -------------------------------------
    match_start = time.perf_counter()
    try:
        if MATCHER is None:
            match_result = native_match(filt_x, filt_y)
        else:
            call_kwargs = {
                "timestamp": fix_time,        # ST-Matching only
                "speed_mps": speed_mps,       # all but ST's core scoring
                "course_deg": course_for_match,
                "accuracy_m": accuracy_m,     # HMM and fuzzy declare it
            }
            call_kwargs = {k: v for k, v in call_kwargs.items()
                           if k in MATCH_PARAMS}
            match_result = MATCHER.match(filt_x, filt_y, **call_kwargs)
    except Exception:
        print("[ERROR] Matcher raised:")
        traceback.print_exc()
        match_result = None
    row["match_ms"] = (time.perf_counter() - match_start) * 1000.0

    # No match -> this point simply doesn't get a matched edge. Unlike the
    # live loop there's no vehicle position to hold onto; we just record
    # the miss and move to the next point.
    if match_result is None:
        row["unmatched_reason"] = "no_match"
        return row

    edge_id = match_result["edge_id"]
    lane_index = match_result.get("lane_index", 0)

    row["matched"] = True
    row["edge_id"] = edge_id
    row["lane_index"] = lane_index
    row["is_internal_edge"] = bool(edge_id) and edge_id.startswith(":")
    row["raw_dist"] = match_result.get("raw_dist")
    row["score"] = match_result.get("score")
    row["components"] = match_result.get("components")
    row["window_len"] = match_result.get("window_len")

    if MATCHER is None:
        # Native: convertRoad only gave us edge/lane/pos, not a point --
        # go to the network geometry ourselves (see snap_to_lane_geometry).
        row["lane_pos"] = match_result.get("lane_pos", "")
        snapped = snap_to_lane_geometry(
            net, edge_id, lane_index, match_result.get("lane_pos", 0.0)
        )
        if snapped is not None:
            move_x, move_y = snapped
        else:
            move_x, move_y = match_result["x"], match_result["y"]
            row["unmatched_reason"] = "geometry_lookup_failed"
    else:
        # The custom matchers already computed a proper on-edge point.
        move_x, move_y = match_result["x"], match_result["y"]
        row["lane_pos"] = match_result.get("lane_pos", "")

    row["matched_x"], row["matched_y"] = move_x, move_y

    try:
        matched_lon, matched_lat = net.convertXY2LonLat(move_x, move_y)
        row["matched_lat"], row["matched_lon"] = matched_lat, matched_lon
    except Exception as e:
        print(f"[WARN] Could not back-project matched point to lat/lon: {e}")

    # How far the matched point is from the raw GPS fix, and separately
    # from the (possibly filtered) point actually fed to the matcher.
    # Large values aren't automatically errors -- correcting noise is the
    # point -- but they're the numbers Newson-Krumm style CMP scoring
    # needs.
    row["match_error_raw_m"] = math.hypot(move_x - x, move_y - y)
    row["match_error_filt_m"] = math.hypot(move_x - filt_x, move_y - filt_y)

    return row


def load_gps_rows(csv_path, lat_col, lon_col, timestamp_col, speed_col,
                   course_col, accuracy_col):
    """Read a generic GPS CSV and return a list of dicts with the raw row
    plus parsed lat/lon/timestamp/speed/course/accuracy. Extra columns are
    preserved and passed through to the output untouched. Any of the
    optional columns can be absent -- the matchers and Kalman filter all
    tolerate None for these, just with a less-informed result (nominal dt
    instead of real elapsed time, default GPS sigma instead of a per-fix
    one, etc.)."""
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Could not read header row from {csv_path}")

        missing = [c for c in (lat_col, lon_col) if c not in reader.fieldnames]
        if missing:
            raise ValueError(
                f"Input CSV is missing expected column(s) {missing}. "
                f"Found columns: {reader.fieldnames}. "
                f"Use --lat-col/--lon-col to point at the right columns."
            )

        have_ts = bool(timestamp_col) and timestamp_col in reader.fieldnames
        have_speed = bool(speed_col) and speed_col in reader.fieldnames
        have_course = bool(course_col) and course_col in reader.fieldnames
        have_accuracy = bool(accuracy_col) and accuracy_col in reader.fieldnames

        if timestamp_col and not have_ts:
            print(f"[INFO] Timestamp column '{timestamp_col}' not found; "
                  f"ST-Matching/HMM temporal terms and the Kalman filter "
                  f"will use a nominal dt of {NOMINAL_DT}s.")
        if speed_col and not have_speed:
            print(f"[INFO] Speed column '{speed_col}' not found; speed will "
                  f"be treated as 0 for every point.")
        if course_col and not have_course:
            print(f"[INFO] Course column '{course_col}' not found; heading "
                  f"will be treated as unavailable for every point.")
        if accuracy_col and not have_accuracy:
            print(f"[INFO] Accuracy column '{accuracy_col}' not found; "
                  f"matchers/Kalman will fall back to their default GPS sigma.")

        for raw_row in reader:
            try:
                lat = float(raw_row[lat_col])
                lon = float(raw_row[lon_col])
            except (TypeError, ValueError):
                print(f"[WARN] Skipping unparsable row: {raw_row}")
                continue

            fix_time = parse_fix_time(raw_row.get(timestamp_col)) if have_ts else None

            try:
                speed_raw = raw_row.get(speed_col) if have_speed else None
                speed_mps = float(speed_raw) if speed_raw not in (None, "") else 0.0
            except (TypeError, ValueError):
                speed_mps = 0.0

            course_raw = raw_row.get(course_col) if have_course else None
            course_deg = None if course_raw in (None, "") else course_raw

            try:
                accuracy_raw = raw_row.get(accuracy_col) if have_accuracy else None
                accuracy_m = float(accuracy_raw) if accuracy_raw not in (None, "") else None
            except (TypeError, ValueError):
                accuracy_m = None

            rows.append({
                "raw": raw_row,
                "lat": lat,
                "lon": lon,
                "fix_time": fix_time,
                "speed_mps": speed_mps,
                "course_deg": course_deg,
                "accuracy_m": accuracy_m,
            })

    return rows


def default_output_path(method_key, use_kalman):
    """Same naming idea as the live script's open_run_log: one file per
    run, named by method/Kalman/start-time, so repeated runs never
    overwrite each other."""
    out_dir = os.path.join(SCRIPT_DIR, OUTPUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    suffix = "_kalman" if use_kalman else ""
    return os.path.join(out_dir, f"matched_{method_key}{suffix}_{stamp}.csv")


def parse_args():
    """Command-line interface. Mirrors live_phone_to_sumo.py's --method /
    --kalman / --no-kalman UI exactly; everything else is post-processing
    I/O plumbing in the spirit of post_processed_matching_v1.py."""
    p = argparse.ArgumentParser(
        description="Post-process a CSV of recorded GPS points through SUMO "
                     "map matching, with a choice of five methods and an "
                     "optional Kalman pre-filter.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Methods:\n" + "\n".join(
            f"  {k:<8} {v['label']}" for k, v in METHODS.items()
        ),
    )
    p.add_argument("-m", "--method", default=None,
                   help="matching method to run; prompts if omitted")
    # Tri-state on purpose, same as the live script: None triggers the
    # interactive prompt, while either flag skips it (so a scripted batch
    # of runs -- e.g. all five methods x on/off -- never blocks on input).
    p.add_argument("--kalman", dest="kalman", action="store_true", default=None,
                   help="apply the Kalman pre-filter before matching")
    p.add_argument("--no-kalman", dest="kalman", action="store_false",
                   help="skip the Kalman pre-filter (no prompt)")
    p.add_argument("--sumocfg", default=os.path.join(SCRIPT_DIR, SUMO_CFG),
                   help="Path to .sumocfg file")
    p.add_argument("--input", default=os.path.join(SCRIPT_DIR, DEFAULT_INPUT_CSV),
                   help="Input GPS CSV path")
    p.add_argument("--output", default=None,
                   help="Output CSV path (default: Data/matched_<method>"
                        "[_kalman]_<timestamp>.csv)")
    p.add_argument("--lat-col", default=DEFAULT_LAT_COL, help="Name of the latitude column")
    p.add_argument("--lon-col", default=DEFAULT_LON_COL, help="Name of the longitude column")
    p.add_argument("--timestamp-col", default=DEFAULT_TIMESTAMP_COL,
                   help="Column with each fix's time (epoch seconds or "
                        "ISO-8601); pass '' to disable")
    p.add_argument("--speed-col", default=DEFAULT_SPEED_COL,
                   help="Column with each fix's speed in m/s; pass '' to disable")
    p.add_argument("--course-col", default=DEFAULT_COURSE_COL,
                   help="Column with each fix's heading in degrees; pass '' to disable")
    p.add_argument("--accuracy-col", default=DEFAULT_ACCURACY_COL,
                   help="Column with each fix's horizontal accuracy in m; "
                        "pass '' to disable")
    p.add_argument("--match-threshold", type=float, default=MATCH_THRESHOLD,
                   help="Informational only -- see MATCH_THRESHOLD note in the source")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="print a per-point match/skip line, not just periodic progress")
    return p.parse_args()


def main():
    global METHOD_KEY, METHOD_CFG, MATCHER, KALMAN

    args = parse_args()

    METHOD_KEY = resolve_method(args.method) if args.method else prompt_for_method()
    METHOD_CFG = METHODS[METHOD_KEY]

    use_kalman = args.kalman if args.kalman is not None else prompt_for_kalman()

    sumocfg_abs = os.path.abspath(args.sumocfg)
    if not os.path.exists(sumocfg_abs):
        raise FileNotFoundError(f"SUMO config not found: {sumocfg_abs}")

    net_file = parse_sumocfg_for_netfile(sumocfg_abs)
    output_path = args.output or default_output_path(METHOD_KEY, use_kalman)

    print("\n" + "-" * 62)
    print(f"[INFO] Method:      {METHOD_KEY} -- {METHOD_CFG['label']}")
    print(f"[INFO] Kalman:      {'on' if use_kalman else 'off'}")
    print(f"[INFO] SUMO config: {sumocfg_abs}")
    print(f"[INFO] Net file:    {net_file}")
    print(f"[INFO] Input CSV:   {args.input}")
    print(f"[INFO] Output CSV:  {output_path}")
    print("-" * 62)

    print("[INFO] Loading network geometry with sumolib...")
    # withInternal=True so junction-internal edges (IDs starting with ':')
    # have usable lane geometry -- otherwise every match landing on one
    # during a turn would silently fail to produce coordinates.
    net = sumolib.net.readNet(net_file, withInternal=True)

    # Build the matcher ONCE and reuse it for every point.
    MATCHER = build_matcher(METHOD_KEY, net_file)
    if MATCHER is None:
        print("[INFO] Native SUMO matching -- no external matcher loaded.")
    else:
        shown = ", ".join(f"{k}={v}" for k, v in USED_KWARGS.items())
        print(f"[INFO] {type(MATCHER).__name__} ready ({shown}).")

    # Built after the matcher so a missing kalman_filter.py fails before
    # the (potentially slow) CSV load and SUMO startup.
    if use_kalman:
        KALMAN = build_kalman()
        print("[INFO] Kalman pre-filter active.")

    print("[INFO] Loading GPS points...")
    gps_rows = load_gps_rows(
        args.input, args.lat_col, args.lon_col,
        args.timestamp_col, args.speed_col, args.course_col, args.accuracy_col,
    )
    print(f"[INFO] Loaded {len(gps_rows)} GPS points.")

    if not gps_rows:
        print("[WARN] No usable GPS points loaded; nothing to do.")
        return

    # ST-Matching's fixed-lag Viterbi window, the fuzzy matcher's
    # confirm_count debounce, and the Kalman filter's constant-velocity
    # prediction all assume points arrive in chronological order. Sort by
    # timestamp when every point has one; otherwise trust the CSV's own
    # ordering rather than guessing.
    have_all_ts = all(r["fix_time"] is not None for r in gps_rows)
    have_any_ts = any(r["fix_time"] is not None for r in gps_rows)
    if have_all_ts:
        gps_rows.sort(key=lambda r: r["fix_time"])
    elif have_any_ts:
        print("[WARN] Some rows are missing a parseable timestamp; leaving "
              "GPS points in their original CSV order rather than sorting "
              "by time.")

    print("[INFO] Starting headless SUMO for coordinate conversion"
          + (" / native matching..." if MATCHER is None else "..."))
    traci.start(["sumo", "-c", sumocfg_abs, "--start", "--no-step-log", "true"])

    output_rows = []
    matched_count = 0
    internal_count = 0
    reason_counts = {}

    try:
        for i, row in enumerate(gps_rows):
            res = process_point(
                net, row["lat"], row["lon"], row["speed_mps"],
                row["course_deg"], row["fix_time"], row["accuracy_m"],
            )

            if res["matched"]:
                matched_count += 1
            if res["is_internal_edge"]:
                internal_count += 1
            if not res["matched"]:
                reason = res["unmatched_reason"] or "unknown"
                reason_counts[reason] = reason_counts.get(reason, 0) + 1

            out_row = dict(row["raw"])  # preserve any extra original columns
            out_row.update({
                args.lat_col: row["lat"],
                args.lon_col: row["lon"],
                "method": METHOD_KEY,
                "kalman": use_kalman,
                "matched": res["matched"],
                "edge_id": res["edge_id"],
                "lane_index": res["lane_index"],
                "lane_pos": res["lane_pos"],
                "is_internal_edge": res["is_internal_edge"],
                "raw_x": res["raw_x"],
                "raw_y": res["raw_y"],
                "filt_x": res["filt_x"],
                "filt_y": res["filt_y"],
                "matched_x": res["matched_x"],
                "matched_y": res["matched_y"],
                "matched_lat": res["matched_lat"],
                "matched_lon": res["matched_lon"],
                "match_error_raw_m": res["match_error_raw_m"],
                "match_error_filt_m": res["match_error_filt_m"],
                "raw_dist": res["raw_dist"],
                "score": res["score"],
                "components": _fmt_components(res["components"]),
                "window_len": res["window_len"],
                "match_ms": res["match_ms"],
                "phone_speed_mps": row["speed_mps"],
                "phone_course_deg": row["course_deg"],
                "accuracy_m": row["accuracy_m"],
                "unmatched_reason": res["unmatched_reason"],
            })
            output_rows.append(out_row)

            if args.verbose:
                if res["matched"]:
                    print(
                        f"[MATCH][{METHOD_KEY}] #{i + 1} edge={res['edge_id']} "
                        f"raw=({_fmt(res['raw_x'])}, {_fmt(res['raw_y'])}) "
                        f"matched=({_fmt(res['matched_x'])}, {_fmt(res['matched_y'])}) "
                        + (f"win={res['window_len']} " if res['window_len'] is not None else "")
                        + (_fmt_components(res['components']) + " " if res['components'] else "")
                        + f"match={_fmt(res['match_ms'], '.1f')} ms"
                    )
                else:
                    print(f"[SKIP][{METHOD_KEY}] #{i + 1} "
                          f"reason={res['unmatched_reason'] or 'unknown'}")

            if (i + 1) % 50 == 0 or (i + 1) == len(gps_rows):
                print(f"[INFO] Processed {i + 1}/{len(gps_rows)} points "
                      f"({matched_count} matched so far)")

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user -- writing out what was processed so far.")
    finally:
        try:
            traci.close()
        except Exception:
            pass

    if not output_rows:
        print("[WARN] No output rows produced; nothing written.")
        return

    fieldnames = list(output_rows[0].keys())
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(output_rows)

    print(
        f"[DONE] Wrote {len(output_rows)} rows to {output_path} "
        f"({matched_count}/{len(output_rows)} points matched to an edge, "
        f"{internal_count} of which landed on a junction/internal edge)."
    )
    if reason_counts:
        print("[INFO] Breakdown of unmatched points:")
        for reason, count in sorted(reason_counts.items(), key=lambda kv: -kv[1]):
            print(f"         {reason}: {count}")


if __name__ == "__main__":
    main()
