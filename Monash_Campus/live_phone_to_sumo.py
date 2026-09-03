"""
live_phone_to_sumo.py
======================================================================
eBike-in-the-Loop live bridge -- UNIFIED MULTI-METHOD RUNNER.

One script, five map-matching methods. Which one runs is chosen at
launch, either as a command-line flag or from an interactive menu:

    python live_phone_to_sumo.py --method st
    python live_phone_to_sumo.py --method native --no-gui
    python live_phone_to_sumo.py                 (prompts for a method)

Methods:
    native  -- SUMO's own moveToXY snapping (the geometric baseline)
    topo    -- TopologicalMatcher   (topological.py)
    fuzzy   -- FuzzyMatcher         (FuzzyLogic.py)
    hmm     -- HMMMatcher           (HMM.py)
    st      -- STMatcher            (STMatching.py)

The loop itself does three jobs, unchanged from the single-method
version:

    1. Poll the Flask server for the newest GPS fix the phone posted.
    2. Convert that lat/lon into SUMO network (x, y) coordinates and hand
       it to whichever matcher was selected.
    3. Move the virtual eBike to the matched position inside a running
       SUMO simulation via traci.vehicle.moveToXY().

Data flow for one update:

    phone GPS --(HTTP POST)--> Flask /update
                                  |
                                  v
    this script --(HTTP GET)--> Flask /latest --> convertGeo() --> (x, y)
                                  |
                                  v
              [optional Kalman pre-filter] --> MATCHER.match(...)
                                  |
                                  v
                        traci.vehicle.moveToXY(...)  --> eBike moves

WHY ONE SCRIPT INSTEAD OF FIVE
------------------------------
Every difference between the five runs that is NOT the matcher itself is
a confound in the CMP comparison: polling interval, staleness cutoff,
spawn behaviour, speed handling, keepRoute, logging. Holding all of that
in one file means the five methods are genuinely compared under
identical conditions, and a fix to the harness applies to all of them at
once. Method-specific settings live in exactly one place, the METHODS
registry below.

NO FALLBACK TO NATIVE MATCHING. If a matcher returns nothing, the update
is SKIPPED entirely and the bike holds its last position. Falling back to
SUMO's own snapping would silently mix native results into the method's
CMP score and make the five-method comparison meaningless. A skipped fix
is visible in the logs; a contaminated one is not. (The "native" method
is the one exception, because there the native result IS the method.)

TIMESTAMPS: ST-Matching needs the elapsed time between fixes for its
temporal term, so the phone's own fix time is parsed and passed through
to every matcher (harmless to those that ignore it). Phone time is used
rather than arrival time because the temporal term is about how fast the
RIDER moved, not how fast the network delivered the packet.

CSV LOG: --log writes one row per fix with the raw converted GPS point,
the matcher's chosen point, and the position SUMO actually ended up
placing the bike at. Those three columns are what the CMP script needs;
recording them live avoids re-deriving them from console text later.
----------------------------------------------------------------------
"""

import os
import sys
import csv
import time
import inspect
import argparse
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
SUMO_DELAY_MS = "1000"     # GUI pacing, handled by SUMO's own event loop

# Directory for the per-run CSV logs written when --log is passed.
LOG_DIR = "runs"
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

# Anchor relative paths to THIS FILE's directory, not the working directory,
# so the script runs correctly regardless of where it was launched from.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# The script's own directory must also be importable, otherwise launching
# from elsewhere breaks "from topological import ...".
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


# ======================================================================
# METHOD REGISTRY
# ======================================================================
# One entry per map-matching method. This is the ONLY place that knows
# anything method-specific, so adding the remaining matchers is a matter
# of filling in a module name, a class name and a kwargs dict -- no
# changes anywhere else in the file.
#
#   module     : python module to import the matcher class from, or None
#                for the native baseline (which uses no matcher object).
#   cls        : class name inside that module.
#   kwargs     : constructor arguments. Unsupported keys are dropped
#                automatically (see build_matcher), so a matcher whose
#                constructor doesn't take, say, "sigma" is fine.
#   keep_route : the moveToXY keepRoute bitmask used for this method.
#                  0 = let SUMO re-snap geometrically  -> native only
#                  2 = exact placement, matcher decision preserved
#                  6 = bits 1+2, exact placement + lane-permission bypass
#                Native must stay at 0: re-snapping IS the baseline being
#                measured. The matchers use 6 rather than the nominally
#                correct 2 because bare keepRoute=2 can leave the vehicle
#                lane-less and trigger the sumo-gui freeze in upstream
#                SUMO issue #10974; bit 2 bypasses the lane permission
#                check without touching the placed coordinates.
#   label      : human-readable name for logs and the CSV filename.

METHODS = {
    "native": {
        "module": None,
        "cls": None,
        "kwargs": {},
        "keep_route": 0,
        "label": "Geometric (SUMO native moveToXY)",
    },
    "topo": {
        "module": "topological",
        "cls": "TopologicalMatcher",
        "kwargs": {
            "search_radius": 50.0,          # m; hard cutoff for candidate lookup
            "sigma_prox": 15.0,             # m; proximity score decay scale
            # S(e) = w1*prox + w2*head + w3*conn + w4*turn, passed as one
            # 4-tuple in the order (prox, head, conn, turn). These are the
            # knobs to sweep when tuning the topological method.
            "weights": (0.40, 0.30, 0.20, 0.10),
            "min_speed_for_heading": 0.5,   # m/s; below this heading is ignored
            "vclass": None,                 # e.g. "bicycle" to reject disallowed edges
        },
        "keep_route": 6,
        "label": "Topological (weighted, Velaga et al.)",
    },
    "fuzzy": {
        "module": "FuzzyLogic",
        "cls": "FuzzyMatcher",
        "kwargs": {
            "search_radius": 50.0,        # m; hard cutoff for candidate lookup
            "dist_half_width": 2.5,       # m; distance at which short/long = 0.5
            "angle_small_break": 25.0,    # deg; below this "small" membership = 1
            "angle_large_break": 65.0,    # deg; above this "small" membership = 0
            "junction_threshold": 5.0,    # m; clearance before "entering" mode
            "min_speed_for_switch": 0.5,  # m/s; below this the edge never changes
            "confirm_count": 2,           # fixes a new edge must win to take over
            "output_low": 10.0,           # crisp rule outputs for the
            "output_average": 50.0,       # weighted-average defuzzification
            "output_high": 100.0,
            "vclass": None,               # e.g. "bicycle" to reject disallowed edges
        },
        "keep_route": 6,
        "label": "Fuzzy logic (Ren & Karimi)",
    },
    "hmm": {
        "module": "HMM",
        "cls": "HMMMatcher",
        "kwargs": {
            "search_radius": 50.0,    # m; hard cutoff for candidate lookup
            "sigma_default": 4.07,    # m; Newson & Krumm's GPS error std,
                                      # used when the phone reports no accuracy
            "beta": 0.2,              # detour tolerance in the transition term
            "max_candidates": 5,      # transition cost is quadratic in this
            "use_accuracy": True,     # prefer the phone's per-fix accuracy_m
            "min_sigma": 1.0,         # m; floor on that accuracy reading
            "vclass": None,           # e.g. "bicycle" to reject disallowed edges
        },
        "keep_route": 6,
        "label": "Hidden Markov Model (Newson & Krumm)",
    },
    "st": {
        "module": "STMatching",
        "cls": "STMatcher",
        "kwargs": {
            "search_radius": 50.0,    # m; hard cutoff for candidate lookup
            "sigma": 20.0,            # m; GPS error std for observation prob
            "window_size": 8,         # fixes in the fixed-lag Viterbi window
            "max_candidates": 5,      # transition cost is quadratic in this
            "temporal_mode": "lou",   # "lou" | "ratio" | "off"
            "speed_reference": None,  # m/s; rescales car speed limits
            "nominal_dt": POLL_INTERVAL,
            "vclass": None,           # e.g. "bicycle" to reject disallowed edges
        },
        "keep_route": 6,
        "label": "ST-Matching (Lou et al.)",
    },
}

# Accepted on the command line in addition to the canonical keys above,
# so "--method topological" or "--method geometric" also work.
METHOD_ALIASES = {
    "geometric": "native",
    "base": "native",
    "sumo": "native",
    "topological": "topo",
    "fuzzylogic": "fuzzy",
    "stmatching": "st",
    "st-matching": "st",
}

# ---- Run-time state, all set once in main() and read by the loop ----
METHOD_KEY = None     # which registry entry is active
METHOD_CFG = None     # the registry dict for that entry
MATCHER = None        # the matcher instance, or None for native
MATCH_PARAMS = set()  # parameter names the active matcher's match() accepts
KALMAN = None         # optional pre-filter instance, or None
CSV_WRITER = None     # csv.writer for the run log, or None
CSV_FILE = None       # the underlying file handle, closed on shutdown
USED_KWARGS = {}      # constructor kwargs the matcher actually accepted
_LAST_FIX_TIME = None # epoch seconds of the previous fix, for the Kalman dt


def resolve_method(name):
    """
    Normalise a user-supplied method name to a registry key.

    Case-insensitive and alias-aware so "ST", "st-matching" and
    "STMatching" all land on the same entry. Raises with the full list of
    valid names rather than failing obscurely later.
    """
    key = str(name).strip().lower()
    key = METHOD_ALIASES.get(key, key)
    if key not in METHODS:
        valid = ", ".join(METHODS.keys())
        raise ValueError(f"Unknown method '{name}'. Choose one of: {valid}")
    return key


def prompt_for_method():
    """
    Interactive fallback when --method is omitted.

    Prints a numbered menu and accepts either the number or the method
    name. Re-prompts on bad input rather than exiting, since a typo at
    launch is not worth restarting the whole run for.
    """
    keys = list(METHODS.keys())

    print("\n" + "=" * 62)
    print(" eBike-in-the-Loop -- select map-matching method")
    print("=" * 62)
    for i, k in enumerate(keys, start=1):
        print(f"  {i}. {k:<8} {METHODS[k]['label']}")
    print("=" * 62)

    while True:
        choice = input("Method [number or name]: ").strip()

        # Accept the menu number as a shortcut for the name.
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
    """
    Interactive on/off choice for the Kalman pre-filter.

    Shown right after the method menu whenever neither --kalman nor
    --no-kalman was given on the command line, because the filter is a
    per-run choice made just as often as the method itself: the results
    table has a filtered and an unfiltered row for all five methods, so
    this question comes up on every single run.

    Defaults to OFF on a bare Enter. The unfiltered number is the one each
    method's CMP is reported against, so the safer default is the one that
    doesn't quietly add a second variable to the comparison.
    """
    print("\n" + "=" * 62)
    print(" Kalman pre-filter (smooths GPS before matching)")
    print("=" * 62)
    print("  1. off   raw GPS straight into the matcher  [default]")
    print("  2. on    constant-velocity filter first")
    print("=" * 62)

    while True:
        choice = input("Kalman [1/2, y/n, Enter=off]: ").strip().lower()

        # Bare Enter takes the default rather than re-asking.
        if choice == "":
            return False
        if choice in ("1", "n", "no", "off", "false"):
            return False
        if choice in ("2", "y", "yes", "on", "true"):
            return True
        print("  -> answer 1/2, y/n, or press Enter for off")


def _resolve_class(module, preferred, method_name):
    """
    Find the matcher class inside an imported module.

    Uses the name given in the registry when it exists. If it doesn't, look
    for exactly one class in the module that defines `method_name` and use
    that instead, reporting the substitution. The five matchers were named
    independently (STMatcher, TopologicalMatcher, ...), so this saves a
    launch failure over a class name that differs from the guess in the
    registry -- while still failing loudly if the module has zero or
    several plausible classes, since silently picking one would be worse
    than not running at all.
    """
    if hasattr(module, preferred):
        return getattr(module, preferred)

    # Consider only classes DEFINED in this module (not imported into it),
    # otherwise a "from x import Y" at the top would be a false positive.
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
    """
    Import and instantiate the matcher class for `method_key`.

    Returns None for the native baseline, which has no matcher object.

    Constructor kwargs from the registry are filtered against the class's
    actual signature before being passed. That is deliberate: the five
    matchers were written at different times and do not all take the same
    tuning parameters, and a hard TypeError at launch over an unused
    keyword would be a pointless obstacle. Anything dropped is printed,
    so a silently ignored parameter can't quietly skew a run.
    """
    cfg = METHODS[method_key]

    # Native uses SUMO's own snapping; there is nothing to construct.
    if cfg["module"] is None:
        return None

    # Import late, not at module load, so a missing or half-finished
    # matcher file only breaks the run that actually selected it.
    try:
        module = __import__(cfg["module"], fromlist=[cfg["cls"]])
    except ImportError as e:
        raise ImportError(
            f"Could not import '{cfg['module']}' for method '{method_key}': {e}\n"
            f"       Expected {cfg['module']}.py alongside this script in\n"
            f"       {SCRIPT_DIR}"
        ) from e

    # Registry name first, discovery as a fallback (see _resolve_class).
    cls = _resolve_class(module, cfg["cls"], "match")

    # Keep only the kwargs this particular constructor accepts.
    accepted = set(inspect.signature(cls.__init__).parameters)
    kwargs = {k: v for k, v in cfg["kwargs"].items() if k in accepted}
    dropped = sorted(set(cfg["kwargs"]) - set(kwargs))
    if dropped:
        print(f"[WARN] {cfg['cls']} does not accept: {', '.join(dropped)} "
              f"-- these settings were IGNORED for this run.")

    # The five matchers do not all take the same OPTIONAL match() arguments
    # either: ST-Matching and the HMM need `timestamp` for their temporal
    # terms, while the topological matcher has no time term and its match()
    # signature is (x, y, course_deg, speed_mps) only. Recording the
    # accepted names here lets the live loop pass one uniform set of
    # arguments and have the irrelevant ones dropped, instead of the caller
    # needing a branch per method.
    global MATCH_PARAMS, USED_KWARGS
    MATCH_PARAMS = set(inspect.signature(cls.match).parameters)
    USED_KWARGS = kwargs

    print(f"[INFO] Loading network into {cfg['cls']}...")
    return cls(net_file, **kwargs)


def build_kalman():
    """
    Build the optional Kalman pre-filter used by --kalman.

    Kept as a separate module (kalman_filter.py) because the paper treats it
    as a pre-processing stage applicable to all five matchers, not as part
    of any one of them. A missing module is a hard error rather than a
    silent disable: a run labelled "kalman" in the results table that
    quietly didn't filter anything would be worse than no run at all.

    Returns a KalmanFilter instance. Its update() takes the raw (x, y), the
    elapsed time since the previous fix, and the phone's speed/heading/
    accuracy, and returns a dict of filtered values -- so it is called
    directly rather than through an adapter.
    """
    try:
        from kalman_filter import KalmanFilter
    except ImportError as e:
        raise ImportError(
            f"--kalman requested but kalman_filter.py could not be imported: {e}\n"
            f"       Expected it alongside this script in {SCRIPT_DIR}"
        ) from e

    return KalmanFilter(
        process_noise=1.0,       # (m/s^2)^2; higher = follow the GPS more
                                 # closely, lower = smoother but laggier
        sigma_default=4.07,      # m; same constant HMM.py uses
        use_accuracy=True,       # prefer the phone's per-fix accuracy_m
        min_sigma=1.0,           # m; floor on that reading
        min_dt=0.05,             # s; numerical safety floor on dt
        max_dt=STALE_DATA_SECONDS,  # s; a long gap must not blow up P
    )


def open_run_log(method_key, use_kalman):
    """
    Open the per-run CSV and write its header row.

    One file per run, named by method and start time, so repeated runs of
    the same method never overwrite each other. The three coordinate
    pairs (raw / matched / actual) are the columns the CMP script needs:
    raw is ground-truth-side, actual is what SUMO really did, and matched
    is the matcher's intent -- which differs from actual whenever
    keepRoute overrides the placement.
    """
    global CSV_FILE, CSV_WRITER

    log_dir = os.path.join(SCRIPT_DIR, LOG_DIR)
    os.makedirs(log_dir, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    suffix = "_kalman" if use_kalman else ""
    path = os.path.join(log_dir, f"{method_key}{suffix}_{stamp}.csv")

    # newline="" is required on Windows or csv writes blank rows between
    # every record.
    CSV_FILE = open(path, "w", newline="", encoding="utf-8")
    CSV_WRITER = csv.writer(CSV_FILE)
    CSV_WRITER.writerow([
        "wall_time", "phone_timestamp", "method", "kalman",
        "lat", "lon",
        "raw_x", "raw_y",          # converted GPS, before matching
        "filt_x", "filt_y",        # after Kalman (== raw when disabled)
        "match_x", "match_y",      # matcher's chosen on-edge point
        "actual_x", "actual_y",    # where SUMO ended up placing the bike
        "edge_id", "raw_dist", "score", "correction_m", "match_ms",
        "phone_speed_mps", "sumo_speed_mps", "phone_course_deg", "accuracy_m",
    ])
    print(f"[INFO] Logging to {path}")
    return path


def parse_sumocfg_for_netfile(sumocfg_path: str) -> str:
    """
    Read the .sumocfg XML and return the absolute path to its <net-file>.

    The matchers need the .net.xml directly (they load the network with
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

    ST-Matching and the HMM divide by the elapsed time between fixes, so
    this feeds their temporal terms directly. The phone's OWN timestamp is
    used rather than the server arrival time: network jitter would
    otherwise show up as the rider changing speed. Returns None if
    unparseable, in which case a matcher falls back to its nominal_dt.
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

    NOTE: not every matcher uses heading -- Lou's ST-Matching formulation
    has no heading term, while the topological and fuzzy methods do. The
    angle is passed to moveToXY regardless so the bike is drawn facing the
    right way in the GUI.
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
    """
    Format one log value: floats to `spec`, anything else as-is.

    Score components are legitimately None on the first fix of a route and
    whenever a Viterbi chain restarts, so the logger has to tolerate it.
    None ("not applicable") and 0.0 ("scored zero") are genuinely different
    and must not be collapsed.

    Not every component is numeric, either. FuzzyMatcher reports a "mode"
    string ("following" / "entering_pending" / "confirmed") alongside its
    firing strengths, and HMMMatcher reports an integer candidate count.
    Applying a float format code to those raises ValueError/TypeError, and
    this function is called on the console-line path OUTSIDE the matcher's
    try/except -- so an unformattable component used to take down the whole
    run rather than producing an ugly log line. Non-floats are passed
    straight through instead.
    """
    if value is None:
        return "--"
    # bool is a subclass of int; print it as True/False, not 1.000.
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, (int, float)):
        try:
            return format(value, spec)
        except (ValueError, TypeError):
            return str(value)
    return str(value)


def _fmt_components(comps):
    """
    Render a matcher's score components as "name=value" pairs.

    Built from whatever keys the matcher returned rather than a fixed
    list, because the five methods report different terms: ST gives
    obs/trans/fs/ft, the topological method gives prox/head/conn/turn,
    fuzzy gives its rule firing strengths, and native gives nothing.
    """
    if not comps:
        return ""
    return " ".join(f"{k}={_fmt(v)}" for k, v in comps.items())


def native_match(x, y):
    """
    Baseline "matcher": ask SUMO which edge this point is on and let
    moveToXY do its own snapping.

    Returns a dict in the same shape the real matchers return, so the rest
    of the loop doesn't need to know which method is running. The x/y
    handed back are the RAW converted point, not a snapped one, because
    with keepRoute=0 the snapping happens inside moveToXY -- the resulting
    position is read back afterwards and logged as actual_x/actual_y.

    convertRoad is used only to supply an edge id for the call; it is the
    same nearest-edge projection described by equation (3) in the paper.

    It is called on the (possibly Kalman-filtered) NETWORK coordinates with
    isGeo=False, not on the raw lat/lon. Using the raw geo point here would
    mean a --kalman native run looked up its edge from the unfiltered
    position while being placed at the filtered one, which is a different
    pipeline from every other method's and would not be comparable.
    """
    try:
        edge_id, pos, lane_index = traci.simulation.convertRoad(x, y, isGeo=False)
    except traci.TraCIException as e:
        print(f"[WARN] convertRoad failed: {e}")
        return None

    return {
        "x": x,
        "y": y,
        "edge_id": edge_id,
        "lane_index": lane_index,
        "raw_dist": None,
        "score": None,
        "components": {},
        "window_len": None,
    }


def move_vehicle_to_phone_position(lat, lon, speed_mps, course_deg_raw,
                                   fix_time, accuracy_m):
    """
    Convert one GPS fix to SUMO coordinates, run the selected matcher, and
    move the controlled bike there. Respawns the vehicle first if it
    disappeared.

    accuracy_m is the phone's own reported horizontal accuracy for this
    fix. HMM.py and kalman_filter.py both use it as a per-fix sigma in
    preference to their fixed default, so a fix taken under a building is
    automatically trusted less than one in the open. Matchers that don't
    declare the parameter simply never see it.

    If the matcher returns None the update is skipped -- see the module
    docstring on why there is deliberately no native-matching fallback.
    """
    # Tracks the previous fix time so the Kalman filter gets the real
    # elapsed time rather than the nominal poll interval. Phone fixes do
    # not arrive on a perfect 1 Hz cadence, and the constant-velocity
    # prediction step is proportional to dt.
    global _LAST_FIX_TIME
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

    # ---- Optional Kalman pre-filter -----------------------------------
    # Applied to the converted (x, y) rather than to lat/lon so the state
    # and covariance are in metres, which is what the constant-velocity
    # model in equations (7)-(8) assumes. Runs identically ahead of all
    # five methods, which is the point of it being a separate stage.
    # angle_to_use is what we hand to moveToXY (may be INVALID_DOUBLE_VALUE).
    # Computed BEFORE the filter runs: the Kalman filter seeds its initial
    # velocity from the heading, so it must see a sanitised value (or None)
    # rather than the phone's "no heading available" sentinel.
    angle_to_use = parse_course_deg(course_deg_raw)

    # The matchers and the filter want a plain float or None for heading,
    # so translate SUMO's INVALID sentinel back into None.
    course_for_match = angle_to_use
    if course_for_match == traci.constants.INVALID_DOUBLE_VALUE:
        course_for_match = None

    filt_x, filt_y = x, y
    if KALMAN is not None:
        # Real elapsed time between phone fixes where both timestamps are
        # available, otherwise the nominal poll interval. The filter clamps
        # this to its own [min_dt, max_dt] range, so a garbage value here
        # degrades the smoothing rather than destabilising it.
        if fix_time is not None and _LAST_FIX_TIME is not None:
            dt = fix_time - _LAST_FIX_TIME
        else:
            dt = POLL_INTERVAL

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

    # Advance the fix clock whether or not the filter ran, so enabling
    # --kalman mid-session can't inherit a stale dt.
    if fix_time is not None:
        _LAST_FIX_TIME = fix_time

    # ---- Run the selected matcher -------------------------------------
    # match_ms is the per-fix matching latency. ST-Matching and the HMM run
    # Viterbi over a window with shortest-path queries per candidate pair,
    # so they are the expensive methods and this is the number the
    # real-time claim rests on. Measured identically for all five.
    match_start = time.perf_counter()
    try:
        if MATCHER is None:
            result = native_match(filt_x, filt_y)
        else:
            # Offer every optional input; keep only the ones this matcher's
            # match() actually declares. Dropping `timestamp` for the
            # topological matcher is not a loss of information -- it has no
            # temporal term to spend it on.
            call_kwargs = {
                "timestamp": fix_time,        # ST-Matching only
                "speed_mps": speed_mps,       # all but ST's core scoring
                "course_deg": course_for_match,
                "accuracy_m": accuracy_m,     # HMM and fuzzy declare it
            }
            call_kwargs = {k: v for k, v in call_kwargs.items()
                           if k in MATCH_PARAMS}
            result = MATCHER.match(filt_x, filt_y, **call_kwargs)
    except Exception:
        import traceback
        print("[ERROR] Matcher raised:")
        traceback.print_exc()
        result = None
    match_ms = (time.perf_counter() - match_start) * 1000.0

    # No match -> skip this fix entirely. The bike holds its last position.
    if result is None:
        print(f"[SKIP] {METHOD_KEY}: no match for ({filt_x:.1f}, {filt_y:.1f}) "
              f"-- update skipped ({match_ms:.1f} ms)")
        return

    # Matcher succeeded: use its chosen on-edge (x, y) and edge id.
    move_x, move_y = result["x"], result["y"]
    edge_id = result["edge_id"]
    # Matchers that pick a specific lane can say so; the rest default to 0.
    lane_index = result.get("lane_index", 0)

    # keepRoute comes from the registry: 0 for native (SUMO re-snaps, which
    # is the baseline being measured), 6 for the matchers (exact placement
    # so their decision survives, plus the lane-permission bypass that
    # avoids the sumo-gui freeze).
    keep_route = METHOD_CFG["keep_route"]

    # NOTE: do NOT call setRoute() here. SUMO requires a replacement route to
    # contain the vehicle's CURRENT edge, so setRoute(VEHICLE_ID, [edge_id])
    # fails with "current edge ... not found in new route" on every fix after
    # the bike has moved off its spawn edge, and additionally trips a
    # "prohibits" warning whenever the matched edge disallows bicycles. It is
    # also unnecessary: keepRoute bit 2 (exact placement) means the placement
    # does not consult the route at all, and bit 4 bypasses the lane
    # permission check. The dummy spawn route is deliberately left stale.
    try:
        traci.vehicle.moveToXY(
            vehID=VEHICLE_ID,
            edgeID=edge_id,
            laneIndex=lane_index,
            x=move_x,
            y=move_y,
            angle=angle_to_use,
            keepRoute=keep_route,
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

    # ---- Read back SUMO's current state --------------------------------
    # getPosition is the authoritative answer to "where did the bike
    # actually end up". For native it is the only way to see the snapped
    # point at all; for the matchers it is the check that keepRoute really
    # did preserve the requested coordinates. CMP is computed from this
    # column, not from move_x/move_y.
    try:
        actual_x, actual_y = traci.vehicle.getPosition(VEHICLE_ID)
    except traci.TraCIException:
        actual_x, actual_y = None, None

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

    # ---- Console line, one per update ---------------------------------
    # xy is the position requested from moveToXY; act is where SUMO put
    # the bike. Those two diverging means keepRoute overrode the matcher.
    act_str = (f"({actual_x:.1f}, {actual_y:.1f})"
               if actual_x is not None else "(N/A)")
    # Defensive: the bike has already been moved successfully by this point,
    # so nothing in the reporting below is worth ending the run over.
    try:
        comp_str = _fmt_components(result.get("components"))
    except Exception as e:
        comp_str = f"<components unprintable: {e}>"
    win = result.get("window_len")

    print(
        f"[OK][{METHOD_KEY}] {VEHICLE_ID} -> edge={edge_id}, "
        f"xy=({move_x:.1f}, {move_y:.1f}), act={act_str}, "
        f"raw=({x:.1f}, {y:.1f}), corr={correction_m:.2f} m | "
        + (f"win={win} " if win is not None else "")
        + (comp_str + " | " if comp_str else "")
        + f"match={match_ms:.1f} ms | "
        f"PHONE speed={phone_speed_mps:.2f} m/s ({phone_speed_kmh:.2f} km/h), "
        f"PHONE course={phone_course_str} | "
        f"SUMO speed={sumo_speed_str}, SUMO angle={sumo_angle_str}"
    )

    # ---- CSV row for offline CMP evaluation ---------------------------
    if CSV_WRITER is not None:
        CSV_WRITER.writerow([
            datetime.now().isoformat(timespec="milliseconds"),
            fix_time, METHOD_KEY, KALMAN is not None,
            lat, lon,
            f"{x:.4f}", f"{y:.4f}",
            f"{filt_x:.4f}", f"{filt_y:.4f}",
            f"{move_x:.4f}", f"{move_y:.4f}",
            "" if actual_x is None else f"{actual_x:.4f}",
            "" if actual_y is None else f"{actual_y:.4f}",
            edge_id,
            "" if result.get("raw_dist") is None else f"{result['raw_dist']:.4f}",
            "" if result.get("score") is None else f"{result['score']:.6f}",
            f"{correction_m:.4f}", f"{match_ms:.3f}",
            f"{phone_speed_mps:.3f}",
            "" if sumo_speed_mps is None else f"{sumo_speed_mps:.3f}",
            "" if phone_course_deg is None else f"{phone_course_deg:.1f}",
            "" if accuracy_m is None else f"{accuracy_m:.2f}",
        ])
        # Flush every row: a run that ends with Ctrl+C or a SUMO crash
        # would otherwise lose whatever was still sitting in the buffer,
        # and these are ride recordings that can't be regenerated.
        CSV_FILE.flush()


def parse_args():
    """Command-line interface. Everything here has a sensible default."""
    p = argparse.ArgumentParser(
        description="eBike-in-the-Loop live bridge with selectable map matching.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Methods:\n" + "\n".join(
            f"  {k:<8} {v['label']}" for k, v in METHODS.items()
        ),
    )
    p.add_argument("-m", "--method", default=None,
                   help="matching method to run; prompts if omitted")
    # Tri-state on purpose: None means "not specified", which triggers the
    # interactive prompt. --kalman / --no-kalman both skip that prompt, so
    # a scripted batch of runs never blocks waiting for input.
    p.add_argument("--kalman", dest="kalman", action="store_true", default=None,
                   help="apply the Kalman pre-filter before matching")
    p.add_argument("--no-kalman", dest="kalman", action="store_false",
                   help="skip the Kalman pre-filter (no prompt)")
    p.add_argument("--log", action="store_true",
                   help=f"write a per-fix CSV into ./{LOG_DIR}/")
    p.add_argument("--no-gui", action="store_true",
                   help="run headless (sumo instead of sumo-gui)")
    p.add_argument("--cfg", default=SUMO_CFG,
                   help="path to the .sumocfg, relative to this script")
    return p.parse_args()


def main():
    # These are assigned here and read inside the move function, so they
    # have to be declared global before first assignment.
    global METHOD_KEY, METHOD_CFG, MATCHER, KALMAN

    args = parse_args()

    # Method comes from --method if given, otherwise from the menu.
    METHOD_KEY = resolve_method(args.method) if args.method else prompt_for_method()
    METHOD_CFG = METHODS[METHOD_KEY]

    # Same for the pre-filter: explicit flag wins, otherwise ask.
    use_kalman = args.kalman if args.kalman is not None else prompt_for_kalman()

    # Resolve and validate the SUMO config path up front, anchored to this
    # script's directory so the launch dir doesn't matter.
    sumocfg_abs = os.path.abspath(os.path.join(SCRIPT_DIR, args.cfg))
    if not os.path.exists(sumocfg_abs):
        raise FileNotFoundError(f"SUMO config not found: {sumocfg_abs}")

    # Extract the .net.xml path the matchers need from the config.
    net_file = parse_sumocfg_for_netfile(sumocfg_abs)

    print("\n" + "-" * 62)
    print(f"[INFO] Method:     {METHOD_KEY} -- {METHOD_CFG['label']}")
    print(f"[INFO] Kalman:     {'on' if use_kalman else 'off'}")
    print(f"[INFO] keepRoute:  {METHOD_CFG['keep_route']}")
    print(f"[INFO] SUMO config: {sumocfg_abs}")
    print(f"[INFO] Net file:    {net_file}")
    print("-" * 62)

    # Build the matcher ONCE (readNet parses the whole network, which is
    # seconds for a campus-sized map) and reuse it for every fix.
    MATCHER = build_matcher(METHOD_KEY, net_file)
    if MATCHER is None:
        print("[INFO] Native SUMO matching -- no external matcher loaded.")
    else:
        # Echo the settings that actually reached the constructor, so the
        # console records the exact configuration of this run.
        shown = ", ".join(f"{k}={v}" for k, v in USED_KWARGS.items())
        print(f"[INFO] {type(MATCHER).__name__} ready ({shown}).")

    # Optional pre-filter, built after the matcher so a missing kalman.py
    # fails before SUMO is launched rather than after.
    if use_kalman:
        KALMAN = build_kalman()
        print("[INFO] Kalman pre-filter active.")

    if args.log:
        open_run_log(METHOD_KEY, use_kalman)

    print("[INFO] Starting SUMO...")

    # Launch SUMO under TraCI's control:
    #   sumo-gui / sumo -> visual front-end, or headless with --no-gui
    #   --start         -> begin stepping immediately, no manual play
    #   --delay         -> GUI pacing; handled by SUMO so its event loop
    #                      keeps running (a manual sleep() starves the GUI)
    #   --end 1000000   -> effectively never auto-terminate
    binary = "sumo" if args.no_gui else "sumo-gui"
    cmd = [
        binary,
        "-c", sumocfg_abs,
        "--step-length", str(SUMO_STEP_LENGTH),
        "--start",
        "--end", "1000000",
    ]
    # --delay only means anything to the GUI build.
    if not args.no_gui:
        cmd += ["--delay", SUMO_DELAY_MS]

    traci.start(cmd)

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

                        # Phone-reported horizontal accuracy, used as a
                        # per-fix sigma by the HMM and the Kalman filter.
                        # May be absent; None means "use the default".
                        try:
                            accuracy_m = data.get("accuracy_m")
                            accuracy_m = (None if accuracy_m is None
                                          else float(accuracy_m))
                        except Exception:
                            accuracy_m = None

                        # Fix time in epoch seconds for the temporal terms.
                        fix_time = phone_timestamp_seconds(phone_timestamp)

                        # Do the actual convert -> match -> move for this fix.
                        move_vehicle_to_phone_position(
                            lat, lon, speed_mps, course_deg,
                            fix_time, accuracy_m
                        )

    except KeyboardInterrupt:
        # Ctrl+C is the intended way to stop; exit the loop cleanly.
        print("\n[INFO] Stopped by user.")
    finally:
        # Always close the log first -- if TraCI shutdown hangs, the ride
        # data is already safely on disk.
        if CSV_FILE is not None:
            try:
                CSV_FILE.close()
            except Exception:
                pass
        # Always try to close the TraCI connection so SUMO shuts down.
        try:
            traci.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()