"""
live_matching_v1.py
======================================================================
eBike-in-the-Loop live bridge -- MULTI-BIKE, MULTI-METHOD RUNNER.

This is live_phone_to_sumo.py's job (poll the Flask server for the phone's
newest GPS fix, map-match it, and animate an eBike in SUMO) rebuilt on the
engine of post_processed_matching_v2.py: the same METHODS registry, the
same matcher / Kalman construction, the same native (convertRoad +
sumolib) baseline, and the same output schema -- but able to drive up to
TEN bikes at once, one per (method, Kalman on/off) combination, all fed the
exact same GPS fix at the exact same moment.

    python live_matching_v1.py --method st --kalman        one bike
    python live_matching_v1.py --method hmm --both-kalman  two bikes
    python live_matching_v1.py --method all                all TEN bikes
    python live_matching_v1.py --method topo,st --no-kalman
    python live_matching_v1.py                             (interactive menu)

Methods (identical registry to post_processed_matching_v2.py):
    native  -- SUMO convertRoad snapping + sumolib lane geometry (baseline)
    topo    -- TopologicalMatcher   (topological.py)
    fuzzy   -- FuzzyMatcher         (FuzzyLogic.py)
    hmm     -- HMMMatcher           (HMM.py)
    st      -- STMatcher            (STMatching.py)

BIKE NAMES
----------
Every bike is called  <method>_<wkal|wokal>_ebike :

    native_wokal_ebike   native_wkal_ebike
    topo_wokal_ebike     topo_wkal_ebike
    fuzzy_wokal_ebike    fuzzy_wkal_ebike
    hmm_wokal_ebike      hmm_wkal_ebike
    st_wokal_ebike       st_wkal_ebike

wkal = fed the Kalman-filtered point, wokal = fed the raw GPS point. (A
literal "/" is deliberately avoided: it is not safe in SUMO IDs or file
names.) In the GUI each method has its own colour; the with-Kalman bike is
the lighter tint of its method's colour.

HOW ONE FIX IS PROCESSED
------------------------
    phone GPS --> Flask /latest --> convertGeo() --> raw (x, y)     [once]
                                        |
                          Kalman filter (only if any wkal bike)     [once]
                                        |
            +---------------+-----------+-----------+---------------+
            v               v                       v               v
       bike 1 matcher   bike 2 matcher   ...   bike N matcher   (each with
       (raw or filtered point, per bike)                          its OWN
            |               |                       |             state)
            v               v                       v
        moveToXY        moveToXY                moveToXY   --> N CSV rows

Everything up to the fan-out happens ONCE per fix, so all bikes are
compared on identical input. Every bike owns its own matcher instance
(the matchers are stateful: ST's Viterbi window, the fuzzy matcher's
confirm-count debounce and the HMM's chain must never be shared between
the with- and without-Kalman variants). The Kalman filter is deterministic,
so a single shared instance is updated once per fix and its output is
handed to all the wkal bikes.

NO FALLBACK TO NATIVE MATCHING. If a matcher returns nothing, that bike is
skipped for this fix and holds its last position; the miss is still
written to the CSV (matched=False + unmatched_reason). A failure or
exception in one bike never stops the others.

PLACEMENT (no exact placement anywhere)
---------------------------------------
Exact placement (keepRoute 2/6) crashed sumo-gui, so every bike is now
SNAPPED onto a lane by SUMO:
    native         keepRoute 0 -- pure SUMO snapping (the baseline)
    custom methods keepRoute 4 -- SUMO snapping, permissions ignored,
                                  steered by the matcher's edge as the
                                  moveToXY edgeID hint
The METHOD RESULT is still the matcher's own output (matched_x/matched_y,
edge_id) -- that is what the comparison should use. actual_x/actual_y and
actual_edge_id are where the bike was drawn in SUMO. sumo_override=True
marks any fix where SUMO drew a bike on a different edge from the one its
matcher chose; check that column after a run -- it should be (almost)
always False for the custom methods.

CSV LOG
-------
On by default (--no-log to disable). ONE file per run in ./runs/ with one
row per bike per fix:
    single bike : <method>[_kalman]_<timestamp>.csv
    all 10      : all_methods_<timestamp>.csv
    otherwise   : multi_<N>bikes_<timestamp>.csv
Rows are identified by vehicle_id / method / kalman, and grouped by fix_seq
(all rows sharing a fix_seq are the same GPS fix). The columns are a
superset of the old live log and the post-processed log -- see FIELDNAMES.
matched_lat/matched_lon are back-projected through SUMO itself
(traci.simulation.convertGeo), so pyproj is NOT required.
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
import requests
import xml.etree.ElementTree as ET
from datetime import datetime

# ---------- USER SETTINGS ----------
# Path to the SUMO scenario config, relative to this script's own directory.
SUMO_CFG = r"2026-08-25-19-43-30/osm.sumocfg"
# Endpoint on the Flask server that returns the most recent phone fix.
FLASK_LATEST_URL = "http://localhost:5000/latest"

# Vehicle type shared by every bike (see ensure_vehicle_type), and the
# single throwaway route every bike is created with.
VEHICLE_TYPE_ID = "bike_live"
DUMMY_ROUTE_ID = "route_live_dummy"

POLL_INTERVAL = 1.0        # seconds between polls of the Flask server
STALE_DATA_SECONDS = 5.0   # ignore phone fixes older than this
SUMO_STEP_LENGTH = 1.0     # simulation seconds advanced per step
MATCH_THRESHOLD = 100.0    # moveToXY search radius (m) for candidate edges
SUMO_DELAY_MS = "1000"     # GUI pacing, handled by SUMO's own event loop

# Upper bound on the Kalman filter's dt. Matches post_processed_matching_v2
# (30 s): a fix is only ever dropped for staleness BEFORE it reaches the
# filter, but the gap between two accepted fixes can still exceed 5 s
# after a dropout.
MAX_KALMAN_DT = 30.0

# Directory for the CSV logs.
LOG_DIR = "runs"

# File SUMO writes its own messages to when --sumo-log is given.
SUMO_LOG_FILE = "sumo_log.txt"

# Hard ceiling on simultaneous bikes (5 methods x Kalman on/off).
MAX_BIKES = 10
# ----------------------------------

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

# Anchor relative paths to THIS FILE's directory, not the working directory,
# and make that directory importable so "from topological import ..." etc.
# work regardless of where the script was launched from.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


# ======================================================================
# METHOD REGISTRY -- identical to post_processed_matching_v2.py
# ======================================================================
# One entry per map-matching method; the ONLY place that knows anything
# method-specific. Matcher tuning is copied verbatim from
# post_processed_matching_v2.py so a live run and an offline run of the
# same method use the same parameters. The ground-truth pseudo-method
# (generate_gt) is NOT a matcher and is deliberately left out here.
#
# Extra fields on top of v2's registry (they only matter live):
#   keep_route : the moveToXY keepRoute bitmask used to place the bike.
#                It is a sum of flag VALUES:
#                  1 = only lanes on the current route
#                  2 = exact placement at the given (x, y)
#                  4 = ignore lane permissions
#                Exact placement (flag 2, i.e. keepRoute 2 or 6) is NOT
#                used by anything: it froze/crashed sumo-gui (2 alone via
#                lane-less bikes, upstream SUMO issue #10974; 6 still
#                crashed with 10 bikes running).
#                Custom matchers use 4: SUMO snaps the bike onto a lane
#                (always a clean on-lane state, like native), permissions
#                ignored, and the matcher's edge is passed as the moveToXY
#                edgeID hint. SUMO's scoring weights a matching edge ID
#                heavily, and the matcher's point is already on that edge,
#                so the bike lands on the matcher's edge at (nearly) the
#                matcher's point. Every fix is CHECKED: actual_edge_id is
#                read back and sumo_override=True flags any fix where SUMO
#                put the bike on a different edge than the matcher chose.
#                Native uses 0: pure SUMO snapping, the baseline.
#                Never use 1/3/5/7 here: every bike lives on a one-edge
#                dummy route, so bit 1 would fail as soon as the bike
#                leaves that edge.
#   colour     : (r, g, b) of the wokal bike; the wkal bike gets a tint.
METHODS = {
    "native": {
        "module": None,
        "cls": None,
        "kwargs": {},
        "keep_route": 0,
        "colour": (110, 110, 110),
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
        "keep_route": 4,
        "colour": (30, 90, 220),
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
        "keep_route": 4,
        "colour": (30, 160, 60),
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
        "keep_route": 4,
        "colour": (240, 130, 20),
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
            "nominal_dt": POLL_INTERVAL,
            "vclass": None,
        },
        "keep_route": 4,
        "colour": (220, 30, 30),
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

ALL_KEYWORD = "all"
KAL_TAG = {False: "wokal", True: "wkal"}
KALMAN_MODE_VARIANTS = {"off": [False], "on": [True], "both": [False, True]}

# The exact columns of the CSV. This is a SUPERSET of both existing logs:
#   * every column of the old live_phone_to_sumo.py log, same names
#     (match_x/match_y, actual_x/actual_y, correction_m, sumo_speed_mps...)
#   * every column of the post_processed_matching_v2.py log
#     (matched, lane_index, lane_pos, is_internal_edge, matched_x/y,
#      matched_lat/lon, match_error_raw_m, match_error_filt_m, components,
#      window_len, unmatched_reason)
#   * plus fix_seq and vehicle_id so 10 bikes can share one file.
# match_x/match_y == matched_x/matched_y and correction_m ==
# match_error_raw_m: the duplicates exist only so a CMP script written for
# either existing log reads this one unchanged. Drop them if unneeded.
FIELDNAMES = [
    "wall_time", "phone_timestamp", "fix_seq",
    "vehicle_id", "method", "kalman",
    "lat", "lon",
    "matched",
    "raw_x", "raw_y",              # converted GPS, before Kalman/matching
    "filt_x", "filt_y",            # point the matcher was actually given
                                   # (== raw for wokal bikes)
    "match_x", "match_y",          # matcher's chosen on-edge point (legacy)
    "matched_x", "matched_y",      # same values, v2 naming
    "matched_lat", "matched_lon",
    "actual_x", "actual_y",        # where SUMO really placed the bike
    "actual_edge_id",              # edge SUMO really placed the bike on
    "sumo_override",               # True if actual_edge_id != edge_id
    "edge_id", "lane_index", "lane_pos", "is_internal_edge",
    "raw_dist", "score", "components", "window_len",
    "correction_m", "match_error_raw_m", "match_error_filt_m", "match_ms",
    "phone_speed_mps", "sumo_speed_mps", "phone_course_deg", "accuracy_m",
    "unmatched_reason",
]


# ======================================================================
# METHOD / KALMAN SELECTION
# ======================================================================
def resolve_method(name):
    """Normalise ONE user-supplied method name to a registry key. Case- and
    alias-insensitive; raises with the full list of valid names."""
    key = str(name).strip().lower()
    key = METHOD_ALIASES.get(key, key)
    if key not in METHODS:
        valid = ", ".join(list(METHODS.keys()) + [ALL_KEYWORD])
        raise ValueError(f"Unknown method '{name}'. Choose one of: {valid}")
    return key


def parse_method_arg(text):
    """Turn a --method value into (list_of_method_keys, is_all).

    Accepts 'all', a single method, or a comma-separated list
    ('topo,st'). Order follows the registry so bikes always appear in the
    same order regardless of how the list was typed; duplicates collapse.
    """
    parts = [p for p in str(text).replace(" ", "").split(",") if p]
    if not parts:
        raise ValueError("--method was given but empty")
    if any(p.lower() == ALL_KEYWORD for p in parts):
        return list(METHODS.keys()), True
    wanted = {resolve_method(p) for p in parts}
    return [k for k in METHODS if k in wanted], False


def prompt_for_methods():
    """Interactive method menu. Accepts a number, a name, or 'all' (the
    last runs all five methods, with and without Kalman = 10 bikes).
    Returns (list_of_method_keys, is_all)."""
    keys = list(METHODS.keys())
    all_idx = len(keys) + 1

    print("\n" + "=" * 66)
    print(" eBike-in-the-Loop -- select map-matching method")
    print("=" * 66)
    for i, k in enumerate(keys, start=1):
        print(f"  {i}. {k:<8} {METHODS[k]['label']}")
    print(f"  {all_idx}. {ALL_KEYWORD:<8} ALL five methods, with AND without Kalman "
          f"({MAX_BIKES} bikes at once)")
    print("=" * 66)

    while True:
        choice = input("Method [number or name]: ").strip()

        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(keys):
                return [keys[idx - 1]], False
            if idx == all_idx:
                return keys, True
            print(f"  -> pick 1..{all_idx}")
            continue

        try:
            return parse_method_arg(choice)
        except ValueError as e:
            print(f"  -> {e}")


def prompt_for_kalman_mode():
    """Interactive Kalman choice for a non-'all' selection. Defaults to OFF
    on a bare Enter (the unfiltered number is the safer default: it doesn't
    quietly add a second variable to the comparison). 'both' runs the
    method twice, side by side, as two bikes. Returns a list of booleans."""
    print("\n" + "=" * 66)
    print(" Kalman pre-filter (smooths GPS before matching)")
    print("=" * 66)
    print("  1. off   raw GPS straight into the matcher  [default]")
    print("  2. on    constant-velocity filter first")
    print("  3. both  one bike of each, side by side")
    print("=" * 66)

    while True:
        choice = input("Kalman [1/2/3, off/on/both, Enter=off]: ").strip().lower()

        if choice in ("", "1", "n", "no", "off", "false"):
            return KALMAN_MODE_VARIANTS["off"]
        if choice in ("2", "y", "yes", "on", "true"):
            return KALMAN_MODE_VARIANTS["on"]
        if choice in ("3", "both", "b"):
            return KALMAN_MODE_VARIANTS["both"]
        print("  -> answer 1/2/3, off/on/both, or press Enter for off")


def resolve_selection(args):
    """Work out which (method, kalman) bikes to run from the CLI flags,
    falling back to the interactive menus for whatever was not given.

    'all' with no Kalman flag means all TEN combinations and does not ask
    about Kalman (there is nothing left to choose). 'all' WITH --kalman or
    --no-kalman is honoured as five bikes.
    """
    if args.method:
        method_keys, is_all = parse_method_arg(args.method)
    else:
        method_keys, is_all = prompt_for_methods()

    if args.kalman_mode:
        variants = KALMAN_MODE_VARIANTS[args.kalman_mode]
    elif is_all:
        variants = KALMAN_MODE_VARIANTS["both"]
    else:
        variants = prompt_for_kalman_mode()

    return method_keys, variants


# ======================================================================
# RUNNER -- one bike = one method + one Kalman setting + its own matcher
# ======================================================================
def _tint(rgb, amount=0.55):
    """Blend an (r, g, b) colour toward white; used for the wkal bikes."""
    return tuple(int(c + (255 - c) * amount) for c in rgb)


class Runner:
    """Everything belonging to ONE bike. Holding the matcher (and its match()
    parameter names) per runner, rather than in module globals as the
    single-bike scripts do, is what lets several methods -- and the with/
    without-Kalman variants of the same method -- run side by side without
    sharing any matcher state."""

    def __init__(self, method_key, use_kalman):
        self.method_key = method_key
        self.use_kalman = use_kalman
        self.cfg = METHODS[method_key]
        self.tag = f"{method_key}_{KAL_TAG[use_kalman]}"
        self.vehicle_id = f"{self.tag}_ebike"

        base = self.cfg["colour"]
        rgb = _tint(base) if use_kalman else base
        self.colour = (rgb[0], rgb[1], rgb[2], 255)

        self.matcher = None
        self.match_params = set()
        self.used_kwargs = {}

        self.stats = {"fixes": 0, "matched": 0, "skipped": 0, "ms_total": 0.0}


def _resolve_class(module, preferred, method_name):
    """Find the matcher class inside an imported module: the registry name
    if it exists there, otherwise the sole class defined in the module that
    has `method_name` (reported as a substitution). Fails loudly if there is
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


def build_matcher(runner, net_file, announced):
    """Import and instantiate a FRESH matcher for this runner (native gets
    None). Constructor kwargs are filtered against the class's real
    signature and anything dropped is printed -- once per method, tracked
    in `announced`, so 10 bikes don't repeat the same warning."""
    cfg = runner.cfg
    if cfg["module"] is None:
        return

    try:
        module = __import__(cfg["module"], fromlist=[cfg["cls"]])
    except ImportError as e:
        raise ImportError(
            f"Could not import '{cfg['module']}' for method '{runner.method_key}': {e}\n"
            f"       Expected {cfg['module']}.py alongside this script in\n"
            f"       {SCRIPT_DIR}"
        ) from e

    cls = _resolve_class(module, cfg["cls"], "match")

    accepted = set(inspect.signature(cls.__init__).parameters)
    kwargs = {k: v for k, v in cfg["kwargs"].items() if k in accepted}
    dropped = sorted(set(cfg["kwargs"]) - set(kwargs))
    if dropped and runner.method_key not in announced:
        announced.add(runner.method_key)
        print(f"[WARN] {cfg['cls']} does not accept: {', '.join(dropped)} "
              f"-- these settings were IGNORED for this run.")

    runner.match_params = set(inspect.signature(cls.match).parameters)
    runner.used_kwargs = kwargs
    runner.matcher = cls(net_file, **kwargs)


def build_kalman():
    """Build the ONE shared Kalman pre-filter. Same module and parameters as
    post_processed_matching_v2.py (process_noise=0.1). A missing module is
    a hard error rather than a silent disable: a bike labelled 'wkal' that
    quietly didn't filter anything would be worse than no bike at all."""
    try:
        from kalman_filter import KalmanFilter
    except ImportError as e:
        raise ImportError(
            f"Kalman requested but kalman_filter.py could not be imported: {e}\n"
            f"       Expected it alongside this script in {SCRIPT_DIR}"
        ) from e

    return KalmanFilter(
        process_noise=0.1,
        sigma_default=4.07,
        use_accuracy=True,
        min_sigma=1.0,
        min_dt=0.05,
        max_dt=MAX_KALMAN_DT,
    )


# ======================================================================
# SMALL HELPERS (parsing / formatting)
# ======================================================================
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
    """Valid heading in degrees (0..360), or None if unusable (negative /
    missing / unparseable). The phone sends 0 or negative when it has no
    real heading. Plain float-or-None form, as in v2; the INVALID_DOUBLE_
    VALUE sentinel moveToXY wants is substituted at the call site."""
    try:
        if course_deg_raw in (None, ""):
            return None
        course_deg = float(course_deg_raw)
        if course_deg < 0:
            return None
        return course_deg % 360.0
    except Exception:
        return None


def phone_timestamp_seconds(ts_raw):
    """Phone's ISO-8601 fix time -> epoch seconds (float), or None if
    unparseable (matchers then fall back to their nominal dt). The phone's
    OWN time is used, not server arrival time, so network jitter doesn't
    show up as the rider changing speed."""
    if not ts_raw:
        return None
    try:
        return float(ts_raw)
    except (TypeError, ValueError):
        pass
    try:
        return datetime.fromisoformat(str(ts_raw).replace("Z", "+00:00")).timestamp()
    except Exception:
        return None


def _fmt(value, spec=".3f"):
    """Format one value for the console: floats to `spec`, anything else
    as-is. Matchers report non-numeric components too (the fuzzy mode
    string, the HMM candidate count), so this tolerates those."""
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
    from whatever keys it returned (the five methods report different
    terms)."""
    if not comps:
        return ""
    return " ".join(f"{k}={_fmt(v)}" for k, v in comps.items())


def _csv(value, nd):
    """CSV cell: '' for None/'' , else fixed-decimals for numbers."""
    if value is None or value == "":
        return ""
    try:
        return f"{float(value):.{nd}f}"
    except (TypeError, ValueError):
        return str(value)


def xy_to_lonlat(x, y):
    """Network (x, y) -> (lon, lat) using SUMO's OWN projection, via TraCI.

    This replaces sumolib's net.convertXY2LonLat, which needs the pyproj
    package. convertGeo with its default fromGeo=False goes network ->
    geo, using exactly the projection SUMO used to build the network (and
    that convertGeo(..., fromGeo=True) used to bring the raw GPS in), so
    the round trip is consistent. Returns (lon, lat) or (None, None)."""
    try:
        lon, lat = traci.simulation.convertGeo(x, y)
        return lon, lat
    except traci.exceptions.FatalTraCIError:
        raise
    except traci.TraCIException as e:
        print(f"[WARN] Could not back-project ({x:.1f}, {y:.1f}) to lat/lon: {e}")
        return None, None


# ======================================================================
# NATIVE BASELINE
# ======================================================================
def native_match(x, y):
    """Baseline 'matcher': ask SUMO which edge the point is on via
    convertRoad (same as post_processed_matching_v2.py). Returns a dict in
    the same shape the real matchers return, plus 'lane_pos', which
    snap_to_lane_geometry needs to turn edge/lane/pos back into a point.

    Called on the (possibly Kalman-filtered) NETWORK coordinates with
    isGeo=False so a wkal native bike looks up its edge from the same
    position it reports as matched."""
    try:
        edge_id, lane_pos, lane_index = traci.simulation.convertRoad(x, y, isGeo=False)
    except traci.TraCIException as e:
        print(f"[WARN] convertRoad failed: {e}")
        return None

    if not edge_id:
        return None

    return {
        "x": x, "y": y,
        "edge_id": edge_id,
        "lane_index": lane_index,
        "lane_pos": lane_pos,
        "raw_dist": None,
        "score": None,
        "components": {},
        "window_len": None,
    }


def snap_to_lane_geometry(net, edge_id, lane_index, lane_pos):
    """Actual on-lane (x, y) for an edge/lane/pos from the static network
    geometry (sumolib). Only native needs this: the custom matchers already
    return an on-edge point. Returns None on failure."""
    try:
        lane = net.getLane(f"{edge_id}_{lane_index}")
        return sumolib.geomhelper.positionAtShapeOffset(lane.getShape(), lane_pos)
    except Exception as e:
        print(f"[WARN] Could not compute snapped geometry for edge {edge_id}: {e}")
        return None


# ======================================================================
# SUMO VEHICLE HANDLING
# ======================================================================
def ensure_vehicle_type():
    """Make sure VEHICLE_TYPE_ID exists: clone SUMO's DEFAULT_BIKETYPE and
    cap its top speed at 12 m/s, or fall back to cloning DEFAULT_VEHTYPE
    as a bicycle. One type is shared by all bikes."""
    try:
        existing = set(traci.vehicletype.getIDList())
        if VEHICLE_TYPE_ID not in existing:
            try:
                traci.vehicletype.copy("DEFAULT_BIKETYPE", VEHICLE_TYPE_ID)
                traci.vehicletype.setMaxSpeed(VEHICLE_TYPE_ID, 12.0)
            except traci.TraCIException:
                traci.vehicletype.copy("DEFAULT_VEHTYPE", VEHICLE_TYPE_ID)
                traci.vehicletype.setVehicleClass(VEHICLE_TYPE_ID, "bicycle")
    except traci.exceptions.FatalTraCIError:
        raise
    except Exception as e:
        print(f"[WARN] Could not ensure vehicle type: {e}")


def prepare_spawn_route():
    """Register the ONE throwaway single-edge route every bike is created
    with. SUMO needs a route to add a vehicle at all, but the bikes are
    driven entirely by GPS/moveToXY, so any real edge will do. Returns the
    route id."""
    usable = [e for e in traci.edge.getIDList() if not e.startswith(":")]
    if not usable:
        raise RuntimeError("No usable edges found in network.")
    if DUMMY_ROUTE_ID not in traci.route.getIDList():
        traci.route.add(DUMMY_ROUTE_ID, [usable[0]])
    return DUMMY_ROUTE_ID


def spawn_bike_if_missing(runner):
    """Create this runner's bike if it isn't in the simulation. Called only
    once a match exists, so the bike appears at its first matched position
    rather than parked on an arbitrary edge.

    Returns True if the vehicle exists (or is loaded and merely awaiting
    insertion, which moveToXY then completes)."""
    vid = runner.vehicle_id
    try:
        if vid in traci.vehicle.getIDList():
            return True
        # Added on a previous fix but not yet inserted (moveToXY failed):
        # re-adding would raise 'already exists'.
        if vid in traci.simulation.getLoadedIDList():
            return True
    except traci.exceptions.FatalTraCIError:
        raise
    except Exception:
        pass

    try:
        traci.vehicle.add(
            vehID=vid,
            routeID=DUMMY_ROUTE_ID,
            typeID=VEHICLE_TYPE_ID,
            depart="now",
            departLane="best",
            departPos="base",
            departSpeed="0",
        )
        # setSpeedMode(0) disables SUMO's own safety checks (car-following,
        # junction, red-light) so the bike goes exactly where GPS puts it.
        traci.vehicle.setSpeedMode(vid, 0)
        traci.vehicle.setSpeed(vid, 0.0)
        traci.vehicle.setColor(vid, runner.colour)
        print(f"[INFO] Spawned {vid}")
        return True
    except traci.exceptions.FatalTraCIError:
        raise
    except traci.TraCIException as e:
        print(f"[WARN] Could not spawn {vid}: {e}")
        return False


# ======================================================================
# LIVE SESSION
# ======================================================================
class LiveSession:
    """Owns everything that is shared across bikes for one run: the sumolib
    network, the (single) Kalman filter, the CSV log and the fix clock."""

    def __init__(self, net, runners, kalman, csv_file, csv_writer):
        self.net = net
        self.runners = runners
        self.kalman = kalman
        self.csv_file = csv_file
        self.csv_writer = csv_writer
        self.fix_seq = 0
        self.last_fix_time = None      # epoch seconds, for the Kalman dt
        # The batch of rows for the most recent fix, held back until SUMO has
        # actually applied the moveToXY calls (next simulationStep) so that
        # actual_x/actual_y/sumo_speed_mps can be read back correctly. See
        # finalize_pending().
        self._batch = None
        # One bike -> the detailed per-fix line; several -> a compact
        # line each (10 long lines a second are unreadable).
        self.verbose = False

    # ------------------------------------------------------------------
    def process_fix(self, lat, lon, speed_mps, course_deg_raw, fix_time, accuracy_m):
        """Convert one GPS fix, Kalman-filter it once if any bike needs it,
        then run every bike on it and log one CSV row per bike."""
        t_fix = time.perf_counter()
        # Defensive: the main loop finalises after every simulationStep, so
        # nothing should be pending here.
        self.finalize_pending(read_back=False)
        self.fix_seq += 1
        seq = self.fix_seq
        wall_time = datetime.now().isoformat(timespec="milliseconds")

        try:
            phone_course_deg = None if course_deg_raw in (None, "") else float(course_deg_raw)
        except Exception:
            phone_course_deg = None
        phone_speed_mps = float(speed_mps or 0.0)

        # Fields identical for every bike on this fix.
        common = {
            "wall_time": wall_time,
            "phone_timestamp": "" if fix_time is None else fix_time,
            "fix_seq": seq,
            "lat": lat, "lon": lon,
            "phone_speed_mps": _csv(phone_speed_mps, 3),
            "phone_course_deg": _csv(phone_course_deg, 1),
            "accuracy_m": _csv(accuracy_m, 2),
        }

        if not self.verbose:
            print(f"[FIX #{seq}] lat={lat:.6f} lon={lon:.6f} "
                  f"speed={phone_speed_mps:.2f} m/s "
                  f"acc={'--' if accuracy_m is None else f'{accuracy_m:.1f}'} m")

        # ---- Convert once -------------------------------------------------
        # convertGeo takes (lon, lat) order with fromGeo=True.
        try:
            x, y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
        except traci.exceptions.FatalTraCIError:
            raise
        except traci.TraCIException as e:
            print(f"[WARN] convertGeo failed for ({lat}, {lon}): {e}")
            rows = []
            for r in self.runners:
                row = self._blank_row(r, common)
                row["unmatched_reason"] = "convertGeo_failed"
                r.stats["fixes"] += 1
                r.stats["skipped"] += 1
                rows.append(row)
            self._write(rows)
            return

        # Heading sanitised once, before the filter (which seeds its
        # initial velocity from it).
        course_for_match = parse_course_deg(course_deg_raw)

        # ---- Kalman once (shared by every wkal bike) -----------------------
        filt_x, filt_y = x, y
        if self.kalman is not None:
            if fix_time is not None and self.last_fix_time is not None:
                dt = fix_time - self.last_fix_time
            else:
                dt = POLL_INTERVAL
            try:
                kf = self.kalman.update(
                    x, y, dt,
                    speed_mps=speed_mps,
                    course_deg=course_for_match,
                    accuracy_m=accuracy_m,
                )
                filt_x, filt_y = kf["x"], kf["y"]
            except Exception as e:
                print(f"[WARN] Kalman filter failed, wkal bikes use the raw point "
                      f"for this fix: {e}")
                filt_x, filt_y = x, y

        # Advance the fix clock whether or not the filter ran.
        if fix_time is not None:
            self.last_fix_time = fix_time

        # ---- Fan out -------------------------------------------------------
        rows = []
        placed = []      # (runner, row, console-context) for bikes moved this fix
        for r in self.runners:
            px, py = (filt_x, filt_y) if r.use_kalman else (x, y)
            ctx = None
            try:
                row, ctx = self._run_bike(
                    r, common, x, y, px, py,
                    speed_mps, course_for_match,
                    fix_time, accuracy_m,
                )
            except traci.exceptions.FatalTraCIError:
                raise
            except Exception:
                # One bike blowing up must not take the other nine with it.
                print(f"[ERROR] {r.vehicle_id} failed on fix #{seq}:")
                traceback.print_exc()
                row = self._blank_row(r, common)
                row.update({"raw_x": _csv(x, 4), "raw_y": _csv(y, 4),
                            "filt_x": _csv(px, 4), "filt_y": _csv(py, 4)})
                row["unmatched_reason"] = "bike_error"
                r.stats["fixes"] += 1
                r.stats["skipped"] += 1
            rows.append(row)
            if ctx is not None:
                placed.append((r, row, ctx))

        # Hold the whole fix's rows until the next simulationStep has applied
        # the moves; the main loop then calls finalize_pending().
        self._batch = (rows, placed)

        total_ms = (time.perf_counter() - t_fix) * 1000.0
        if len(self.runners) > 1 or self.verbose:
            print(f"[FIX #{seq}] {len(self.runners)} bike(s) processed in {total_ms:.1f} ms")
        if total_ms > POLL_INTERVAL * 1000.0:
            print(f"[WARN] Fix #{seq} took {total_ms:.0f} ms, longer than the "
                  f"{POLL_INTERVAL:.1f} s poll interval -- intermediate phone fixes "
                  f"will be missed. Run fewer bikes or lighten the slowest matcher.")

    # ------------------------------------------------------------------
    def finalize_pending(self, read_back=True):
        """Complete and write the held-back batch for the previous fix.

        WHY THIS EXISTS: moveToXY does not move the bike immediately -- SUMO
        applies it on the NEXT simulationStep(). Reading getPosition() right
        after moveToXY therefore returns where the bike was on the PREVIOUS
        fix. Calling this straight after simulationStep() reads the bike
        where SUMO really put it.

        read_back=False (shutdown / defensive flush) writes the rows with
        blank actual_* rather than logging a stale position.
        """
        if self._batch is None:
            return
        rows, placed = self._batch
        self._batch = None

        def _read(vid, fn):
            try:
                return fn(vid)
            except traci.exceptions.FatalTraCIError:
                raise
            except traci.TraCIException:
                return None

        for r, row, ctx in placed:
            actual_x = actual_y = sumo_speed = sumo_angle = None
            actual_edge = None
            if read_back:
                pos = _read(r.vehicle_id, traci.vehicle.getPosition)
                if pos is not None:
                    actual_x, actual_y = pos
                    # Guard against SUMO's INVALID_DOUBLE_VALUE sentinel
                    # (-1073741824) for a bike that is not yet on the road.
                    inv = traci.constants.INVALID_DOUBLE_VALUE
                    if actual_x <= inv or actual_y <= inv:
                        actual_x = actual_y = None
                sumo_speed = _read(r.vehicle_id, traci.vehicle.getSpeed)
                sumo_angle = _read(r.vehicle_id, traci.vehicle.getAngle)
                actual_edge = _read(r.vehicle_id, traci.vehicle.getRoadID)

            row["actual_x"], row["actual_y"] = _csv(actual_x, 4), _csv(actual_y, 4)
            row["sumo_speed_mps"] = _csv(sumo_speed, 3)
            if actual_edge is not None:
                row["actual_edge_id"] = actual_edge
                override = actual_edge != ctx["edge_id"]
                row["sumo_override"] = override
                # Only worth shouting about for the custom matchers: for
                # native, SUMO choosing the edge IS the method.
                if override and r.cfg["module"] is not None:
                    print(f"[OVERRIDE][{r.tag}] matcher chose {ctx['edge_id']} "
                          f"but SUMO placed the bike on {actual_edge or '(none)'}")
            self._print_ok(r, row, ctx, actual_x, actual_y, sumo_speed, sumo_angle)

        self._write(rows)

    def _print_ok(self, r, row, ctx, actual_x, actual_y, sumo_speed, sumo_angle):
        """One console line per successfully placed bike."""
        if self.verbose:
            speed = ctx["speed_mps"]
            phone_kmh = float(speed or 0.0) * 3.6
            act_str = (f"({actual_x:.1f}, {actual_y:.1f})"
                       if actual_x is not None else "(N/A)")
            sumo_speed_str = (f"{sumo_speed:.2f} m/s ({sumo_speed * 3.6:.2f} km/h)"
                              if sumo_speed is not None else "N/A")
            sumo_angle_str = f"{sumo_angle:.1f} deg" if sumo_angle is not None else "N/A"
            course = ctx["course"]
            course_str = "N/A" if course is None else f"{course:.1f} deg"
            comp = row["components"]
            win = ctx["window_len"]
            print(
                f"[OK][{r.tag}] {r.vehicle_id} -> edge={ctx['edge_id']}, "
                f"xy=({ctx['move_x']:.1f}, {ctx['move_y']:.1f}), act={act_str}, "
                f"raw=({ctx['x']:.1f}, {ctx['y']:.1f}), corr={ctx['correction_m']:.2f} m | "
                + (f"win={win} " if win is not None else "")
                + (comp + " | " if comp else "")
                + f"match={ctx['match_ms']:.1f} ms | "
                f"PHONE speed={float(speed or 0.0):.2f} m/s ({phone_kmh:.2f} km/h), "
                f"PHONE course={course_str} | "
                f"SUMO speed={sumo_speed_str}, SUMO angle={sumo_angle_str}"
            )
        else:
            print(f"[OK][{r.tag:<12}] edge={ctx['edge_id']} "
                  f"corr={ctx['correction_m']:6.2f} m match={ctx['match_ms']:7.1f} ms")

    def _blank_row(self, runner, common):
        row = dict(common)
        row.update({
            "vehicle_id": runner.vehicle_id,
            "method": runner.method_key,
            "kalman": runner.use_kalman,
            "matched": False,
            "is_internal_edge": False,
            "unmatched_reason": "",
        })
        return row

    def _write(self, rows):
        if self.csv_writer is None:
            return
        self.csv_writer.writerows(rows)
        # Flush every fix: a run ending with Ctrl+C or a SUMO crash would
        # otherwise lose whatever was still buffered, and these are ride
        # recordings that can't be regenerated.
        self.csv_file.flush()

    # ------------------------------------------------------------------
    def _run_bike(self, r, common, x, y, px, py, speed_mps, course_for_match,
                  fix_time, accuracy_m):
        """Match one fix for one bike and place the bike. Returns
        (csv_row, ctx): ctx is None unless the bike was actually moved, in
        which case the row is completed later by finalize_pending() once
        SUMO has applied the move. This is process_point() from
        post_processed_matching_v2.py plus the moveToXY half that only
        exists live."""
        row = self._blank_row(r, common)
        row.update({
            "raw_x": _csv(x, 4), "raw_y": _csv(y, 4),
            "filt_x": _csv(px, 4), "filt_y": _csv(py, 4),
        })
        r.stats["fixes"] += 1

        # ---- Run the matcher ---------------------------------------------
        # match_ms is the per-fix latency of THIS bike's matcher only.
        t0 = time.perf_counter()
        result = None
        try:
            if r.matcher is None:
                result = native_match(px, py)
            else:
                # Offer every optional input; keep only the ones this
                # matcher's match() declares.
                call_kwargs = {
                    "timestamp": fix_time,        # ST-Matching / HMM
                    "speed_mps": speed_mps,
                    "course_deg": course_for_match,
                    "accuracy_m": accuracy_m,     # HMM and fuzzy
                }
                call_kwargs = {k: v for k, v in call_kwargs.items()
                               if k in r.match_params}
                result = r.matcher.match(px, py, **call_kwargs)
        except traci.exceptions.FatalTraCIError:
            raise
        except Exception:
            print(f"[ERROR] {r.vehicle_id}: matcher raised:")
            traceback.print_exc()
            row["unmatched_reason"] = "matcher_error"
        match_ms = (time.perf_counter() - t0) * 1000.0
        row["match_ms"] = _csv(match_ms, 3)
        r.stats["ms_total"] += match_ms

        # No match -> this bike skips the fix and holds its last position.
        if result is None:
            row["unmatched_reason"] = row["unmatched_reason"] or "no_match"
            r.stats["skipped"] += 1
            print(f"[SKIP][{r.tag}] no match for ({px:.1f}, {py:.1f}) "
                  f"-- {row['unmatched_reason']} ({match_ms:.1f} ms)")
            return row, None

        edge_id = result["edge_id"]
        lane_index = result.get("lane_index", 0)
        row["edge_id"] = edge_id
        row["lane_index"] = lane_index
        row["is_internal_edge"] = bool(edge_id) and edge_id.startswith(":")
        row["raw_dist"] = _csv(result.get("raw_dist"), 4)
        row["score"] = _csv(result.get("score"), 6)
        row["components"] = _fmt_components(result.get("components"))
        row["window_len"] = "" if result.get("window_len") is None else result["window_len"]
        row["lane_pos"] = _csv(result.get("lane_pos"), 4)

        # ---- Where the matched point is -----------------------------------
        if r.matcher is None:
            # Native: convertRoad only gave edge/lane/pos -- go to the
            # network geometry for the actual point.
            snapped = snap_to_lane_geometry(
                self.net, edge_id, lane_index, result.get("lane_pos", 0.0))
            if snapped is None:
                # Live has no sensible fallback point to place a bike at
                # (v2 substitutes the unsnapped point offline); skip.
                row["unmatched_reason"] = "geometry_lookup_failed"
                r.stats["skipped"] += 1
                print(f"[SKIP][{r.tag}] geometry lookup failed on edge {edge_id}")
                return row, None
            move_x, move_y = snapped
        else:
            move_x, move_y = result["x"], result["y"]

        row["matched"] = True
        row["match_x"] = row["matched_x"] = _csv(move_x, 4)
        row["match_y"] = row["matched_y"] = _csv(move_y, 4)

        # Back-project the matched point through SUMO's own projection (no
        # pyproj needed). convertGeo returns (lon, lat).
        m_lon, m_lat = xy_to_lonlat(move_x, move_y)
        row["matched_lat"], row["matched_lon"] = _csv(m_lat, 8), _csv(m_lon, 8)

        # How far the matcher moved the point off the raw fix, and off the
        # (possibly filtered) point it was actually given.
        correction_m = math.hypot(move_x - x, move_y - y)
        row["correction_m"] = row["match_error_raw_m"] = _csv(correction_m, 4)
        row["match_error_filt_m"] = _csv(math.hypot(move_x - px, move_y - py), 4)
        r.stats["matched"] += 1

        # ---- Place the bike -----------------------------------------------
        if not spawn_bike_if_missing(rwd):
            row["unmatched_reason"] = "spawn_failed"
            return row, None

        angle_to_use = (course_for_match if course_for_match is not None
                        else traci.constants.INVALID_DOUBLE_VALUE)

        # NOTE: do NOT call setRoute() here. SUMO requires a replacement
        # route to contain the vehicle's CURRENT edge, so it fails on every
        # fix after the bike leaves its spawn edge. With keepRoute 0 or 4
        # SUMO snaps the bike and replaces the route itself as needed; the
        # edgeID argument is the hint that steers it onto the matcher's
        # edge (4 additionally ignores lane permissions).
        try:
            traci.vehicle.moveToXY(
                vehID=r.vehicle_id,
                edgeID=edge_id,
                laneIndex=lane_index,
                x=move_x,
                y=move_y,
                angle=angle_to_use,
                keepRoute=r.cfg["keep_route"],
                matchThreshold=MATCH_THRESHOLD,
            )
        except traci.exceptions.FatalTraCIError:
            raise
        except traci.TraCIException as e:
            print(f"[WARN] moveToXY failed for {r.vehicle_id}: {e}")
            row["unmatched_reason"] = "moveToXY_failed"
            return row, None

        # Phone speed onto the vehicle (cosmetic; position comes from
        # moveToXY).
        try:
            traci.vehicle.setSpeed(r.vehicle_id, max(0.0, float(speed_mps or 0.0)))
        except traci.exceptions.FatalTraCIError:
            raise
        except traci.TraCIException:
            pass

        # The read-back (actual_x/y, SUMO speed/angle) and the console line
        # happen in finalize_pending(), after the next simulationStep().
        ctx = {
            "edge_id": edge_id, "move_x": move_x, "move_y": move_y,
            "x": x, "y": y, "correction_m": correction_m, "match_ms": match_ms,
            "speed_mps": speed_mps, "course": course_for_match,
            "window_len": result.get("window_len"),
        }
        return row, ctx


# ======================================================================
# FLASK POLLING
# ======================================================================
def get_latest_phone_data(url: str):
    """GET the latest phone fix from Flask. Returns the parsed JSON dict, or
    None on any failure. Never raises, so a transient Flask hiccup can't
    kill the simulation loop."""
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
    """True if the server received the fix within the last `stale_seconds`.
    Uses server_received_at (set by Flask on arrival) rather than the
    phone's own timestamp, to avoid clock skew between devices."""
    received = data.get("server_received_at")
    if not received:
        return False
    try:
        t = received.replace("Z", "+00:00")
        dt = datetime.fromisoformat(t)
        now = datetime.now(dt.tzinfo) if dt.tzinfo else datetime.now()
        return (now - dt).total_seconds() <= stale_seconds
    except Exception:
        return False


# ======================================================================
# LOGGING
# ======================================================================
def open_run_log(runners):
    """Open the single CSV for this run and write its header. One file, one
    row per bike per fix. Named by what is running so repeated runs never
    overwrite each other. Returns (file, writer, path)."""
    log_dir = os.path.join(SCRIPT_DIR, LOG_DIR)
    os.makedirs(log_dir, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    if len(runners) == 1:
        r = runners[0]
        name = f"{r.method_key}{'_kalman' if r.use_kalman else ''}_{stamp}.csv"
    elif len(runners) == MAX_BIKES:
        name = f"all_methods_{stamp}.csv"
    else:
        name = f"multi_{len(runners)}bikes_{stamp}.csv"
    path = os.path.join(log_dir, name)

    # newline="" is required on Windows or csv writes blank rows.
    f = open(path, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(f, fieldnames=FIELDNAMES, restval="", extrasaction="raise")
    writer.writeheader()
    f.flush()
    print(f"[INFO] Logging to {path}")
    return f, writer, path


def print_summary(runners):
    """Per-bike totals at shutdown."""
    print("\n" + "=" * 66)
    print(" Run summary")
    print("=" * 66)
    print(f"  {'bike':<24}{'fixes':>7}{'matched':>9}{'skipped':>9}{'mean ms':>10}")
    for r in runners:
        s = r.stats
        mean = s["ms_total"] / s["fixes"] if s["fixes"] else 0.0
        print(f"  {r.vehicle_id:<24}{s['fixes']:>7}{s['matched']:>9}"
              f"{s['skipped']:>9}{mean:>10.1f}")
    print("=" * 66)


# ======================================================================
# CLI / MAIN
# ======================================================================
def parse_args():
    """Command-line interface. Everything has a sensible default."""
    p = argparse.ArgumentParser(
        description="eBike-in-the-Loop live bridge: up to 10 bikes (5 map-"
                    "matching methods x Kalman on/off) driven by one phone.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Methods:\n" + "\n".join(
            f"  {k:<8} {v['label']}" for k, v in METHODS.items()
        ) + f"\n  {ALL_KEYWORD:<8} all five methods (10 bikes unless a Kalman "
            "flag restricts it)",
    )
    p.add_argument("-m", "--method", default=None,
                   help="method name, comma-separated list (e.g. topo,st) or "
                        "'all'; prompts if omitted")
    # Tri-state on purpose: None means 'not specified', which triggers the
    # interactive prompt (or, for --method all, both variants).
    p.add_argument("--kalman", dest="kalman_mode", action="store_const",
                   const="on", default=None,
                   help="Kalman pre-filter ON for every selected method")
    p.add_argument("--no-kalman", dest="kalman_mode", action="store_const",
                   const="off", help="Kalman pre-filter OFF (no prompt)")
    p.add_argument("--both-kalman", dest="kalman_mode", action="store_const",
                   const="both",
                   help="one bike with and one without Kalman per method")
    p.add_argument("--log", action="store_true",
                   help="write the CSV log (already the default; kept so "
                        "old command lines still work)")
    p.add_argument("--no-log", action="store_true",
                   help="do NOT write a CSV log")
    p.add_argument("--no-gui", action="store_true",
                   help="run headless (sumo instead of sumo-gui)")
    p.add_argument("--sumo-log", action="store_true",
                   help=f"have SUMO write its own messages to {SUMO_LOG_FILE} "
                        "next to this script (for diagnosing GUI crashes)")
    p.add_argument("--cfg", default=SUMO_CFG,
                   help="path to the .sumocfg, relative to this script")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="full per-bike detail line every fix (always on when "
                        "only one bike is running)")
    return p.parse_args()


def main():
    args = parse_args()

    # ---- Decide which bikes to run --------------------------------------
    method_keys, variants = resolve_selection(args)
    runners = [Runner(m, k) for m in method_keys for k in variants]
    if not 1 <= len(runners) <= MAX_BIKES:
        raise ValueError(f"{len(runners)} bikes selected; must be 1..{MAX_BIKES}")

    sumocfg_abs = os.path.abspath(os.path.join(SCRIPT_DIR, args.cfg))
    if not os.path.exists(sumocfg_abs):
        raise FileNotFoundError(f"SUMO config not found: {sumocfg_abs}")
    net_file = parse_sumocfg_for_netfile(sumocfg_abs)

    print("\n" + "-" * 66)
    print(f"[INFO] Bikes ({len(runners)}):")
    for r in runners:
        print(f"         {r.vehicle_id:<24} {r.cfg['label']}"
              f"{'  + Kalman' if r.use_kalman else ''}"
              f"  [keepRoute={r.cfg['keep_route']}]")
    print(f"[INFO] SUMO config: {sumocfg_abs}")
    print(f"[INFO] Net file:    {net_file}")
    print("-" * 66)

    # ---- Load the network and build everything BEFORE SUMO starts, so a
    # missing module fails fast rather than after the GUI has opened. ------
    print("[INFO] Loading network geometry with sumolib...")
    # withInternal=True so junction-internal edges (IDs starting ':') have
    # usable lane geometry for native matching.
    net = sumolib.net.readNet(net_file, withInternal=True)

    announced = set()
    for i, r in enumerate(runners, start=1):
        if r.cfg["module"] is None:
            print(f"[INFO] ({i}/{len(runners)}) {r.tag}: native SUMO matching "
                  f"-- no external matcher loaded.")
            continue
        print(f"[INFO] ({i}/{len(runners)}) {r.tag}: loading network into "
              f"{r.cfg['cls']}...")
        build_matcher(r, net_file, announced)
        shown = ", ".join(f"{k}={v}" for k, v in r.used_kwargs.items())
        print(f"[INFO]        {type(r.matcher).__name__} ready ({shown}).")

    kalman = None
    if any(r.use_kalman for r in runners):
        kalman = build_kalman()
        print("[INFO] Kalman pre-filter active (one shared instance).")

    csv_file = csv_writer = None
    if not args.no_log:
        csv_file, csv_writer, _ = open_run_log(runners)

    session = LiveSession(net, runners, kalman, csv_file, csv_writer)
    session.verbose = args.verbose or len(runners) == 1

    print("[INFO] Starting SUMO...")

    # Launch SUMO under TraCI's control:
    #   --start                -> begin stepping immediately
    #   --delay                -> GUI pacing (handled by SUMO's own loop; a
    #                             manual sleep() would starve the GUI)
    #   --end 1000000          -> effectively never auto-terminate
    #   --collision.action none-> the bikes sit almost on top of each other
    #                             by design; without this SUMO may treat the
    #                             overlap as a collision and teleport/remove
    #                             one of them
    #   --time-to-teleport -1  -> never jam-teleport a bike that is standing
    #                             still because the rider is
    binary = "sumo" if args.no_gui else "sumo-gui"
    cmd = [
        binary,
        "-c", sumocfg_abs,
        "--step-length", str(SUMO_STEP_LENGTH),
        "--start",
        "--end", "1000000",
        "--collision.action", "none",
        "--time-to-teleport", "-1",
    ]
    if not args.no_gui:
        cmd += ["--delay", SUMO_DELAY_MS]
    if args.sumo_log:
        sumo_log_path = os.path.join(SCRIPT_DIR, SUMO_LOG_FILE)
        cmd += ["--log", sumo_log_path, "--verbose"]
        print(f"[INFO] SUMO messages -> {sumo_log_path}")

    traci.start(cmd)

    # last_seen_timestamp dedupes fixes; last_poll_time rate-limits Flask.
    last_seen_timestamp = None
    last_poll_time = 0.0

    try:
        ensure_vehicle_type()
        prepare_spawn_route()

        # ---- Main real-time loop ----
        while True:
            traci.simulationStep()
            # SUMO has now applied last fix's moveToXY calls: read the bikes
            # back and write that fix's rows.
            session.finalize_pending()

            now = time.time()
            if now - last_poll_time >= POLL_INTERVAL:
                last_poll_time = now

                data = get_latest_phone_data(FLASK_LATEST_URL)

                if not phone_data_is_valid(data):
                    print("[WARN] No valid phone data yet")
                elif not phone_data_is_fresh(data, STALE_DATA_SECONDS):
                    print("[WARN] Latest phone data is stale; ignoring")
                else:
                    phone_timestamp = data.get("phone_timestamp")

                    # Only act on a fix we haven't already processed.
                    if phone_timestamp != last_seen_timestamp:
                        last_seen_timestamp = phone_timestamp

                        lat = float(data["lat"])
                        lon = float(data["lon"])

                        try:
                            speed_mps = float(data.get("speed_mps", 0.0) or 0.0)
                        except Exception:
                            speed_mps = 0.0

                        course_deg = data.get("course_deg")

                        # Phone-reported horizontal accuracy (per-fix sigma
                        # for the HMM, fuzzy and Kalman). None = use default.
                        try:
                            accuracy_m = data.get("accuracy_m")
                            accuracy_m = None if accuracy_m is None else float(accuracy_m)
                        except Exception:
                            accuracy_m = None

                        fix_time = phone_timestamp_seconds(phone_timestamp)

                        session.process_fix(
                            lat, lon, speed_mps, course_deg, fix_time, accuracy_m
                        )

            # Headless has no --delay pacing, so the loop would otherwise run
            # the simulation flat out (hundreds of sim-seconds per phone
            # fix). Remote control of a bike lapses after a few idle sim
            # seconds and SUMO then drops the off-route bike, so pace
            # headless runs exactly like the GUI's --delay. (GUI runs never
            # sleep here: that would starve sumo-gui's event loop.)
            if args.no_gui:
                time.sleep(float(SUMO_DELAY_MS) / 1000.0)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    except traci.exceptions.FatalTraCIError as e:
        print(f"\n[INFO] SUMO connection closed ({e}).")
    finally:
        # Close the log first -- if TraCI shutdown hangs, the ride data is
        # already safely on disk. The last fix's rows are still held back
        # (its moves were never stepped), so write them with blank actual_*.
        try:
            session.finalize_pending(read_back=False)
        except Exception:
            pass
        if csv_file is not None:
            try:
                csv_file.close()
            except Exception:
                pass
        try:
            traci.close()
        except Exception:
            pass
        print_summary(runners)


if __name__ == "__main__":
    main()