import argparse
import gzip
import os
import sys
import xml.etree.ElementTree as ET

import requests


DEFAULT_SUMO_CFG = "2026-03-11-17-20-46/osm.sumocfg"
DEFAULT_FLASK_URL = "http://localhost:5000"

EXPECTED_FEEDBACK_FIELDS = [
    "vehicle_id",
    "sim_time",
    "matched",
    "edge_id",
    "road_id",
    "lane_id",
    "lane_index",
    "sumo_speed_mps",
    "sumo_speed_kmh",
    "allowed_speed_mps",
    "angle_deg",
    "traffic_vehicle_count",
    "leader_gap_m",
    "phone_data_age_s",
    "feedback_level",
    "feedback_message",
]


def ok(message):
    print(f"[OK] {message}")


def warn(message):
    print(f"[WARN] {message}")


def fail(message):
    print(f"[FAIL] {message}")
    return False


def check_file(path, label):
    if os.path.exists(path):
        ok(f"{label}: {path}")
        return True
    return fail(f"{label} missing: {path}")


def parse_sumocfg(path):
    tree = ET.parse(path)
    root = tree.getroot()
    base_dir = os.path.dirname(os.path.abspath(path))

    def values(section_name, tag_name):
        section = root.find(section_name)
        if section is None:
            return []
        tag = section.find(tag_name)
        if tag is None:
            return []
        raw = tag.get("value", "")
        return [item.strip() for item in raw.split(",") if item.strip()]

    net_files = values("input", "net-file")
    route_files = values("input", "route-files")
    additional_files = values("input", "additional-files")

    outputs = {}
    output_section = root.find("output")
    if output_section is not None:
        for tag in output_section:
            outputs[tag.tag] = tag.get("value")

    def resolve(items):
        return [item if os.path.isabs(item) else os.path.join(base_dir, item) for item in items]

    return {
        "base_dir": base_dir,
        "net_files": resolve(net_files),
        "route_files": resolve(route_files),
        "additional_files": resolve(additional_files),
        "outputs": outputs,
    }


def check_gzip_xml(path):
    try:
        with gzip.open(path, "rb") as f:
            f.read(128)
        ok(f"gzip-readable XML: {path}")
        return True
    except Exception as e:
        return fail(f"cannot read gzip XML {path}: {e}")


def check_flask(base_url):
    try:
        response = requests.get(f"{base_url}/health", timeout=2)
        response.raise_for_status()
        data = response.json()
        ok(f"Flask health endpoint reachable: {base_url}/health")

        missing = [
            key for key in ("feedback_page", "feedback_update", "feedback_latest")
            if key not in data
        ]
        if missing:
            return fail(f"Flask health response missing keys: {', '.join(missing)}")

        page = requests.get(f"{base_url}/feedback", timeout=2)
        page.raise_for_status()
        ok(f"feedback page reachable: {base_url}/feedback")
        return True
    except Exception as e:
        return fail(f"Flask is not ready at {base_url}: {e}")


def check_sumo_home():
    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        return fail("SUMO_HOME is not set")

    ok(f"SUMO_HOME={sumo_home}")
    tools = os.path.join(sumo_home, "tools")
    if not os.path.isdir(tools):
        return fail(f"SUMO tools folder missing: {tools}")

    if tools not in sys.path:
        sys.path.append(tools)

    try:
        import traci  # noqa: F401
        ok("TraCI Python module imports correctly")
        return True
    except Exception as e:
        return fail(f"cannot import TraCI from SUMO_HOME tools: {e}")


def check_feedback_schema():
    ok("Expected real-SUMO feedback fields:")
    for field in EXPECTED_FEEDBACK_FIELDS:
        print(f"     - {field}")
    return True


def main():
    parser = argparse.ArgumentParser(description="Check whether real SUMO feedback run is ready.")
    parser.add_argument("--sumocfg", default=DEFAULT_SUMO_CFG, help="SUMO config path.")
    parser.add_argument("--flask-url", default=DEFAULT_FLASK_URL, help="Flask base URL.")
    args = parser.parse_args()

    all_ok = True
    sumocfg = os.path.abspath(args.sumocfg)

    all_ok &= check_file(sumocfg, "SUMO config")

    if os.path.exists(sumocfg):
        try:
            cfg = parse_sumocfg(sumocfg)
            for net_file in cfg["net_files"]:
                all_ok &= check_file(net_file, "SUMO net file")
                if net_file.endswith(".gz") and os.path.exists(net_file):
                    all_ok &= check_gzip_xml(net_file)
            for route_file in cfg["route_files"]:
                all_ok &= check_file(route_file, "SUMO route file")
            for additional_file in cfg["additional_files"]:
                all_ok &= check_file(additional_file, "SUMO additional file")
            for output_name, output_path in cfg["outputs"].items():
                ok(f"SUMO configured output {output_name}: {output_path}")
        except Exception as e:
            all_ok &= fail(f"could not parse SUMO config: {e}")

    all_ok &= check_sumo_home()
    all_ok &= check_flask(args.flask_url)
    all_ok &= check_feedback_schema()

    if all_ok:
        print("\nREADY: web feedback path and SUMO files look ready for real SUMO testing.")
    else:
        print("\nNOT READY: fix the failed checks above before real SUMO testing.")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
