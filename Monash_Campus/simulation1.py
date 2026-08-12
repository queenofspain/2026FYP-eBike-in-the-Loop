import os
import sys
import time

# ============================================================
# USER SETTINGS
# ============================================================

SUMO_CFG = r"2026-03-11-17-20-46/osm.sumocfg"

# ROUTE_ID = "block_route"
ROUTE_ID = "full_campus_v2"
VEHICLE_ID = f"simBike_{int(time.time())}"

DEPART_SPEED = 2.5          # m/s
SIM_STEP = 0.1              # seconds

# ============================================================

if "SUMO_HOME" not in os.environ:
    raise EnvironmentError("SUMO_HOME is not set.")

SUMO_HOME = os.environ["SUMO_HOME"]

TOOLS = os.path.join(SUMO_HOME, "tools")
if TOOLS not in sys.path:
    sys.path.append(TOOLS)

import traci


def main():

    print("[INFO] Starting SUMO...")

    traci.start([
        "sumo-gui",
        "-c", SUMO_CFG,
        "--step-length", str(SIM_STEP),
        "--start"
    ])

    # --------------------------------------------------------
    # Wait until routes are loaded
    # --------------------------------------------------------
    traci.simulationStep()

    # --------------------------------------------------------
    # Check route exists
    # --------------------------------------------------------
    routes = traci.route.getIDList()

    if ROUTE_ID not in routes:
        print(f"[ERROR] Route '{ROUTE_ID}' not found.")
        print("Available routes:")
        for r in routes:
            print("  ", r)

        traci.close()
        return

    print(f"[INFO] Found route: {ROUTE_ID}")

    # --------------------------------------------------------
    # Spawn bike
    # --------------------------------------------------------

    traci.vehicle.add(
        vehID=VEHICLE_ID,
        routeID=ROUTE_ID,
        typeID="DEFAULT_BIKETYPE",
        depart="now",
        departLane="best",
        departPos="base",
        departSpeed=str(DEPART_SPEED)
    )

    print(f"[INFO] Spawned vehicle '{VEHICLE_ID}'")

    # Set bike speed for testing
    traci.vehicle.setMaxSpeed(VEHICLE_ID, 50.0)

    # --------------------------------------------------------
    # Simulation loop
    # --------------------------------------------------------
    while True:

        traci.simulationStep()

        try:
            x, y = traci.vehicle.getPosition(VEHICLE_ID)
            speed = traci.vehicle.getSpeed(VEHICLE_ID)
            edge = traci.vehicle.getRoadID(VEHICLE_ID)

            print(
                f"edge={edge} | "
                f"x={x:.2f}, y={y:.2f} | "
                f"speed={speed:.2f} m/s"
            )

        except traci.TraCIException:
            print("[INFO] Vehicle arrived.")
            break

        time.sleep(SIM_STEP)

    traci.close()


if __name__ == "__main__":
    main()