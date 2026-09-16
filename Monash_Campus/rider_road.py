"""Conservative road lookahead for a GPS-placed bicycle in SUMO.

The live bridge has a dummy spawn route. We therefore follow only the current
edge and unambiguous continuations. At a possible turn, farther road context
is unknown until the next GPS fix establishes the chosen edge.
"""

import math


MAX_LOOKAHEAD_M = 250.0


def _project_offset(shape, point):
    """Distance along a lane centreline to the closest projected point."""
    px, py = point
    walked = 0.0
    best_distance = math.inf
    best_offset = 0.0
    for (ax, ay), (bx, by) in zip(shape, shape[1:]):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        if not length2:
            continue
        fraction = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
        segment_length = math.sqrt(length2)
        distance = math.hypot(px - ax - fraction * dx, py - ay - fraction * dy)
        if distance < best_distance:
            best_distance = distance
            best_offset = walked + fraction * segment_length
        walked += segment_length
    return best_offset


def _intersection(node):
    neighbors = {edge.getFromNode().getID() for edge in node.getIncoming()}
    neighbors.update(edge.getToNode().getID() for edge in node.getOutgoing())
    return (len(neighbors) > 2 or node.getType().startswith("traffic_light")
            or node.getType() in {"allway_stop", "priority_stop", "right_before_left"})


def collect(traci, network, vehicle_id, speed_mps=None, course_deg=None, fix_time=None):
    """Read motion and road context after a successful moveToXY call."""
    edge_id = traci.vehicle.getRoadID(vehicle_id)
    lane_id = traci.vehicle.getLaneID(vehicle_id)
    x, y = traci.vehicle.getPosition(vehicle_id)
    sample = {
        "edge_id": edge_id,
        "lane_id": lane_id,
        "x": x,
        "y": y,
        "time_s": traci.simulation.getTime(),
        "speed_mps": speed_mps,
        "angle_deg": course_deg,
        "fix_time": fix_time,
        "speed_limit_mps": None,
        "intersection_status": "unknown",
        "intersection_id": None,
        "intersection_distance_m": None,
        "traffic_light_status": "unknown",
        "traffic_light_id": None,
        "traffic_light_distance_m": None,
        "traffic_light_state": None,
        "route_source": "current_edge_and_unambiguous_continuations",
    }
    if lane_id:
        try:
            sample["speed_limit_mps"] = traci.lane.getMaxSpeed(lane_id)
        except Exception:
            pass
    if edge_id.startswith(":"):
        sample["intersection_status"] = "inside"
        return sample
    try:
        edge = network.getEdge(edge_id)
        lane = network.getLane(lane_id)
        offset = _project_offset(lane.getShape(), (x, y))
        distance = max(0.0, lane.getLength() - offset)
    except Exception:
        return sample

    visited = set()
    while edge.getID() not in visited and distance <= MAX_LOOKAHEAD_M:
        visited.add(edge.getID())
        node = edge.getToNode()
        if _intersection(node):
            sample.update(intersection_status="ahead", intersection_id=node.getID(),
                          intersection_distance_m=distance)
            if node.getType().startswith("traffic_light"):
                sample.update(traffic_light_status="ahead", traffic_light_id=node.getID(),
                              traffic_light_distance_m=distance)
            # A rider's turn beyond this node is unknown. Do not claim there
            # are no traffic lights on the remaining route.
            return sample
        outgoing = [candidate for candidate in edge.getOutgoing()
                    if any(lane.allows("bicycle") for lane in candidate.getLanes())]
        if not outgoing:
            sample.update(intersection_status="none", traffic_light_status="none")
            return sample
        if len(outgoing) != 1:
            return sample
        edge = outgoing[0]
        distance += edge.getLength()
    return sample
