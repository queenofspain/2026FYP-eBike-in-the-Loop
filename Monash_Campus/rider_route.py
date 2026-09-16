"""Route-aware road context for the SUMO demo with a known bike route."""

from rider_advice import positive


def upcoming_light(traci, vehicle_id):
    result = {"traffic_light_status": "unknown", "traffic_light_id": None,
              "traffic_light_distance_m": None, "traffic_light_state": None,
              "traffic_light_link_index": None}
    try:
        lights = traci.vehicle.getNextTLS(vehicle_id)
        if not lights:
            result["traffic_light_status"] = "none"
            return result
        valid = [item for item in lights if positive(item[2]) is not None]
        if valid:
            light_id, link, distance, state = min(valid, key=lambda item: item[2])
            result.update(traffic_light_status="ahead", traffic_light_id=light_id,
                          traffic_light_distance_m=distance, traffic_light_state=state,
                          traffic_light_link_index=link)
    except Exception:
        # Failed queries are unknown, never evidence that the road has no light.
        pass
    return result


def is_intersection(node):
    """Skip geometric road splits with only two distinct neighboring nodes."""
    neighbors = {edge.getFromNode().getID() for edge in node.getIncoming()}
    neighbors.update(edge.getToNode().getID() for edge in node.getOutgoing())
    return (len(neighbors) > 2 or node.getType().startswith("traffic_light")
            or node.getType() in {"allway_stop", "priority_stop", "right_before_left"})


def upcoming_intersection(traci, network, vehicle_id):
    result = {"intersection_status": "unknown", "intersection_id": None,
              "intersection_distance_m": None}
    try:
        if network is None:
            return result
        current_edge = traci.vehicle.getRoadID(vehicle_id)
        if current_edge.startswith(":"):
            result["intersection_status"] = "inside"
            return result
        route = traci.vehicle.getRoute(vehicle_id)
        index = traci.vehicle.getRouteIndex(vehicle_id)
        if index < 0:
            return result
        lane_id = traci.vehicle.getLaneID(vehicle_id)
        for edge_id in route[index:]:
            edge = network.getEdge(edge_id)
            node = edge.getToNode()
            if not is_intersection(node):
                continue
            # Current lane when known, otherwise a bicycle-permitted approach lane.
            lanes = [lane for lane in edge.getLanes() if lane.allows("bicycle")]
            lane = network.getLane(lane_id) if edge_id == current_edge else (lanes[0] if lanes else None)
            if lane is None:
                return result
            distance = positive(traci.vehicle.getDrivingDistance(vehicle_id, edge_id, lane.getLength(), lane.getIndex()))
            if distance is not None:
                result.update(intersection_status="ahead", intersection_id=node.getID(), intersection_distance_m=distance)
            return result
        result["intersection_status"] = "none"
    except Exception:
        pass
    return result
