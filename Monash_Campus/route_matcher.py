"""
route_matcher.py
======================================================================
RouteMatcher -- ground-truth generator, NOT a real map-matcher.

Projects each raw GPS fix onto a predefined, ordered route instead of
choosing among candidates. Meant to be run via --replay against a
previously-recorded live CSV, so GT ends up on the exact same fixes
(same phone_timestamp) as every other method -- no time alignment
needed downstream.

Registry entry for live_phone_to_sumo.py:

    "route": {
        "module": "route_matcher",
        "cls": "RouteMatcher",
        "kwargs": {"route_edges": GROUND_TRUTH_ROUTE, "back_tolerance_m": 5.0},
        "keep_route": 6,
        "label": "Ground truth (route projection)",
    },
"""

# EDIT ME: the planned route, as ordered edge IDs from the .net.xml
# (read them off in netedit, or from a duarouter run over your waypoints).
GROUND_TRUTH_ROUTE = [
    "-1272942443#2",
    "-1272942443#1",
    "-1272942443#0",
    "-996237862#0",
    "1209653411",
    "1209653410",
    "1147258794#0",
    "1147258794#2",
    "5094886#0",
    "5094886#1",
    "5094886#2",
    "985461732",
    "-10539447#1",
    "-10539447#0",
    "-4577073#9",
    "-4577073#4",
    "-4577073#3",
    "-4577073#2",
    "-4577073#1",
    "-4577073#0",
    "491516483#0",
    "491516483#1",
    "1470433047#2",
    "1470433046#0",
    "1470433046#1",
    "1470433046#2",
    "1470433046#3",
    "1360476884",
    "1470433048",
    "711979119",
    "-1360476884",
    "-1470433046#3",
    "-1470433046#2",
    "-1470433046#1",
    "-1470433046#0",
    "1470433047#3",
    "-93686025#2",
    "-93686025#0",
    "-93686034#0",
    "-46129059#4",
    "-46129059#3",
    "-46129059#2",
    "-46129059#1",
    "-46129059#0",
    "44348023#1",
    "1038136215",
    "139327110#0",
    "44348024#1",
    "1209622645",
    "569766862#0",
    "569766862#0-AddedOffRampEdge",
    "44348588#0",
    "44348588#1",
    "139332152#2",
    "139332152#3",
    "5322853#4",
    "5322853#5",
    "5322853#6",
    "5322853#7",
    "5322853#0",
    "988996007#0",
    "988996007#1",
    "630680457#0",
    "630680457#1",
    "630680457#2",
    "630680457#3",
    "630680457#4",
    "630680457#5",
    "630680457#7",
    "-630680455#1",
    "-630680455#0",
    "-10585186",
    "10585197#1",
    "446272714",
    "116895895#9",
    "116895895#10",
    "116895895#11",
    "116895895#12",
    "-116895895#12",
    "48114467",
    "605959218#0",
    "-36667189#3",
    "-36667189#2",
    "-36667189#1",
    "-36667189#0",
    "-73530943#5",
    "36667190#0",
    "36667190#1",
    "36667190#2",
    "146181190",
    "299173860#1",
    "146181188#2",
    "146181188#1",
    "146181188#0",
    "-46410623#1",
    "-46410623#0",
    "24689623#1",
    "46410626#2",
    "46410626#1",
    "46410626#0",
    "115936847",
    "310292416#2",
    "310292416#3",
    "4577076#3",
    "1209653405",
    "5094891#0",
    "5094891#1",
    "1209653407",
    "24801549#0",
    "24801549#1",
    "24801549#3",
    "24801549#4",
    "1209653409#0",
    "-5094896#1",
    "-5094896#0",
    "4577074#0",
    "4577074#1",
    "4577074#2",
    "405414870#0",
    "405414870#2",
    "405414870#3",
    "405414870#4",
    "998107627#0",
    "998107627#1",
    "998107627#2",
    "998107627#3",
    "998107627#4",
    "-403774582#0",
    "-403774582#1",
    "-403774582#2",
    "-403774582#3",
    "988580237#0",
    "-988580235",
    "-773028723",
    "298236180#1",
    "773028722",
    "-988580230#1",
    "-298236187#1",
    "-298236187#0",
    "298236187#0",
    "298236187#1",
    "988580230#1",
    "988580230#2",
    "988580235",
    "-988580237#0",
    "403774582#3",
    "403774582#2",
    "403774582#1",
    "403774582#0",
    "1004615767#2",
    "1004615767#1",
    "1004615767#0",
    "1004615766#0",
    "1004615766#1",
    "1004615766#2",
    "1004615766#3",
    "1004615766#4",
    "10801654#0",
    "10801654#1",
    "10801654#2",
    "10801654#3",
    "10801654#4",
    "10801654#5",
    "10801654#6",
    "-11527061#5",
    "-11527061#3",
    "-11527061#2",
    "-11527061#0",
    "-996576401",
    "45606689#0",
    "-1164319799#0",
    "-45897352#1",
    "-1272942442#0",
    "-250248456",
    "-1272942443#3"
]

import sumolib


class RouteMatcher:
    def __init__(self, net_file, route_edges=None, back_tolerance_m=5.0,
                 vclass=None):
        if not route_edges:
            raise ValueError(
                "RouteMatcher needs route_edges -- edit GROUND_TRUTH_ROUTE "
                "at the top of route_matcher.py."
            )

        self.net = sumolib.net.readNet(net_file)
        self.back_tolerance_m = back_tolerance_m

        # Concatenate the route edges' shapes, in order, into one polyline:
        # each point is (x, y, cumulative_s, edge_id).
        self._points = []
        s = 0.0
        prev_xy = None
        for edge_id in route_edges:
            try:
                edge = self.net.getEdge(edge_id)
            except KeyError:
                raise ValueError(
                    f"Edge '{edge_id}' not found in {net_file}. "
                    f"Check GROUND_TRUTH_ROUTE against the .net.xml."
                )
            if vclass and not edge.allows(vclass):
                print(f"[WARN] Route edge '{edge_id}' disallows vclass "
                      f"'{vclass}' -- kept anyway, GT reflects the PLANNED "
                      f"route, not what a real matcher could legally pick.")

            for x, y in edge.getShape():
                if prev_xy is not None:
                    seg = ((x - prev_xy[0]) ** 2 + (y - prev_xy[1]) ** 2) ** 0.5
                    if seg < 1e-6:
                        continue  # duplicate point at an edge boundary
                    s += seg
                self._points.append((x, y, s, edge_id))
                prev_xy = (x, y)

        if len(self._points) < 2:
            raise ValueError("Route produced <2 shape points -- check "
                              "GROUND_TRUTH_ROUTE.")

        self.total_length = self._points[-1][2]
        self._last_s = None  # progress-along-route, carried across calls

    def match(self, x, y, timestamp=None, speed_mps=None,
              course_deg=None, accuracy_m=None):
        """
        Nearest point on the route polyline, restricted to
        s >= last_s - back_tolerance_m so ordinary GPS jitter doesn't
        fail, but a single bad fix can't snap GT back across a junction.
        """
        lo_s = (0.0 if self._last_s is None
                 else max(0.0, self._last_s - self.back_tolerance_m))

        best = None  # (dist2, px, py, s, edge_id)
        for i in range(len(self._points) - 1):
            x1, y1, s1, e1 = self._points[i]
            x2, y2, s2, _ = self._points[i + 1]
            if s2 < lo_s:
                continue

            dx, dy = x2 - x1, y2 - y1
            t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)
            t = max(0.0, min(1.0, t))
            px, py = x1 + t * dx, y1 + t * dy
            s_here = s1 + t * (s2 - s1)
            if s_here < lo_s:
                continue

            d2 = (x - px) ** 2 + (y - py) ** 2
            if best is None or d2 < best[0]:
                best = (d2, px, py, s_here, e1)

        if best is None:
            # Route has ended -- snap to its final point rather than
            # returning None, so GT never has a hole the CMP script has
            # to special-case.
            x1, y1, s1, e1 = self._points[-1]
            best = (0.0, x1, y1, s1, e1)

        _, px, py, s_here, edge_id = best
        self._last_s = s_here

        return {
            "x": px, "y": py, "edge_id": edge_id, "lane_index": 0,
            "raw_dist": best[0] ** 0.5,   # GPS's perpendicular offset from truth
            "score": s_here,
            "components": {"s_m": round(s_here, 2),
                            "route_frac": round(s_here / self.total_length, 4)},
            "window_len": None,
        }