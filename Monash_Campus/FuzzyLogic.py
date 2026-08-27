"""
FuzzyLogic.py
------------------------------------------------------------------
Fuzzy logic map-matching, based on Ren & Karimi (2012), "A fuzzy logic
map matching for wheelchair navigation" (GPS Solutions 16(3), 273-282),
with a temporal-consensus check and a stationary freeze adapted from
Jagadeesh, Srikanthan & Zhang (2004), "A map matching method for GPS
based real-time vehicle location" (Journal of Navigation 57(3), 429-440).

Each candidate road edge is scored on two fuzzy inputs -- perpendicular
distance and heading difference to the edge -- combined through four
IF-THEN rules (min for AND, weighted average for the crisp output) into
a single likelihood score. A small state machine tracks whether the
vehicle is still plausibly on its current edge ("following") or needs a
new edge chosen from the ones connected at the upcoming junction
("entering").

Call match() once per GPS fix (SUMO network x, y, plus optional course
and speed). Call reset() between separate routes.

match() returns a dict {x, y, edge_id, raw_dist, score, components}, or
None if no candidate edge lies within search_radius -- the caller should
fall back to SUMO's native matching for that one fix.
------------------------------------------------------------------
"""

import math

import sumolib
from sumolib import geomhelper as gh


class FuzzyMatcher:
    def __init__(
        self,
        net_file,
        search_radius=50.0,        # m; hard cutoff for candidate edge lookup
        dist_half_width=2.5,       # m; distance at which short/long membership = 0.5
        angle_small_break=25.0,    # deg; below this, "small" heading-diff membership = 1
        angle_large_break=65.0,    # deg; above this, "small" heading-diff membership = 0
        junction_threshold=5.0,    # m; min clearance to the nearer end-node before
                                    # switching into junction ("entering") mode
        min_speed_for_switch=0.5,  # m/s; below this the matcher never changes edge
        confirm_count=2,           # consecutive fixes a new edge must win before it replaces the current one
        output_low=10.0,
        output_average=50.0,
        output_high=100.0,
        vclass=None,               # e.g. "bicycle" to reject disallowed edges
    ):
        # readNet loads the whole network once (expensive) so the caller
        # should build ONE matcher and reuse it for every fix.
        self.net = sumolib.net.readNet(net_file)
        self.search_radius = search_radius
        self.dist_half_width = dist_half_width
        self.angle_small_break = angle_small_break
        self.angle_large_break = angle_large_break
        self.junction_threshold = junction_threshold
        self.min_speed_for_switch = min_speed_for_switch
        self.confirm_count = confirm_count
        self.output_low = output_low
        self.output_average = output_average
        self.output_high = output_high
        self.vclass = vclass

        # The confirmed edge and how far along it (m from its start), plus
        # the pending-candidate bookkeeping for the temporal-consensus
        # check, and the last raw fix (heading fallback). All None until
        # the first fix of a route.
        self._current_edge = None
        self._current_offset = None
        self._pending_edge = None
        self._pending_count = 0
        self._prev_xy = None

    def reset(self):
        """Clear matching history. Call between separate routes."""
        self._current_edge = None
        self._current_offset = None
        self._pending_edge = None
        self._pending_count = 0
        self._prev_xy = None

    # -- candidate lookup -----------------------------------------------------

    def _nearby_edges(self, x, y):
        raw = self.net.getNeighboringEdges(x, y, self.search_radius)
        edges = []
        for edge, _dist in raw:
            if edge.getID().startswith(":"):
                continue   # skip internal junction edges
            if self.vclass is not None and not edge.allows(self.vclass):
                continue
            edges.append(edge)
        return edges

    def _successor_edges(self, edge):
        # Edges reachable from the downstream end of the current edge --
        # the candidate set for "entering" mode, per Ren & Karimi.
        return list(edge.getOutgoing().keys())

    # -- geometry ---------------------------------------------------------------

    def _project(self, edge, x, y):
        """Perpendicular distance, offset along the edge, and snapped (x, y)."""
        shape = edge.getShape()
        offset = gh.polygonOffsetWithMinimumDistanceToPoint((x, y), shape)
        sx, sy = gh.positionAtShapeOffset(shape, offset)
        return math.hypot(x - sx, y - sy), offset, sx, sy

    def _bearing_at(self, edge, offset):
        shape = edge.getShape()
        cum = 0.0
        for (ax, ay), (bx, by) in zip(shape[:-1], shape[1:]):
            seg_len = math.hypot(bx - ax, by - ay)
            if seg_len == 0.0 or cum + seg_len >= offset:
                return math.degrees(math.atan2(bx - ax, by - ay)) % 360.0
            cum += seg_len
        (ax, ay), (bx, by) = shape[-2], shape[-1]
        return math.degrees(math.atan2(bx - ax, by - ay)) % 360.0

    def _heading_diff(self, edge, offset, x, y, course_deg):
        if course_deg is None:
            # No phone heading: fall back to the bearing of the last two
            # raw fixes, the same idea as deriving heading from consecutive
            # GPS points when the receiver has no compass of its own.
            if self._prev_xy is None or math.hypot(x - self._prev_xy[0], y - self._prev_xy[1]) < 0.5:
                return 45.0   # no usable heading signal yet -- neutral
            course_deg = math.degrees(math.atan2(x - self._prev_xy[0], y - self._prev_xy[1])) % 360.0
        diff = abs(course_deg - self._bearing_at(edge, offset)) % 360.0
        return 360.0 - diff if diff > 180.0 else diff

    # -- fuzzy membership + rules -------------------------------------------------

    def _dist_short(self, d):
        # Decreasing sigmoid: ~1 at d=0, 0.5 at dist_half_width, ->0 beyond it.
        return 1.0 / (1.0 + math.exp((d - self.dist_half_width) / (self.dist_half_width / 4.0)))

    def _angle_small(self, diff):
        # Piecewise-linear membership (Ren & Karimi, Eq. 1).
        if diff < self.angle_small_break:
            return 1.0
        if diff < self.angle_large_break:
            span = self.angle_large_break - self.angle_small_break
            return 1.0 - (diff - self.angle_small_break) / span
        return 0.0

    def _score(self, edge, offset, dist, x, y, course_deg):
        angle_diff = self._heading_diff(edge, offset, x, y, course_deg)
        short = self._dist_short(dist)
        long_ = 1.0 - short
        small = self._angle_small(angle_diff)
        large = 1.0 - small

        # Four rules (Ren & Karimi, Table 2): min for AND, weighted
        # average of each rule's constant output for the crisp result.
        r_high = min(short, small)
        r_low = min(long_, large)
        r_avg = min(short, large) + min(long_, small)
        total = r_high + r_low + r_avg
        score = (
            0.0 if total == 0.0 else
            (r_high * self.output_high + r_low * self.output_low + r_avg * self.output_average) / total
        )
        return score, angle_diff, short, small

    # -- main entry point -----------------------------------------------------------

    def match(self, x, y, course_deg=None, speed_mps=None, accuracy_m=None):
        """
        Match one GPS fix (given in SUMO network x, y) to the best edge.

        accuracy_m is accepted but unused -- this method has no fuzzy
        input that consumes it.

        Returns a dict:
            {
              "x", "y":       snapped position to feed moveToXY (keepRoute=2)
              "edge_id":      chosen edge id
              "raw_dist":     perpendicular distance of the raw point to that edge
              "score":        crisp likelihood (0-100) of the chosen edge
              "components":   {"short", "small", "angle_diff", "mode"}
            }
        or None if no candidate edge lies within search_radius.
        """
        stationary = speed_mps is not None and speed_mps < self.min_speed_for_switch

        if self._current_edge is not None:
            dist, offset, sx, sy = self._project(self._current_edge, x, y)
            length = self._current_edge.getLength()
            in_bounds = 0.0 <= offset <= length
            clearance = min(offset, length - offset) if in_bounds else 0.0

            if stationary or (in_bounds and clearance > self.junction_threshold):
                # Following: still plausibly on the same edge.
                score, angle_diff, short, small = self._score(self._current_edge, offset, dist, x, y, course_deg)
                self._current_offset = offset
                self._pending_edge = None
                self._pending_count = 0
                self._prev_xy = (x, y)
                return {
                    "x": sx, "y": sy, "edge_id": self._current_edge.getID(),
                    "raw_dist": dist, "score": score,
                    "components": {"short": short, "small": small, "angle_diff": angle_diff, "mode": "following"},
                }

            # Entering: approaching or past the junction. Score the edges
            # connected downstream (falling back to a fresh area search if
            # the network offers none), plus the current edge itself, so a
            # borderline fix can't be forced off a still-plausible edge.
            candidates = self._successor_edges(self._current_edge) or self._nearby_edges(x, y)
            if self._current_edge not in candidates:
                candidates = candidates + [self._current_edge]
        else:
            candidates = self._nearby_edges(x, y)

        if not candidates:
            return None

        best = None
        for edge in candidates:
            e_dist, e_offset, e_sx, e_sy = self._project(edge, x, y)
            e_score, e_angle, e_short, e_small = self._score(edge, e_offset, e_dist, x, y, course_deg)
            if best is None or e_score > best[0]:
                best = (e_score, edge, e_dist, e_offset, e_sx, e_sy, e_angle, e_short, e_small)
        score, edge, dist, offset, sx, sy, angle_diff, short, small = best

        # Temporal consensus: a NEW edge must win on confirm_count
        # consecutive fixes before it replaces the current one, so one
        # noisy fix can't trigger a spurious junction switch.
        if self._current_edge is not None and edge is not self._current_edge and not stationary:
            if edge is self._pending_edge:
                self._pending_count += 1
            else:
                self._pending_edge = edge
                self._pending_count = 1
            if self._pending_count < self.confirm_count:
                cur_dist, cur_offset, cur_sx, cur_sy = self._project(self._current_edge, x, y)
                cur_score, cur_angle, cur_short, cur_small = self._score(
                    self._current_edge, cur_offset, cur_dist, x, y, course_deg
                )
                self._prev_xy = (x, y)
                return {
                    "x": cur_sx, "y": cur_sy, "edge_id": self._current_edge.getID(),
                    "raw_dist": cur_dist, "score": cur_score,
                    "components": {"short": cur_short, "small": cur_small, "angle_diff": cur_angle, "mode": "entering_pending"},
                }

        self._current_edge = edge
        self._current_offset = offset
        self._pending_edge = None
        self._pending_count = 0
        self._prev_xy = (x, y)
        return {
            "x": sx, "y": sy, "edge_id": edge.getID(),
            "raw_dist": dist, "score": score,
            "components": {"short": short, "small": small, "angle_diff": angle_diff, "mode": "confirmed"},
        }
