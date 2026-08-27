"""
topological.py
------------------------------------------------------------------
Topological map-matching for the eBike-in-the-Loop SUMO platform.

Implements the weighted-sum topological method from the project paper:

    S(e) = w1*prox(e) + w2*head(e) + w3*conn(e) + w4*turn(e)

and selects the highest-scoring candidate edge. Unlike pure geometric
matching, connectivity (reachability from the previously matched edge)
and a turn term let it reject implausible jumps to parallel/crossing
roads at junctions.

The matcher is STATEFUL: it remembers the previously matched edge so the
connectivity/turn terms have something to reason about. Feed it one point
per SUMO step in the live loop. Call reset() between separate routes.

Input coordinates are SUMO *network* (x, y). In the live loop you already
have these from traci.simulation.convertGeo(lon, lat, fromGeo=True).

Returns a dict with the snapped (x, y) to hand to moveToXY(), plus the
chosen edge id and the individual score components (useful for tuning and
for the CMP write-up).

NOTE on moveToXY: this matcher already decides the final on-edge position,
so call moveToXY with keepRoute=2 to place the bike exactly there. With
keepRoute=0 SUMO would re-snap to the geometrically nearest edge and
override the topological decision at exactly the junctions you care about.

------------------------------------------------------------------
HOW THE FOUR TERMS WORK, IN ONE PLACE
  prox : how close the raw GPS point is to the edge          (geometry)
  head : how well the phone's heading matches the edge's     (direction)
         direction of travel -- also separates the two
         one-way edges of a two-way street
  conn : whether this edge is reachable from the edge we      (topology,
         matched last time -- the term that kills teleports    history)
         to parallel roads
  turn : whether the move from the previous edge to this one   (legality)
         is a legal manoeuvre rather than a forbidden U-turn
All four are normalised so that "higher is better", then combined by the
weighted sum S(e). The edge with the largest S(e) wins.
------------------------------------------------------------------
"""

import math
import sumolib
from sumolib import geomhelper as gh


# ---- small geometry helpers ------------------------------------------------

def _point_segment_distance(px, py, ax, ay, bx, by):
    """Perpendicular distance from point (px,py) to segment a->b."""
    # Vector along the segment.
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    # Degenerate segment (a == b): fall back to point-to-point distance.
    if seg_len_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    # t = projection of the point onto the (infinite) line, as a fraction of
    # the segment length. Clamping to [0, 1] keeps the closest point on the
    # actual segment rather than its extension.
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    # (cx, cy) is that closest point; return its distance to (px, py).
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def _nearest_segment_bearing(shape, x, y):
    """
    Navigational bearing (deg, 0=N, clockwise) of the shape segment
    closest to (x, y). Edge shapes are ordered from-node -> to-node, so
    this is the direction of travel along the edge.
    """
    # An edge shape is a polyline (list of vertices); walk each segment of
    # it and keep the bearing of whichever segment the point sits nearest.
    best_d = float("inf")
    best_bearing = None
    for (ax, ay), (bx, by) in zip(shape[:-1], shape[1:]):
        d = _point_segment_distance(x, y, ax, ay, bx, by)
        if d < best_d:
            best_d = d
            # SUMO net coords: +x = east, +y = north.
            # Bearing clockwise from north = atan2(east, north) = atan2(dx, dy).
            best_bearing = math.degrees(math.atan2(bx - ax, by - ay)) % 360.0
    return best_bearing


def _angular_diff(a, b):
    """Smallest absolute difference between two bearings, in [0, 180]."""
    # Wrap into [0, 360), then fold anything over 180 back down so that,
    # e.g., 350 vs 10 degrees reads as 20, not 340.
    d = abs(a - b) % 360.0
    return 360.0 - d if d > 180.0 else d


# ---- the matcher -----------------------------------------------------------

class TopologicalMatcher:
    def __init__(
        self,
        net_file,
        search_radius=50.0,      # metres; hard cutoff for candidate lookup
        sigma_prox=15.0,         # metres; proximity score decay scale
        weights=(0.40, 0.30, 0.20, 0.10),   # (prox, head, conn, turn)
        min_speed_for_heading=0.5,          # m/s; below this, heading is ignored
        vclass=None,             # e.g. "bicycle" to reject disallowed edges; None = off
    ):
        # readNet loads the whole network once (expensive) so the caller
        # should build ONE matcher and reuse it for every fix.
        # withInternal defaults to False, so internal ":" edges are excluded,
        # matching the real-road granularity used in the CMP evaluation.
        self.net = sumolib.net.readNet(net_file)
        self.search_radius = search_radius
        self.sigma_prox = sigma_prox
        # Unpack the 4-tuple into named weights for readability in match().
        self.w_prox, self.w_head, self.w_conn, self.w_turn = weights
        self.min_speed_for_heading = min_speed_for_heading
        self.vclass = vclass
        # The single piece of state: the edge chosen on the previous fix.
        # Drives the connectivity and turn terms. None until the first match.
        self.prev_edge = None

    def reset(self):
        """Clear matching history. Call between separate routes."""
        # Without this, the first fix of a new route would be scored as if it
        # had to connect to the last edge of the previous route.
        self.prev_edge = None

    # -- individual score terms ---------------------------------------------

    def _proximity_score(self, dist):
        # Gaussian on perpendicular GPS-to-edge distance -> (0, 1], 1 = on the edge.
        # sigma_prox sets how fast the score falls off with distance: larger
        # sigma is more forgiving of noisy GPS, smaller is stricter.
        return math.exp(-(dist * dist) / (2.0 * self.sigma_prox ** 2))

    def _heading_score(self, edge, x, y, course_deg, speed_mps):
        # Heading is unreliable when nearly stationary. Also, the phone client
        # sends course_deg = 0 (not null) when the browser has no heading, which
        # would otherwise masquerade as "due north" -- so guard on speed.
        # In both unusable cases we return 0.5 (neutral), so heading neither
        # helps nor hurts and the decision falls to the other three terms.
        if course_deg is None or course_deg < 0:
            return 0.5
        if speed_mps is not None and speed_mps < self.min_speed_for_heading:
            return 0.5
        bearing = _nearest_segment_bearing(edge.getShape(), x, y)
        if bearing is None:
            return 0.5
        diff = _angular_diff(course_deg, bearing)   # [0, 180]
        # cos maps 0deg->1 (aligned), 90deg->0.5, 180deg->0 (opposite direction).
        # This also disambiguates the two directional edges of a two-way street.
        return (math.cos(math.radians(diff)) + 1.0) / 2.0

    def _connectivity_score(self, edge):
        # Rewards edges that are actually reachable from where we were last,
        # and punishes edges that aren't -- this is the term that stops the
        # bike teleporting onto a parallel/crossing road at a junction.
        if self.prev_edge is None:
            return 0.5   # neutral on the first fix -- no history yet
        if edge.getID() == self.prev_edge.getID():
            return 1.0   # still on the same edge (the common case at 1 Hz)
        outgoing = self.prev_edge.getOutgoing()   # dict {Edge: [Connection, ...]}
        if edge in outgoing:
            return 0.9   # directly downstream
        # Two-hop check: bridges the case where a short internal junction edge
        # sits between prev_edge and this candidate on the real network.
        for succ in outgoing:
            if edge in succ.getOutgoing():
                return 0.5   # two hops away (bridges a short internal junction)
        return 0.1       # not reachable -> implausible jump

    def _turn_score(self, edge):
        # Turn *compliance* in [0, 1] (1 = fine, 0 = forbidden U-turn), so it
        # slots into the weighted sum as "higher is better" like the others.
        if self.prev_edge is None or edge.getID() == self.prev_edge.getID():
            return 1.0   # first fix or staying put -- no manoeuvre to judge
        conns = self.prev_edge.getOutgoing().get(edge, [])
        if not conns:
            return 0.5   # no explicit connection; may still route via a junction
        # Connection.getDirection() == 't' is a turnaround / U-turn.
        # If EVERY connection between the two edges is a U-turn, penalise fully.
        if all(c.getDirection() == "t" for c in conns):
            return 0.0
        return 1.0

    # -- snapping ------------------------------------------------------------

    def _snap(self, edge, x, y):
        # Project the raw (x, y) onto the chosen edge's centreline and return
        # that on-edge point -- this is what gets fed to moveToXY(keepRoute=2).
        shape = edge.getShape()
        # Offset (distance along the shape) of the nearest point to (x, y)...
        offset = gh.polygonOffsetWithMinimumDistanceToPoint((x, y), shape)
        # ...converted back into an actual (x, y) position on the shape.
        return gh.positionAtShapeOffset(shape, offset)

    # -- candidate lookup ----------------------------------------------------

    def _candidates(self, x, y):
        
        # Ask sumolib for every edge whose geometry passes within
        # search_radius of the point, as (edge, distance) pairs. The radius
        # is a HARD cutoff, so it must be sized to GPS noise, not tiny.
        raw = self.net.getNeighboringEdges(x, y, self.search_radius)
        cands = []
        for edge, dist in raw:
            if edge.getID().startswith(":"):
                continue   # skip internal junction edges
            # Optionally drop edges this vehicle class isn't allowed on
            # (e.g. reject motorway edges for a "bicycle").
            if self.vclass is not None and not edge.allows(self.vclass):
                continue
            cands.append((edge, dist))
        return cands

    # -- main entry point ----------------------------------------------------

    def match(self, x, y, course_deg=None, speed_mps=None):
        """
        Match one GPS fix (given in SUMO network x, y) to the best edge.

        Returns a dict:
            {
              "x", "y":       snapped position to feed moveToXY (keepRoute=2)
              "edge_id":      chosen edge id
              "raw_dist":     perpendicular distance of the raw point to that edge
              "score":        winning S(e)
              "components":   {"prox","head","conn","turn"}
            }
        or None if no candidate edge lies within search_radius (caller can
        then hold the previous position or fall back to native matching).
        """
        # 1. Gather nearby edges. No candidates -> nothing to match.
        cands = self._candidates(x, y)
        if not cands:
            return None

        # 2. Score every candidate with S(e) and keep the best.
        best = None
        best_score = float("-inf")
        for edge, dist in cands:
            prox = self._proximity_score(dist)
            head = self._heading_score(edge, x, y, course_deg, speed_mps)
            conn = self._connectivity_score(edge)
            turn = self._turn_score(edge)
            # The paper's weighted sum: S(e) = w1*prox + w2*head + w3*conn + w4*turn.
            score = (
                self.w_prox * prox
                + self.w_head * head
                + self.w_conn * conn
                + self.w_turn * turn
            )
            if score > best_score:
                best_score = score
                # Stash the winning edge plus its component scores so we can
                # report them (handy for tuning weights and for the write-up).
                best = (edge, dist, prox, head, conn, turn)

        # 3. Snap the raw point onto the winning edge and commit it to history.
        edge, dist, prox, head, conn, turn = best
        sx, sy = self._snap(edge, x, y)
        self.prev_edge = edge   # update history for the next fix

        # 4. Hand back the on-edge position plus diagnostics.
        return {
            "x": sx,
            "y": sy,
            "edge_id": edge.getID(),
            "raw_dist": dist,
            "score": best_score,
            "components": {"prox": prox, "head": head, "conn": conn, "turn": turn},
        }