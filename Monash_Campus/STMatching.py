"""
st_matching.py
------------------------------------------------------------------
ST-Matching (spatial-temporal map matching) for the eBike-in-the-Loop
SUMO platform.

Implements the method of Lou et al. (2009) from the project paper:

    F(c_i -> c_j) = Fs(c_i, c_j) * Ft(c_i, c_j)

where Fs is a spatial score (observation probability x transmission
probability) and Ft is a temporal score (cosine similarity between the
speed limits along the connecting path and the implied travel speed).
A candidate graph is built over a window of consecutive GPS fixes and
Viterbi selects the highest-scoring path through it.

Like TopologicalMatcher this class is STATEFUL, but it holds more than
one point of history: it keeps a sliding WINDOW of the last N fixes,
because Viterbi needs a sequence to work on. Feed it one point per SUMO
step in the live loop. Call reset() between separate routes.

Input coordinates are SUMO *network* (x, y), same as the topological
matcher -- from traci.simulation.convertGeo(lon, lat, fromGeo=True).

Returns the same dict shape as TopologicalMatcher.match() so the two are
drop-in interchangeable in live_phone_to_sumo.py, with the score
components being ("obs", "trans", "fs", "ft") instead of the four
topological terms.

NOTE on moveToXY: as with the topological matcher, this class decides
the final on-edge position, so call moveToXY with keepRoute=2. With
keepRoute=0 SUMO re-snaps geometrically and discards the decision.

==================================================================
WHY THIS METHOD IS A POOR FIT FOR THIS PLATFORM
------------------------------------------------------------------
Read this before interpreting any CMP number this file produces. None
of the following are implementation bugs -- they are consequences of
using a method outside the regime it was designed for, and they are the
substance of the discussion section.

1. IT IS DESIGNED FOR LOW SAMPLING RATES.
   Lou et al. target trajectories sampled every 1-5 MINUTES, where
   consecutive fixes are hundreds of metres apart and the route between
   them is genuinely ambiguous. This platform streams at ~1 Hz. At 1 Hz
   consecutive fixes are typically 2-6 m apart, usually on the same
   edge, so the transmission probability V = d(p_i-1, p_i) / w(c_i-1, c_i)
   sits near 1.0 for almost every candidate pair. The term that carries
   most of ST-Matching's discriminating power is therefore close to
   constant here, and the spatial score collapses towards plain
   proximity -- i.e. towards the geometric baseline.

2. THE TEMPORAL TERM CANCELS UNDER THE ORIGINAL FORMULATION.
   In Lou's definition the average speed v' is a single scalar for the
   whole path (path length / elapsed time), repeated once per segment
   inside the cosine similarity. Because it is constant across the
   summation it cancels top and bottom:

       Ft = sum(u_k * v') / ( sqrt(sum(u_k^2)) * sqrt(k * v'^2) )
          = sum(u_k)     / ( sqrt(sum(u_k^2)) * sqrt(k) )

   The observed speed disappears. What is left measures only how
   UNIFORM the speed limits along the path are, not whether the rider's
   speed is plausible for those roads. temporal_mode="lou" reproduces
   this faithfully (use it for the headline comparison);
   temporal_mode="ratio" is a corrected variant that actually consumes
   the observed speed. Reporting both is the honest thing to do.

3. THE SPEED LIMITS IT SCORES AGAINST ARE CAR SPEED LIMITS.
   Even with the corrected temporal term, the Clayton campus network
   carries 40-50 km/h limits while an eBike travels at 15-25 km/h. Every
   candidate is implausibly slow by roughly the same factor, so Ft again
   fails to separate candidates. speed_reference lets you rescale
   against a bike-plausible speed; whether that rescues the term is an
   empirical question worth answering in the report.

4. VITERBI IS NOT REAL-TIME.
   ST-Matching's selling point is a GLOBALLY optimal path over the whole
   trajectory. A live digital twin cannot wait for the trajectory to
   finish. This implementation uses a fixed-lag window (window_size
   fixes) and emits the decision for the CURRENT point each step, so
   only backward context is used -- no lookahead. The global optimality
   guarantee does not survive that, which removes the method's main
   theoretical advantage. Increasing window_size does not fix this; it
   only adds history, not future. The alternative -- delaying emission
   by L steps to get lookahead -- makes the twin lag the real rider by
   L seconds, which defeats the purpose of the platform.

5. IT IGNORES HEADING.
   Lou's formulation has no heading term, because at 1-5 minute
   sampling the phone's course is meaningless. At 1 Hz heading is one
   of the most informative signals available, and it is what lets the
   topological matcher separate the two directional edges of a two-way
   street. ST-Matching discards it by construction.

6. COST GROWS WITH CANDIDATES.
   Each step needs a shortest-path query per candidate PAIR, i.e.
   O(C^2) Dijkstra calls per transition. Results are cached
   (_path_cache) and candidates are pruned (max_candidates), but this
   is still by far the most expensive of the five methods and should be
   included in the latency measurements.
==================================================================
"""

import math
from collections import deque

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
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def _cosine_similarity(u_vec, v_vec):
    """
    Cosine similarity between two equal-length non-negative vectors.
    Returns 0.0 if either vector has zero magnitude.
    """
    # Both inputs here are speeds (non-negative), so the result lands in
    # [0, 1] and can be used directly as a multiplicative score.
    dot = sum(u * v for u, v in zip(u_vec, v_vec))
    mag_u = math.sqrt(sum(u * u for u in u_vec))
    mag_v = math.sqrt(sum(v * v for v in v_vec))
    if mag_u == 0.0 or mag_v == 0.0:
        return 0.0
    return dot / (mag_u * mag_v)


def _safe_log(value, floor=1e-12):
    """log() with a floor, so a zero-probability transition doesn't explode."""
    # Viterbi is run in log space to avoid underflow when multiplying many
    # small probabilities together over a window.
    return math.log(max(value, floor))


# ---- the matcher -----------------------------------------------------------

class STMatcher:
    def __init__(
        self,
        net_file,
        search_radius=50.0,        # metres; hard cutoff for candidate lookup
        sigma=20.0,                # metres; GPS error std for observation prob
        window_size=8,             # fixes held for the Viterbi window
        max_candidates=5,          # per fix; caps the O(C^2) transition cost
        temporal_mode="lou",       # "lou" (faithful) | "ratio" (corrected) | "off"
        speed_reference=None,      # m/s; overrides edge speed limits if set
        nominal_dt=1.0,            # s; fallback when timestamps are unavailable
        vclass=None,               # e.g. "bicycle"; None = off
    ):
        # readNet loads the whole network once (expensive) so the caller
        # should build ONE matcher and reuse it for every fix.
        self.net = sumolib.net.readNet(net_file)
        self.search_radius = search_radius
        self.sigma = sigma
        self.window_size = window_size
        self.max_candidates = max_candidates
        self.temporal_mode = temporal_mode
        self.speed_reference = speed_reference
        self.nominal_dt = nominal_dt
        self.vclass = vclass

        # The window of recent fixes. Each entry is a dict holding the raw
        # point, its timestamp, and its scored candidate list. Viterbi is
        # re-run over this whole window on every new fix.
        self.window = deque(maxlen=window_size)

        # Shortest-path results keyed by (from_edge_id, to_edge_id). The
        # network never changes, so this cache survives reset() and is what
        # keeps the O(C^2) Dijkstra cost tolerable in the live loop.
        self._path_cache = {}

    def reset(self):
        """Clear matching history. Call between separate routes."""
        # Only the window is route-specific; the path cache is network
        # geometry and stays valid across routes.
        self.window.clear()

    # -- candidate lookup ----------------------------------------------------

    def _candidates(self, x, y):
        """
        Nearby edges as candidate states, each snapped onto its centreline.

        Returns a list of dicts: edge, dist (perpendicular), offset (distance
        along the edge shape), and the snapped (x, y).
        """
        raw = self.net.getNeighboringEdges(x, y, self.search_radius)
        cands = []
        for edge, dist in raw:
            if edge.getID().startswith(":"):
                continue   # skip internal junction edges
            # Optionally drop edges this vehicle class isn't allowed on.
            if self.vclass is not None and not edge.allows(self.vclass):
                continue
            shape = edge.getShape()
            # Offset = how far along the edge the nearest point sits. Needed
            # for the network-distance calculation, not just for snapping.
            offset = gh.polygonOffsetWithMinimumDistanceToPoint((x, y), shape)
            sx, sy = gh.positionAtShapeOffset(shape, offset)
            cands.append({
                "edge": edge,
                "dist": dist,
                "offset": offset,
                "x": sx,
                "y": sy,
                # Observation probability N(c): Gaussian on perpendicular
                # GPS-to-edge distance. The 1/(sqrt(2pi)*sigma) constant is
                # identical for every candidate so it cannot change the
                # ranking, but it is kept for fidelity to the paper.
                "obs": (1.0 / (math.sqrt(2.0 * math.pi) * self.sigma))
                       * math.exp(-(dist * dist) / (2.0 * self.sigma ** 2)),
            })

        # Keep only the closest max_candidates. Transition scoring is
        # quadratic in this number, so it is the main latency lever.
        cands.sort(key=lambda c: c["dist"])
        return cands[: self.max_candidates]

    # -- network distance ----------------------------------------------------

    def _edge_path(self, from_edge, to_edge):
        """
        Shortest edge path from_edge -> to_edge, cached.

        Returns (path_edges, intermediate_length) or (None, None) if the
        target is unreachable. intermediate_length is the summed length of
        the edges strictly BETWEEN the two, so partial offsets can be added
        by the caller.
        """
        key = (from_edge.getID(), to_edge.getID())
        if key in self._path_cache:
            return self._path_cache[key]

        path = None
        try:
            # vClass keeps the route legal for a bicycle where the network
            # declares access restrictions.
            if self.vclass is not None:
                path, _cost = self.net.getShortestPath(
                    from_edge, to_edge, vClass=self.vclass
                )
            else:
                path, _cost = self.net.getShortestPath(from_edge, to_edge)
        except TypeError:
            # Older sumolib builds don't accept vClass here.
            path, _cost = self.net.getShortestPath(from_edge, to_edge)
        except Exception:
            path = None

        if not path:
            result = (None, None)
        else:
            intermediate = sum(e.getLength() for e in path[1:-1])
            result = (list(path), intermediate)

        self._path_cache[key] = result
        return result

    def _network_distance(self, cand_a, cand_b):
        """
        Travel distance along the network from candidate a to candidate b,
        accounting for where on each edge the two points actually sit.

        Returns (distance_m, path_edges) or (None, None) if unreachable.
        """
        edge_a, edge_b = cand_a["edge"], cand_b["edge"]

        # Same edge: the distance is just the difference in offsets. A
        # negative difference means the point moved backwards along the
        # edge, which at 1 Hz is almost always GPS jitter rather than a real
        # reversal, so take the magnitude rather than rejecting the pair.
        if edge_a.getID() == edge_b.getID():
            return abs(cand_b["offset"] - cand_a["offset"]), [edge_a]

        path, intermediate = self._edge_path(edge_a, edge_b)
        if path is None:
            return None, None

        # Remaining length on the first edge + whole edges in between +
        # distance into the last edge.
        dist = (edge_a.getLength() - cand_a["offset"]) + intermediate + cand_b["offset"]
        return max(dist, 0.0), path

    # -- the two score functions ---------------------------------------------

    def _spatial_score(self, cand_a, cand_b, straight_dist, net_dist):
        """
        Fs = N(c_b) * V(c_a -> c_b).

        N is the observation probability (already on the candidate) and V is
        the transmission probability: the ratio of the straight-line distance
        between the two GPS fixes to the network distance between the two
        candidates. A detour-free match scores near 1; a match that would
        require a long loop scores low.
        """
        if net_dist is None:
            return 0.0            # unreachable -> impossible transition
        if net_dist <= 0.0:
            # Both fixes snapped to the same on-edge point (stationary rider).
            # No detour is possible, so the transmission term is perfect.
            trans = 1.0
        else:
            trans = straight_dist / net_dist
        # Snapping can make the network path shorter than the straight line;
        # clamp so V stays a probability rather than rewarding that artefact.
        trans = min(trans, 1.0)
        return cand_b["obs"] * trans, trans

    def _temporal_score(self, path_edges, net_dist, dt, observed_speed):
        """
        Ft: cosine similarity between the speed limits along the connecting
        path and the speed the transition implies.

        See point 2 of the header comment -- under temporal_mode="lou" the
        implied speed cancels out of this expression entirely and Ft reduces
        to a measure of speed-limit uniformity along the path. That is
        faithful to the published method, not a bug.
        """
        if self.temporal_mode == "off" or not path_edges:
            return 1.0            # neutral: F reduces to Fs alone

        # u_k: the speed constraint of each edge on the path.
        limits = [e.getSpeed() for e in path_edges]
        if self.speed_reference is not None:
            # Rescale car-oriented limits onto a bike-plausible band while
            # preserving their relative differences (point 3 of the header).
            top = max(limits) if limits else 0.0
            if top > 0.0:
                limits = [self.speed_reference * (u / top) for u in limits]
        if not limits or max(limits) <= 0.0:
            return 1.0

        # v': the average speed implied by this transition.
        dt = dt if dt and dt > 0.0 else self.nominal_dt
        implied_speed = (net_dist / dt) if net_dist is not None else 0.0

        if self.temporal_mode == "ratio":
            # Corrected variant: compare the implied speed against each
            # edge's limit directly, so the observed motion actually
            # influences the score. Uses the measured speed when the phone
            # supplies one, falling back to the implied speed.
            v_meas = observed_speed if observed_speed is not None else implied_speed
            per_edge = [min(v_meas / u, u / v_meas) if v_meas > 0.0 and u > 0.0 else 0.0
                        for u in limits]
            return sum(per_edge) / len(per_edge)

        # Faithful Lou formulation: v' repeated once per path segment.
        speeds = [implied_speed] * len(limits)
        return _cosine_similarity(limits, speeds)

    # -- Viterbi over the window ---------------------------------------------

    def _run_viterbi(self):
        """
        Find the highest-scoring candidate sequence across the current
        window. Runs in log space; returns (best_last_index, backpointers)
        or (None, None) if the window has no scorable candidates.

        Note this re-runs from scratch on every new fix. The window is small
        (window_size fixes x max_candidates states) so this is cheap given
        the shortest-path cache, and it keeps the code readable. It also
        means earlier points may be re-decided as new evidence arrives --
        harmless for logging, but the bike has already been moved, so those
        revisions are never applied. See point 4 of the header comment.
        """
        frames = list(self.window)
        if not frames or not frames[0]["cands"]:
            return None, None

        # Initialise with the observation probability of the oldest fix in
        # the window. ST-Matching has no separate prior over start states.
        scores = [_safe_log(c["obs"]) for c in frames[0]["cands"]]
        backpointers = []
        detail = []   # per-step component values for the winning transitions

        for i in range(1, len(frames)):
            prev_frame, curr_frame = frames[i - 1], frames[i]
            prev_cands, curr_cands = prev_frame["cands"], curr_frame["cands"]

            if not curr_cands:
                # No candidates at this fix -- the chain cannot continue.
                return None, None

            # Straight-line distance between the two RAW GPS fixes, d(p_i-1, p_i).
            straight = math.hypot(
                curr_frame["x"] - prev_frame["x"],
                curr_frame["y"] - prev_frame["y"],
            )
            dt = curr_frame["t"] - prev_frame["t"]

            new_scores = [float("-inf")] * len(curr_cands)
            new_back = [None] * len(curr_cands)
            new_detail = [None] * len(curr_cands)

            for j, cand_b in enumerate(curr_cands):
                for k, cand_a in enumerate(prev_cands):
                    if scores[k] == float("-inf"):
                        continue

                    net_dist, path_edges = self._network_distance(cand_a, cand_b)
                    fs, trans = self._spatial_score(cand_a, cand_b, straight, net_dist)
                    if fs <= 0.0:
                        continue
                    ft = self._temporal_score(
                        path_edges, net_dist, dt, curr_frame["speed"]
                    )

                    # F = Fs * Ft, accumulated as a sum of logs.
                    total = scores[k] + _safe_log(fs) + _safe_log(ft)
                    if total > new_scores[j]:
                        new_scores[j] = total
                        new_back[j] = k
                        new_detail[j] = {
                            "obs": cand_b["obs"],
                            "trans": trans,
                            "fs": fs,
                            "ft": ft,
                        }

            if all(s == float("-inf") for s in new_scores):
                # Every transition was impossible (typically a GPS jump to a
                # disconnected part of the network). Restart the chain from
                # this fix using observation probability alone, so the
                # matcher recovers instead of stalling.
                new_scores = [_safe_log(c["obs"]) for c in curr_cands]
                new_back = [None] * len(curr_cands)
                new_detail = [{"obs": c["obs"], "trans": None, "fs": c["obs"], "ft": None}
                              for c in curr_cands]

            scores = new_scores
            backpointers.append(new_back)
            detail.append(new_detail)

        best_index = max(range(len(scores)), key=lambda i: scores[i])
        return best_index, (scores, backpointers, detail)

    # -- main entry point ----------------------------------------------------

    def match(self, x, y, timestamp=None, speed_mps=None, course_deg=None):
        """
        Match one GPS fix (given in SUMO network x, y) to the best edge.

        course_deg is accepted only so this signature stays interchangeable
        with TopologicalMatcher.match(); ST-Matching does not use heading
        (point 5 of the header comment).

        Returns a dict:
            {
              "x", "y":       snapped position to feed moveToXY (keepRoute=2)
              "edge_id":      chosen edge id
              "raw_dist":     perpendicular distance of the raw point to that edge
              "score":        winning log-score of the path through the window
              "components":   {"obs","trans","fs","ft"}
              "window_len":   how many fixes the decision was based on
            }
        or None if no candidate edge lies within search_radius.
        """
        # 1. Gather nearby edges. No candidates -> nothing to match.
        cands = self._candidates(x, y)
        if not cands:
            return None

        # 2. Push this fix onto the sliding window. deque(maxlen) drops the
        #    oldest automatically, giving the fixed-lag behaviour.
        t = timestamp if timestamp is not None else (
            self.window[-1]["t"] + self.nominal_dt if self.window else 0.0
        )
        self.window.append({
            "x": x, "y": y, "t": t, "speed": speed_mps, "cands": cands,
        })

        # 3. Single fix in the window (first point of a route): no transition
        #    exists yet, so fall back to the observation probability alone.
        if len(self.window) == 1:
            best = cands[0]   # candidates are distance-sorted, so [0] maximises N
            return {
                "x": best["x"],
                "y": best["y"],
                "edge_id": best["edge"].getID(),
                "raw_dist": best["dist"],
                "score": _safe_log(best["obs"]),
                "components": {"obs": best["obs"], "trans": None,
                               "fs": best["obs"], "ft": None},
                "window_len": 1,
            }

        # 4. Run Viterbi over the window and take the state for the NEWEST
        #    fix -- that is the point the bike has to be moved to now.
        best_index, viterbi = self._run_viterbi()
        if best_index is None:
            # Chain broken; fall back to nearest candidate for this fix.
            best = cands[0]
            return {
                "x": best["x"],
                "y": best["y"],
                "edge_id": best["edge"].getID(),
                "raw_dist": best["dist"],
                "score": _safe_log(best["obs"]),
                "components": {"obs": best["obs"], "trans": None,
                               "fs": best["obs"], "ft": None},
                "window_len": len(self.window),
            }

        scores, _backpointers, detail = viterbi
        chosen = cands[best_index]
        # Component values for the transition that won at this fix. Log these
        # per step -- the distribution of trans and ft across candidates is
        # the direct evidence for points 1-3 of the header comment.
        comps = detail[-1][best_index] if detail and detail[-1][best_index] else {
            "obs": chosen["obs"], "trans": None, "fs": chosen["obs"], "ft": None
        }

        return {
            "x": chosen["x"],
            "y": chosen["y"],
            "edge_id": chosen["edge"].getID(),
            "raw_dist": chosen["dist"],
            "score": scores[best_index],
            "components": comps,
            "window_len": len(self.window),
        }