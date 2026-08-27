"""
HMM.py
------------------------------------------------------------------
Hidden Markov Model map-matching, based on Newson & Krumm (2009),
"Hidden Markov Map Matching Through Noise and Sparseness".

Candidate road edges near each GPS fix are treated as hidden states.
Each is scored by an emission probability (Gaussian on perpendicular
GPS-to-edge distance) and a transition probability (exponential decay
on the difference between straight-line GPS displacement and network
shortest-path distance); Viterbi picks the highest-scoring sequence.

Runs as an online Viterbi rather than a batch one: each fix only needs
the previous fix's candidates and their accumulated path scores, so the
matched position is emitted immediately, with no lag, and an earlier
guess is quietly out-scored rather than revised once later points cast
doubt on it.

Call match() once per GPS fix (SUMO network x, y, plus optional course,
speed, and GPS accuracy). Call reset() between separate routes.

match() returns a dict {x, y, edge_id, raw_dist, score, components}, or
None if no candidate edge lies within search_radius -- the caller should
fall back to SUMO's native matching for that one fix.
------------------------------------------------------------------
"""

import math

import sumolib
from sumolib import geomhelper as gh


class HMMMatcher:
    def __init__(
        self,
        net_file,
        search_radius=50.0,      # metres; hard cutoff for candidate lookup
        sigma_default=4.07,      # metres; Newson & Krumm's GPS error std,
                                  # used when the phone reports no accuracy
        beta=0.2,                # detour-tolerance scale for the transition term
        max_candidates=5,        # per fix; transition cost is quadratic in this
        use_accuracy=True,       # use the phone's per-fix accuracy_m as sigma
                                  # when available, instead of sigma_default
        min_sigma=1.0,           # metres; floor so a near-zero accuracy
                                  # reading can't collapse emission to a spike
        vclass=None,              # e.g. "bicycle" to reject disallowed edges
    ):
        # readNet loads the whole network once (expensive) so the caller
        # should build ONE matcher and reuse it for every fix.
        self.net = sumolib.net.readNet(net_file)
        self.search_radius = search_radius
        self.sigma_default = sigma_default
        self.beta = beta
        self.max_candidates = max_candidates
        self.use_accuracy = use_accuracy
        self.min_sigma = min_sigma
        self.vclass = vclass

        # State carried from the previous fix: its candidates (each snapped
        # onto its edge), the raw (x, y) it was observed at, and each
        # candidate's best accumulated log-probability (delta) of any path
        # ending there. None until the first fix of a route.
        self._prev_cands = None
        self._prev_xy = None
        self._prev_delta = None

        # Shortest-path results keyed by (from_edge_id, to_edge_id). The
        # network never changes, so this cache survives reset().
        self._path_cache = {}

    def reset(self):
        """Clear matching history. Call between separate routes."""
        self._prev_cands = None
        self._prev_xy = None
        self._prev_delta = None

    # -- candidate lookup ----------------------------------------------------

    def _candidates(self, x, y):
        """
        Nearby edges as candidate states, each snapped onto its centreline.

        Returns a list of dicts: edge, dist (perpendicular), offset (distance
        along the edge shape), and the snapped (x, y). Sorted by distance and
        capped at max_candidates, since the transition step is quadratic in
        the candidate count.
        """
        raw = self.net.getNeighboringEdges(x, y, self.search_radius)
        cands = []
        for edge, dist in raw:
            if edge.getID().startswith(":"):
                continue   # skip internal junction edges
            if self.vclass is not None and not edge.allows(self.vclass):
                continue
            shape = edge.getShape()
            offset = gh.polygonOffsetWithMinimumDistanceToPoint((x, y), shape)
            sx, sy = gh.positionAtShapeOffset(shape, offset)
            cands.append({"edge": edge, "dist": dist, "offset": offset, "x": sx, "y": sy})

        cands.sort(key=lambda c: c["dist"])
        return cands[: self.max_candidates]

    # -- network distance -----------------------------------------------------

    def _edge_path_intermediate_length(self, from_edge, to_edge):
        """
        Summed length of the edges strictly BETWEEN from_edge and to_edge on
        the shortest path, cached. Returns None if to_edge is unreachable.
        """
        key = (from_edge.getID(), to_edge.getID())
        if key in self._path_cache:
            return self._path_cache[key]

        path = None
        try:
            if self.vclass is not None:
                path, _cost = self.net.getShortestPath(from_edge, to_edge, vClass=self.vclass)
            else:
                path, _cost = self.net.getShortestPath(from_edge, to_edge)
        except TypeError:
            # Older sumolib builds don't accept vClass here.
            path, _cost = self.net.getShortestPath(from_edge, to_edge)
        except Exception:
            path = None

        result = None if not path else sum(e.getLength() for e in path[1:-1])
        self._path_cache[key] = result
        return result

    def _network_distance(self, cand_a, cand_b):
        """
        Travel distance along the network from candidate a to candidate b,
        accounting for where on each edge the two points actually sit.
        Returns None if b is unreachable from a.
        """
        edge_a, edge_b = cand_a["edge"], cand_b["edge"]

        if edge_a.getID() == edge_b.getID():
            # Same edge: distance is just the difference in offsets. Use the
            # magnitude -- at ~1 Hz a negative difference is almost always
            # GPS jitter, not a real reversal.
            return abs(cand_b["offset"] - cand_a["offset"])

        intermediate = self._edge_path_intermediate_length(edge_a, edge_b)
        if intermediate is None:
            return None

        dist = (edge_a.getLength() - cand_a["offset"]) + intermediate + cand_b["offset"]
        return max(dist, 0.0)

    # -- probability terms (log space) ------------------------------------------

    def _sigma_for(self, accuracy_m):
        # Per-point sigma from the phone's own GPS accuracy when available;
        # falls back to Newson & Krumm's fixed constant otherwise.
        if self.use_accuracy and accuracy_m is not None and accuracy_m > 0:
            return max(accuracy_m, self.min_sigma)
        return self.sigma_default

    def _log_emission(self, dist, sigma):
        # log[ 1/(sqrt(2*pi)*sigma) * exp(-dist^2 / (2*sigma^2)) ]
        return -math.log(math.sqrt(2.0 * math.pi) * sigma) - (dist * dist) / (2.0 * sigma * sigma)

    def _log_transition(self, straight_dist, net_dist):
        # log[ 1/beta * exp(-|straight - net| / beta) ], or None if the pair
        # is unreachable (probability 0).
        if net_dist is None:
            return None
        return -math.log(self.beta) - abs(straight_dist - net_dist) / self.beta

    # -- main entry point ------------------------------------------------------

    def match(self, x, y, course_deg=None, speed_mps=None, accuracy_m=None):
        """
        Match one GPS fix (given in SUMO network x, y) to the best edge.

        course_deg and speed_mps are accepted but unused -- Newson & Krumm's
        formulation uses neither.

        Returns a dict:
            {
              "x", "y":       snapped position to feed moveToXY (keepRoute=2)
              "edge_id":      chosen edge id
              "raw_dist":     perpendicular distance of the raw point to that edge
              "score":        winning candidate's accumulated log-probability
              "components":   {"emission", "transition", "sigma", "num_candidates"}
            }
        or None if no candidate edge lies within search_radius (caller can
        then fall back to native matching for this one fix).
        """
        # 1. Gather nearby edges. No candidates -> nothing to match.
        cands = self._candidates(x, y)
        if not cands:
            return None

        sigma = self._sigma_for(accuracy_m)
        emissions = [self._log_emission(c["dist"], sigma) for c in cands]

        if self._prev_cands is None:
            # First fix of a route: no transition to score yet.
            delta = emissions[:]
            trans_used = [None] * len(cands)
        else:
            # d_great: straight-line distance between the two raw GPS fixes.
            straight = math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
            delta = [float("-inf")] * len(cands)
            trans_used = [None] * len(cands)

            for j, cand_j in enumerate(cands):
                best_val = float("-inf")
                best_trans = None
                for i, cand_i in enumerate(self._prev_cands):
                    if self._prev_delta[i] == float("-inf"):
                        continue
                    net_dist = self._network_distance(cand_i, cand_j)
                    log_trans = self._log_transition(straight, net_dist)
                    if log_trans is None:
                        continue   # unreachable pair -> excluded, not just penalised
                    val = self._prev_delta[i] + log_trans
                    if val > best_val:
                        best_val = val
                        best_trans = log_trans
                if best_trans is not None:
                    delta[j] = best_val + emissions[j]
                    trans_used[j] = best_trans

            if all(d == float("-inf") for d in delta):
                # Every transition was impossible (e.g. a GPS jump to a
                # disconnected part of the network). Restart the chain from
                # this fix using emission probability alone, so the matcher
                # recovers instead of staying stuck at -inf forever.
                delta = emissions[:]
                trans_used = [None] * len(cands)

        best_j = max(range(len(cands)), key=lambda k: delta[k])

        # Keep the running scores bounded by re-basing to the winner each
        # step, so delta can't drift toward -inf over a long route. Only the
        # relative ranking between candidates matters, never the absolute
        # probability value.
        max_delta = delta[best_j]
        delta = [d - max_delta for d in delta]

        self._prev_cands = cands
        self._prev_xy = (x, y)
        self._prev_delta = delta

        chosen = cands[best_j]
        return {
            "x": chosen["x"],
            "y": chosen["y"],
            "edge_id": chosen["edge"].getID(),
            "raw_dist": chosen["dist"],
            "score": delta[best_j],
            "components": {
                "emission": emissions[best_j],
                "transition": trans_used[best_j],
                "sigma": sigma,
                "num_candidates": len(cands),
            },
        }
