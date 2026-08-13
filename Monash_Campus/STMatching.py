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

------------------------------------------------------------------
HOW THE TWO SCORE TERMS WORK, IN ONE PLACE
  Fs : SPATIAL. Two things multiplied together --
         N(c)  observation probability: a Gaussian on how far the raw
               GPS point is from this candidate edge. Pure geometry,
               and the direct equivalent of the topological matcher's
               proximity term.
         V     transmission probability: straight-line GPS distance
               divided by network travel distance. Near 1.0 when the
               road route is as direct as the crow flies; small when
               reaching this candidate would need a long detour. This
               is the term that rejects implausible jumps, and it is
               ST-Matching's equivalent of the connectivity term.
  Ft : TEMPORAL. Cosine similarity between the speed limits of the
       edges on the connecting path and the speed the transition
       implies. Intended to reject paths the rider could not plausibly
       have covered in the elapsed time. (See point 2 below for why it
       does not actually do that.)
Unlike the topological matcher's weighted SUM, these combine by
PRODUCT: F = Fs * Ft. A near-zero on either term kills the transition
outright rather than being outvoted by the remaining terms.
------------------------------------------------------------------

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

# math: hypot (Euclidean distance), exp (the Gaussian), sqrt, log, pi.
import math
# deque with maxlen gives the fixed-size sliding window for free: appending
# past the limit silently evicts the oldest entry, which is exactly the
# fixed-lag behaviour described in point 4 above.
from collections import deque

# sumolib reads the .net.xml and provides the network graph (edges, shapes,
# lengths, speed limits, shortest paths) independently of a running TraCI
# connection -- the matcher does not need SUMO to be live to work.
import sumolib
# geomhelper handles projecting a point onto a polyline and converting
# between "distance along a shape" and an (x, y) position.
from sumolib import geomhelper as gh


# ---- small geometry helpers ------------------------------------------------

def _point_segment_distance(px, py, ax, ay, bx, by):
    """
    Perpendicular distance from point (px,py) to the line SEGMENT a->b.

    Segment, not infinite line: if the point lies off the end of the
    segment, the distance returned is to the nearer endpoint rather than to
    a projection out in space beyond it.

    Args:
        px, py: the point being measured.
        ax, ay: start of the segment.
        bx, by: end of the segment.

    Returns:
        float: distance in metres (SUMO network units are metres).
    """
    # Vector along the segment.
    # (dx, dy) is the direction a->b expressed as a displacement.
    dx, dy = bx - ax, by - ay
    # Squared length of the segment. Squared because the division below
    # needs |ab|^2, so taking a square root here would only be undone.
    seg_len_sq = dx * dx + dy * dy
    # Degenerate segment (a == b): fall back to point-to-point distance.
    # Guards against dividing by zero when a shape contains a duplicated
    # vertex, which does occur in OSM-derived networks.
    if seg_len_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    # t = projection of the point onto the (infinite) line, as a fraction of
    # the segment length. Clamping to [0, 1] keeps the closest point on the
    # actual segment rather than its extension.
    # The numerator is the dot product of (a->p) with (a->b): how far along
    # the segment direction the point lies.
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    # t < 0 means p sits behind a; t > 1 means beyond b. Clamping lands the
    # closest point on the segment itself.
    t = max(0.0, min(1.0, t))
    # Walk t of the way from a towards b to get the closest point (cx, cy).
    cx, cy = ax + t * dx, ay + t * dy
    # hypot is sqrt(dx^2 + dy^2) without intermediate overflow.
    return math.hypot(px - cx, py - cy)


def _cosine_similarity(u_vec, v_vec):
    """
    Cosine similarity between two equal-length non-negative vectors.
    Returns 0.0 if either vector has zero magnitude.

    This is the Ft calculation from Lou et al. Cosine similarity measures
    the ANGLE between two vectors and ignores their magnitudes -- which is
    precisely why the observed speed cancels out of the faithful
    formulation (point 2 of the header comment). Both vectors here hold
    speeds, so both are non-negative, the angle cannot exceed 90 degrees,
    and the result is guaranteed to land in [0, 1].

    Args:
        u_vec: speed limits of the edges along the path.
        v_vec: the implied speed, repeated once per edge.

    Returns:
        float: similarity in [0, 1]; 1 means the vectors point the same way.
    """
    # Both inputs here are speeds (non-negative), so the result lands in
    # [0, 1] and can be used directly as a multiplicative score.
    # Dot product: sum of elementwise products. zip stops at the shorter
    # vector, though the caller always builds both the same length.
    dot = sum(u * v for u, v in zip(u_vec, v_vec))
    # Magnitude (Euclidean norm) of each vector.
    mag_u = math.sqrt(sum(u * u for u in u_vec))
    mag_v = math.sqrt(sum(v * v for v in v_vec))
    # A zero-magnitude vector has no direction, so the angle is undefined.
    # Happens when the rider is stationary (implied speed 0) or every edge
    # on the path reports a zero speed limit. Return 0 rather than divide.
    if mag_u == 0.0 or mag_v == 0.0:
        return 0.0
    # cos(theta) = (u . v) / (|u| |v|).
    return dot / (mag_u * mag_v)


def _safe_log(value, floor=1e-12):
    """
    log() with a floor, so a zero-probability transition doesn't explode.

    math.log(0) raises ValueError, which would break the Viterbi
    accumulation mid-sweep. Clamping to a very small positive number keeps
    the score finite and heavily penalised, which is the behaviour wanted.

    Args:
        value: the probability or score to take the log of.
        floor: smallest value permitted before clamping kicks in.

    Returns:
        float: natural log of max(value, floor).
    """
    # Viterbi is run in log space to avoid underflow when multiplying many
    # small probabilities together over a window. (Eight probabilities of
    # ~1e-3 multiply to 1e-24; adding their logs keeps the numbers sane.)
    return math.log(max(value, floor))


# ---- the matcher -----------------------------------------------------------

class STMatcher:
    """
    Stateful ST-Matching map matcher.

    Build ONE of these per run (the constructor loads the entire road
    network) and call match() once per GPS fix. It keeps a sliding window of
    recent fixes internally so Viterbi has a sequence to work over.
    """

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
        """
        Load the network and store the tuning parameters.

        Args:
            net_file:        path to the scenario's .net.xml.
            search_radius:   candidate edges must pass within this distance
                             of the GPS point. A HARD cutoff -- set too
                             small and a noisy fix has no candidates at all
                             and match() returns None.
            sigma:           assumed std deviation of GPS error, controlling
                             how fast the observation probability decays with
                             distance. Larger is more forgiving of noise.
            window_size:     how many fixes Viterbi runs over. Larger gives
                             more backward context at more cost per step; it
                             does NOT give lookahead (header point 4).
            max_candidates:  per-fix cap on candidate edges. The single most
                             effective latency lever, since transition
                             scoring is quadratic in this number.
            temporal_mode:   which Ft variant to use (see _temporal_score).
            speed_reference: if set, rescales the network's car speed limits
                             onto this bike-plausible top speed (header pt 3).
            nominal_dt:      assumed seconds between fixes when no usable
                             timestamp is supplied.
            vclass:          SUMO vehicle class used to filter candidate
                             edges and constrain routing. None disables it.
        """
        # readNet loads the whole network once (expensive) so the caller
        # should build ONE matcher and reuse it for every fix. This parses
        # the entire .net.xml into an in-memory graph -- seconds for a
        # campus-sized network, so it must never be called inside the loop.
        self.net = sumolib.net.readNet(net_file)
        # Store each tuning parameter on the instance so the scoring methods
        # can reach them without them being threaded through every call.
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
        # maxlen makes eviction automatic: appending to a full deque drops
        # the oldest entry with no bookkeeping needed here.
        self.window = deque(maxlen=window_size)

        # Shortest-path results keyed by (from_edge_id, to_edge_id). The
        # network never changes, so this cache survives reset() and is what
        # keeps the O(C^2) Dijkstra cost tolerable in the live loop.
        # A ride revisits the same candidate pairs constantly (the rider is
        # usually on the same few edges), so the hit rate is very high.
        self._path_cache = {}

    def reset(self):
        """
        Clear matching history. Call between separate routes.

        Without this, the first fix of a new route would be scored as a
        transition from the last fix of the previous one -- a jump of
        arbitrary length that would poison the first several matches.
        """
        # Only the window is route-specific; the path cache is network
        # geometry and stays valid across routes. Deliberately NOT clearing
        # _path_cache: the roads are identical between routes, so discarding
        # it would only force the same Dijkstra queries to be recomputed.
        self.window.clear()

    # -- candidate lookup ----------------------------------------------------

    def _candidates(self, x, y):
        """
        Nearby edges as candidate states, each snapped onto its centreline.

        This builds one column of the Viterbi trellis: every road the rider
        might plausibly be on for this fix, with the geometry each later
        scoring step will need already computed.

        Args:
            x, y: the raw GPS fix in SUMO network coordinates.

        Returns:
            list of dicts with keys edge, dist (perpendicular), offset
            (distance along the edge shape), x/y (the snapped position) and
            obs (observation probability). Distance-sorted closest-first and
            truncated to max_candidates. Empty list if nothing is in range.
        """
        # Ask sumolib for every edge whose geometry passes within
        # search_radius of the point, returned as (edge, distance) pairs.
        # Uses an rtree spatial index when available, brute force if not.
        raw = self.net.getNeighboringEdges(x, y, self.search_radius)
        cands = []
        # Examine each nearby edge and decide whether it is a usable state.
        for edge, dist in raw:
            # Internal edges (IDs beginning ":") are the short connector
            # stubs inside junctions. Excluding them means matches are
            # reported against real named roads, consistent with the
            # granularity used in the CMP evaluation.
            if edge.getID().startswith(":"):
                continue   # skip internal junction edges
            # Optionally drop edges this vehicle class isn't allowed on.
            # With vclass="bicycle" a motorway is not a candidate even if it
            # happens to run alongside the bike path.
            if self.vclass is not None and not edge.allows(self.vclass):
                continue
            # The edge's centreline as a polyline (list of vertices).
            shape = edge.getShape()
            # Offset = how far along the edge the nearest point sits. Needed
            # for the network-distance calculation, not just for snapping.
            # This is the key extra requirement over the topological matcher:
            # measuring travel distance between two candidates needs to know
            # WHERE on each edge the points are, not just which edge.
            offset = gh.polygonOffsetWithMinimumDistanceToPoint((x, y), shape)
            # Convert that along-the-shape offset back into a real (x, y):
            # the on-road position this candidate would place the bike at.
            sx, sy = gh.positionAtShapeOffset(shape, offset)
            # Package everything the scoring functions need, so none of it
            # has to be recomputed inside the O(C^2) transition loop.
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
                # exp(-d^2 / 2 sigma^2) is 1 when the point lies on the edge
                # and decays towards 0 as it gets further away.
                "obs": (1.0 / (math.sqrt(2.0 * math.pi) * self.sigma))
                       * math.exp(-(dist * dist) / (2.0 * self.sigma ** 2)),
            })

        # Keep only the closest max_candidates. Transition scoring is
        # quadratic in this number, so it is the main latency lever.
        # Sorting by perpendicular distance is equivalent to sorting by
        # observation probability descending, because the Gaussian decreases
        # monotonically with distance.
        cands.sort(key=lambda c: c["dist"])
        # Slicing past the end is safe when fewer candidates exist than the cap.
        return cands[: self.max_candidates]

    # -- network distance ----------------------------------------------------

    def _edge_path(self, from_edge, to_edge):
        """
        Shortest edge path from_edge -> to_edge, cached.

        Wraps sumolib's Dijkstra with a memo, because the same edge pairs
        recur constantly across steps and this is the most expensive
        operation in the matcher.

        Args:
            from_edge, to_edge: sumolib Edge objects.

        Returns:
            (path_edges, intermediate_length) or (None, None) if the target
            is unreachable. intermediate_length is the summed length of the
            edges strictly BETWEEN the two, so partial offsets can be added
            by the caller.
        """
        # Key on edge IDs (strings) rather than Edge objects, so the key is
        # hashable and stable.
        key = (from_edge.getID(), to_edge.getID())
        # Cache hit: skip Dijkstra entirely. Failures are cached too (as
        # (None, None)), so an unreachable pair is only ever computed once.
        if key in self._path_cache:
            return self._path_cache[key]

        # Default to "no path" so every branch below has something to assign.
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
            # Retry without it rather than failing the whole match.
            path, _cost = self.net.getShortestPath(from_edge, to_edge)
        except Exception:
            # Any other routing failure is treated as "unreachable" so one
            # bad pair cannot bring down the simulation loop.
            path = None

        # Empty or None means the target cannot be reached from the source.
        if not path:
            result = (None, None)
        else:
            # Sum the lengths of the edges strictly between source and
            # target. The [1:-1] slice excludes both endpoints, because the
            # caller adds partial distances for those using the offsets.
            intermediate = sum(e.getLength() for e in path[1:-1])
            # list() copies the path so a later mutation elsewhere cannot
            # corrupt the cached entry.
            result = (list(path), intermediate)

        # Memoise before returning -- failures included.
        self._path_cache[key] = result
        return result

    def _network_distance(self, cand_a, cand_b):
        """
        Travel distance along the network from candidate a to candidate b,
        accounting for where on each edge the two points actually sit.

        This is w(c_i-1, c_i) in Lou's transmission probability: how far the
        rider would have to travel along roads, as opposed to the
        straight-line distance between the two GPS readings.

        Args:
            cand_a: candidate dict for the earlier fix.
            cand_b: candidate dict for the later fix.

        Returns:
            (distance_m, path_edges), or (None, None) if b is unreachable
            from a.
        """
        # Pull the Edge objects out of the candidate dicts for readability.
        edge_a, edge_b = cand_a["edge"], cand_b["edge"]

        # Same edge: the distance is just the difference in offsets. A
        # negative difference means the point moved backwards along the
        # edge, which at 1 Hz is almost always GPS jitter rather than a real
        # reversal, so take the magnitude rather than rejecting the pair.
        # This is by far the most common case at 1 Hz, and it short-circuits
        # before any routing call -- the reason the matcher is usable live.
        if edge_a.getID() == edge_b.getID():
            return abs(cand_b["offset"] - cand_a["offset"]), [edge_a]

        # Different edges: ask for (and cache) the shortest road path.
        path, intermediate = self._edge_path(edge_a, edge_b)
        # No route between them -- the caller scores this as impossible.
        if path is None:
            return None, None

        # Remaining length on the first edge + whole edges in between +
        # distance into the last edge.
        # Without the offset terms this would measure edge-start to
        # edge-start and badly overstate short hops across a junction.
        dist = (edge_a.getLength() - cand_a["offset"]) + intermediate + cand_b["offset"]
        # Clamp at zero: rounding in the offsets can in principle produce a
        # very small negative, which is meaningless as a distance.
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

        Args:
            cand_a:        candidate dict for the earlier fix.
            cand_b:        candidate dict for the later fix (the one scored).
            straight_dist: crow-flies distance between the two RAW fixes.
            net_dist:      along-the-road distance between the candidates,
                           or None if unreachable.

        Returns:
            (fs, trans) on a reachable pair.
            CAUTION: the unreachable branch returns a bare 0.0, NOT a tuple.
            See the comment on that line.
        """
        # Unreachable: no road connects these two candidates, so this
        # transition is impossible and scores zero.
        # WARNING -- INCONSISTENT RETURN TYPE. This branch returns a SCALAR
        # while the success path returns a TUPLE, and the caller unpacks with
        # "fs, trans = self._spatial_score(...)". That unpacking raises
        # TypeError whenever this branch is taken. Left unchanged here
        # because this pass was comments-only; fix before any evaluation run
        # by returning (0.0, 0.0) instead.
        if net_dist is None:
            return 0.0            # unreachable -> impossible transition
        if net_dist <= 0.0:
            # Both fixes snapped to the same on-edge point (stationary rider).
            # No detour is possible, so the transmission term is perfect.
            # This also avoids dividing by zero on the branch below.
            trans = 1.0
        else:
            # V = d(p_i-1, p_i) / w(c_i-1, c_i). Close to 1 when the road
            # route is about as direct as the straight line; small when
            # reaching this candidate would demand a long way round.
            trans = straight_dist / net_dist
        # Snapping can make the network path shorter than the straight line;
        # clamp so V stays a probability rather than rewarding that artefact.
        # (Both points are pulled onto centrelines, which can shorten the
        # road distance below the raw GPS separation.)
        trans = min(trans, 1.0)
        # Fs is the product: a candidate must be BOTH geometrically close
        # AND reachable without a detour to score well.
        # trans is returned alongside so the caller can log it -- its
        # distribution across candidates is the direct evidence for header
        # point 1.
        return cand_b["obs"] * trans, trans

    def _temporal_score(self, path_edges, net_dist, dt, observed_speed):
        """
        Ft: cosine similarity between the speed limits along the connecting
        path and the speed the transition implies.

        Three modes, selected by self.temporal_mode:
          "lou"   - faithful to the published formula. Use for the headline
                    comparison.
          "ratio" - corrected variant that actually consumes the observed
                    speed, for the side-by-side in the report.
          "off"   - returns 1.0 always, so F reduces to Fs alone. Useful as
                    an ablation showing what the temporal term contributes.

        Args:
            path_edges:     edges on the connecting road path.
            net_dist:       along-the-road distance for this transition.
            dt:             elapsed seconds between the two fixes.
            observed_speed: the phone's reported speed in m/s, or None.

        Returns:
            float in [0, 1]; 1.0 is neutral (no penalty applied).

        See point 2 of the header comment -- under temporal_mode="lou" the
        implied speed cancels out of this expression entirely and Ft reduces
        to a measure of speed-limit uniformity along the path. That is
        faithful to the published method, not a bug.
        """
        # Disabled, or no path to score (same-edge transitions still supply a
        # one-element path, so in practice this is the unreachable case).
        # 1.0 is the multiplicative identity, leaving F = Fs unchanged.
        if self.temporal_mode == "off" or not path_edges:
            return 1.0            # neutral: F reduces to Fs alone

        # u_k: the speed constraint of each edge on the path.
        # getSpeed() returns the edge's speed limit in m/s.
        limits = [e.getSpeed() for e in path_edges]
        if self.speed_reference is not None:
            # Rescale car-oriented limits onto a bike-plausible band while
            # preserving their relative differences (point 3 of the header).
            # Dividing by the maximum normalises to [0, 1]; multiplying by
            # speed_reference maps the fastest edge onto a plausible bike
            # top speed with the others held in proportion.
            top = max(limits) if limits else 0.0
            if top > 0.0:
                limits = [self.speed_reference * (u / top) for u in limits]
        # No usable limits (empty path, or every edge reporting zero). There
        # is nothing to compare against, so stay neutral rather than
        # penalising a candidate for missing network data.
        if not limits or max(limits) <= 0.0:
            return 1.0

        # v': the average speed implied by this transition.
        # Fall back to nominal_dt when dt is missing, zero or negative
        # (duplicate timestamps, clock skew) to avoid dividing by zero.
        dt = dt if dt and dt > 0.0 else self.nominal_dt
        # distance / time = the speed the rider must have held to cover this
        # particular candidate path in the elapsed time.
        implied_speed = (net_dist / dt) if net_dist is not None else 0.0

        if self.temporal_mode == "ratio":
            # Corrected variant: compare the implied speed against each
            # edge's limit directly, so the observed motion actually
            # influences the score. Uses the measured speed when the phone
            # supplies one, falling back to the implied speed.
            v_meas = observed_speed if observed_speed is not None else implied_speed
            # min(a/b, b/a) is a symmetric ratio in [0, 1]: 1 when the two
            # speeds match, falling away whether the rider is implausibly
            # faster OR slower than the limit. Guarded against /0 on both.
            per_edge = [min(v_meas / u, u / v_meas) if v_meas > 0.0 and u > 0.0 else 0.0
                        for u in limits]
            # Mean across the path, so longer paths are not penalised for
            # length alone.
            return sum(per_edge) / len(per_edge)

        # Faithful Lou formulation: v' repeated once per path segment.
        # Building this constant vector is exactly what causes the
        # cancellation -- a vector of identical components always points
        # along the diagonal regardless of magnitude, so cosine similarity
        # cannot see v' at all.
        speeds = [implied_speed] * len(limits)
        return _cosine_similarity(limits, speeds)

    # -- Viterbi over the window ---------------------------------------------

    def _run_viterbi(self):
        """
        Find the highest-scoring candidate sequence across the current window.

        Standard Viterbi: sweep forward through the window keeping, for each
        candidate at the current fix, the best cumulative score of any path
        ending there plus a backpointer to the predecessor that achieved it.
        Runs in log space so probabilities add instead of multiplying.

        Returns:
            (best_last_index, (scores, backpointers, detail)) where
            best_last_index indexes the winning candidate at the NEWEST fix,
            or (None, None) if the window has no scorable candidates.

        Note this re-runs from scratch on every new fix. The window is small
        (window_size fixes x max_candidates states) so this is cheap given
        the shortest-path cache, and it keeps the code readable. It also
        means earlier points may be re-decided as new evidence arrives --
        harmless for logging, but the bike has already been moved, so those
        revisions are never applied. See point 4 of the header comment.
        """
        # Snapshot the deque as a list so it can be indexed by position.
        frames = list(self.window)
        # Nothing to do if the window is empty or its oldest fix found no
        # candidates -- there is no state to start the chain from.
        if not frames or not frames[0]["cands"]:
            return None, None

        # Initialise with the observation probability of the oldest fix in
        # the window. ST-Matching has no separate prior over start states.
        # One score per candidate of the first frame; log space from here on.
        scores = [_safe_log(c["obs"]) for c in frames[0]["cands"]]
        # backpointers[i][j] = index of the predecessor candidate that gave
        # candidate j of frame i+1 its best score.
        backpointers = []
        detail = []   # per-step component values for the winning transitions

        # Sweep forward from the second frame: each iteration scores the
        # transitions INTO frame i from frame i-1.
        for i in range(1, len(frames)):
            prev_frame, curr_frame = frames[i - 1], frames[i]
            prev_cands, curr_cands = prev_frame["cands"], curr_frame["cands"]

            if not curr_cands:
                # No candidates at this fix -- the chain cannot continue.
                # (In practice match() returns early before this, since it
                # never appends a candidate-less frame to the window.)
                return None, None

            # Straight-line distance between the two RAW GPS fixes, d(p_i-1, p_i).
            # Raw deliberately: this is the observed displacement, which the
            # transmission probability compares the road distance against.
            straight = math.hypot(
                curr_frame["x"] - prev_frame["x"],
                curr_frame["y"] - prev_frame["y"],
            )
            # Elapsed time between the two fixes, for the temporal term.
            dt = curr_frame["t"] - prev_frame["t"]

            # Fresh accumulators for this frame. -inf marks "no path reaches
            # this candidate yet" and loses every comparison below.
            new_scores = [float("-inf")] * len(curr_cands)
            new_back = [None] * len(curr_cands)
            new_detail = [None] * len(curr_cands)

            # For every candidate at the current fix...
            for j, cand_b in enumerate(curr_cands):
                # ...consider arriving from every candidate at the previous
                # fix. This double loop is the O(C^2) cost of header point 6.
                for k, cand_a in enumerate(prev_cands):
                    # Skip predecessors nothing can reach -- no path through
                    # them can possibly be optimal.
                    if scores[k] == float("-inf"):
                        continue

                    # How far along the road, and via which edges.
                    net_dist, path_edges = self._network_distance(cand_a, cand_b)
                    # Spatial half of F.
                    fs, trans = self._spatial_score(cand_a, cand_b, straight, net_dist)
                    # Zero spatial score = impossible transition; skip rather
                    # than accumulate a floored log.
                    if fs <= 0.0:
                        continue
                    # Temporal half of F.
                    ft = self._temporal_score(
                        path_edges, net_dist, dt, curr_frame["speed"]
                    )

                    # F = Fs * Ft, accumulated as a sum of logs.
                    # log(prev * fs * ft) = log(prev) + log(fs) + log(ft).
                    total = scores[k] + _safe_log(fs) + _safe_log(ft)
                    # Keep only the best way to reach cand_b -- the core
                    # Viterbi step. Discarding the rest is what keeps the
                    # cost linear in window length instead of exponential.
                    if total > new_scores[j]:
                        new_scores[j] = total
                        new_back[j] = k
                        # Stash the component values of the winning
                        # transition for logging and the write-up.
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
                # Without this the chain would stay dead for the remainder of
                # the ride after a single bad fix.
                new_scores = [_safe_log(c["obs"]) for c in curr_cands]
                # No predecessors, because the chain was severed here.
                new_back = [None] * len(curr_cands)
                new_detail = [{"obs": c["obs"], "trans": None, "fs": c["obs"], "ft": None}
                              for c in curr_cands]

            # Roll forward: this frame's scores become the next iteration's
            # "previous" scores.
            scores = new_scores
            backpointers.append(new_back)
            detail.append(new_detail)

        # The winning candidate at the newest fix is the one with the best
        # cumulative score. (The backpointers would let the whole path be
        # traced back from here; only the endpoint is needed live.)
        best_index = max(range(len(scores)), key=lambda i: scores[i])
        return best_index, (scores, backpointers, detail)

    # -- main entry point ----------------------------------------------------

    def match(self, x, y, timestamp=None, speed_mps=None, course_deg=None):
        """
        Match one GPS fix (given in SUMO network x, y) to the best edge.

        Called once per fix by the live loop. Four stages: build candidates,
        push the fix onto the window, run Viterbi across the window, and
        return the decision for the NEWEST fix -- the point the bike has to
        be moved to now.

        Args:
            x, y:       the fix in SUMO network coordinates.
            timestamp:  fix time in epoch seconds, for the temporal term.
                        Falls back to nominal spacing when omitted.
            speed_mps:  phone-reported speed, used by temporal_mode="ratio".
            course_deg: accepted only so this signature stays interchangeable
                        with TopologicalMatcher.match(); ST-Matching does not
                        use heading (point 5 of the header comment).

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
        # Returning None (rather than guessing) lets the caller skip the
        # update, which keeps the method comparison clean.
        cands = self._candidates(x, y)
        if not cands:
            return None

        # 2. Push this fix onto the sliding window. deque(maxlen) drops the
        #    oldest automatically, giving the fixed-lag behaviour.
        # With no timestamp supplied, synthesise one by advancing the
        # previous fix's time by nominal_dt so dt stays sane. The first fix
        # of a route with no timestamp simply starts the clock at 0.
        t = timestamp if timestamp is not None else (
            self.window[-1]["t"] + self.nominal_dt if self.window else 0.0
        )
        # The candidate list is stored WITH the frame so Viterbi does not
        # recompute it for older fixes on every sweep.
        self.window.append({
            "x": x, "y": y, "t": t, "speed": speed_mps, "cands": cands,
        })

        # 3. Single fix in the window (first point of a route): no transition
        #    exists yet, so fall back to the observation probability alone.
        if len(self.window) == 1:
            best = cands[0]   # candidates are distance-sorted, so [0] maximises N
            # trans and ft are None (not 0) to mark "not applicable here"
            # rather than "scored zero" -- the log formatter distinguishes
            # the two, and so should any later analysis of the components.
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
            # Degrades to geometric matching for one step rather than
            # freezing the bike or returning nothing at all.
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

        # Unpack the Viterbi results. The backpointers go unused live --
        # only the endpoint matters, since earlier fixes have already been
        # acted on and cannot be revised (header point 4).
        scores, _backpointers, detail = viterbi
        # best_index indexes the current frame's candidate list, which is the
        # same list built in step 1.
        chosen = cands[best_index]
        # Component values for the transition that won at this fix. Log these
        # per step -- the distribution of trans and ft across candidates is
        # the direct evidence for points 1-3 of the header comment.
        # detail[-1] is the newest frame; the fallback covers a restarted
        # chain, where no transition detail was recorded.
        comps = detail[-1][best_index] if detail and detail[-1][best_index] else {
            "obs": chosen["obs"], "trans": None, "fs": chosen["obs"], "ft": None
        }

        # Hand back the on-edge position plus diagnostics, in the same shape
        # TopologicalMatcher returns, so the live loop stays agnostic to
        # which matcher it is holding.
        return {
            "x": chosen["x"],
            "y": chosen["y"],
            "edge_id": chosen["edge"].getID(),
            "raw_dist": chosen["dist"],
            "score": scores[best_index],
            "components": comps,
            "window_len": len(self.window),
        }