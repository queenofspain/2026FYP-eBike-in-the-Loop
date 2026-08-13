"""
Constant-velocity Kalman filter for smoothing live phone GPS before it
reaches SUMO, for the eBike-in-the-Loop platform.

Unlike HMM.py / topological.py / STMatching.py, this is not a map
matcher -- it doesn't know about roads or edges. It only smooths the
raw (x, y) track: state = [pos, vel] per axis, predicted forward each
step and corrected against the new GPS fix. It exists to sit BEFORE
those matchers (or before a plain moveToXY(keepRoute=0) call), so the
position/heading/speed handed downstream is less jittery than the raw
phone reading -- see the "no filtering anywhere in main" gap noted for
this project.

Input/output coordinates are SUMO *network* (x, y) in metres, same as
the map matchers -- from traci.simulation.convertGeo(lon, lat,
fromGeo=True). x and y are filtered independently (their process and
measurement noise are uncorrelated), so internally this is just two
identical 1D constant-velocity filters, one per axis.

Call update() once per GPS fix, in chronological order. Call reset()
between separate routes -- exactly like the map matchers' convention.

update() returns a dict:
    {
        "x", "y":         filtered position (metres, SUMO network coords)
        "vx", "vy":       filtered velocity components (m/s)
        "speed_mps":      filtered speed, hypot(vx, vy)
        "course_deg":     filtered compass heading (0=N, 90=E), derived
                           from (vx, vy) -- see NOTE below
        "sigma":          GPS measurement std (m) used for this fix
    }

NOTE on course_deg: it assumes SUMO's projected network is north-up
(x increases east, y increases north), which holds for the standard
netconvert/UTM-style projection OSM Web Wizard produces (as used by
this project's 2026-03-11-17-20-46 map). If a net was imported with a
different projection this field will be rotated -- ignore it and use
the phone's own course_deg in that case; it has no effect on x/y.

NOTE on integration: the natural call site is in live_phone_to_sumo.py,
right after convertGeo() and before moveToXY() -- filter the raw (x, y)
there and pass the filtered values (and optionally filtered speed) into
moveToXY/setSpeed instead of the raw ones.
------------------------------------------------------------------
"""

import math


class _AxisKalman:
    """
    One-dimensional constant-velocity Kalman filter for a single axis.

    State is [pos, vel]; covariance P is the symmetric 2x2 matrix
    [[p_pp, p_pv], [p_pv, p_vv]], stored as three scalars. Kept private
    and instantiated twice (x, y) by KalmanFilter below, rather than
    written as general NxN matrix code -- this project has no numpy
    dependency anywhere, and a 2x2 system is simpler done by hand.
    """

    def __init__(self, pos, vel, p_pp, p_vv):
        self.pos = pos
        self.vel = vel
        self.p_pp = p_pp
        self.p_pv = 0.0
        self.p_vv = p_vv

    def step(self, z, dt, q, r):
        """Predict forward by dt, then correct against measurement z."""
        # -- predict: F = [[1, dt], [0, 1]] --
        self.pos = self.pos + self.vel * dt
        p_pp = self.p_pp + 2.0 * dt * self.p_pv + dt * dt * self.p_vv
        p_pv = self.p_pv + dt * self.p_vv
        p_vv = self.p_vv

        # discretized white-noise-acceleration process noise
        p_pp += q * dt ** 3 / 3.0
        p_pv += q * dt ** 2 / 2.0
        p_vv += q * dt

        # -- update: H = [1, 0], measurement is position only --
        innovation = z - self.pos
        s = p_pp + r
        k_pos = p_pp / s
        k_vel = p_pv / s

        self.pos = self.pos + k_pos * innovation
        self.vel = self.vel + k_vel * innovation

        self.p_pp = (1.0 - k_pos) * p_pp
        self.p_pv = (1.0 - k_pos) * p_pv
        self.p_vv = p_vv - k_vel * p_pv

        return self.pos, self.vel


class KalmanFilter:
    def __init__(
        self,
        process_noise=1.0,        # (m/s^2)^2; higher = trust the GPS more
                                    # and follow it more closely, lower =
                                    # smoother but laggier on turns
        sigma_default=4.07,        # metres; Newson & Krumm's GPS error std,
                                    # same constant HMM.py uses, applied when
                                    # the phone reports no accuracy
        use_accuracy=True,         # use the phone's per-fix accuracy_m as
                                    # measurement sigma when available,
                                    # instead of sigma_default
        min_sigma=1.0,             # metres; floor so a near-zero accuracy
                                    # reading can't collapse trust to a spike
        min_dt=0.05,                # seconds; floor on dt for numerical safety
        max_dt=5.0,                 # seconds; ceiling -- same order as this
                                    # project's STALE_DATA_SECONDS, so a long
                                    # gap doesn't blow up the covariance
        initial_position_variance=100.0,  # m^2; how much to trust the very
                                            # first fix's position
        initial_velocity_variance=25.0,   # (m/s)^2; how much to trust the
                                            # very first fix's velocity guess
    ):
        self.process_noise = process_noise
        self.sigma_default = sigma_default
        self.use_accuracy = use_accuracy
        self.min_sigma = min_sigma
        self.min_dt = min_dt
        self.max_dt = max_dt
        self.initial_position_variance = initial_position_variance
        self.initial_velocity_variance = initial_velocity_variance

        self._kx = None
        self._ky = None

    def reset(self):
        """Clear filter state. Call between separate routes."""
        self._kx = None
        self._ky = None

    def _sigma_for(self, accuracy_m):
        # Same convention as HMM.py's _sigma_for: per-fix accuracy when
        # available, else the fixed Newson & Krumm constant.
        if self.use_accuracy and accuracy_m is not None and accuracy_m > 0:
            return max(accuracy_m, self.min_sigma)
        return self.sigma_default

    def update(self, x, y, dt, speed_mps=None, course_deg=None, accuracy_m=None):
        """
        Filter one GPS fix.

        x, y        : raw SUMO network coordinates for this fix (metres)
        dt          : seconds since the previous fix (caller-supplied --
                      e.g. the actual gap between phone timestamps, or the
                      live loop's fixed poll interval)
        speed_mps,
        course_deg  : optional phone-reported speed/heading, used only to
                      seed initial velocity on the very first fix of a route
        accuracy_m  : optional phone-reported GPS accuracy for this fix

        Returns a dict; see module docstring for its shape.
        """
        sigma = self._sigma_for(accuracy_m)
        r = sigma * sigma

        if self._kx is None:
            # First fix of a route: nothing to predict from yet, so seed
            # state directly from this fix (mirrors the map matchers'
            # "no history -> pass through" handling of their first point).
            if speed_mps is not None and course_deg is not None:
                bearing = math.radians(course_deg)
                vx0 = speed_mps * math.sin(bearing)
                vy0 = speed_mps * math.cos(bearing)
            else:
                vx0 = vy0 = 0.0

            self._kx = _AxisKalman(x, vx0, self.initial_position_variance, self.initial_velocity_variance)
            self._ky = _AxisKalman(y, vy0, self.initial_position_variance, self.initial_velocity_variance)

            fx, fvx = self._kx.pos, self._kx.vel
            fy, fvy = self._ky.pos, self._ky.vel
        else:
            dt = max(self.min_dt, min(dt, self.max_dt))
            fx, fvx = self._kx.step(x, dt, self.process_noise, r)
            fy, fvy = self._ky.step(y, dt, self.process_noise, r)

        return {
            "x": fx,
            "y": fy,
            "vx": fvx,
            "vy": fvy,
            "speed_mps": math.hypot(fvx, fvy),
            "course_deg": math.degrees(math.atan2(fvx, fvy)) % 360.0,
            "sigma": sigma,
        }


if __name__ == "__main__":
    # Self-test: no SUMO, no phone, no server -- just synthetic noisy data.
    # Run with `python kalman_filter.py`. Demonstrates two things a live
    # ride would otherwise be needed to check:
    #   1. Under steady GPS noise, filtered RMS error is lower than raw.
    #   2. A single large outlier fix (simulated multipath) is damped
    #      instead of being followed exactly.
    import random

    random.seed(42)  # matches this project's convention (build.bat, etc.)

    kf = KalmanFilter()

    dt = 1.0
    true_speed = 4.0  # m/s, roughly bike pace
    gps_noise_std = 4.07
    n_steps = 60
    outlier_step = 30
    outlier_offset = 40.0  # metres -- a bad multipath-style jump

    raw_sq_err = 0.0
    filt_sq_err = 0.0

    true_x, true_y = 0.0, 0.0
    for i in range(n_steps):
        true_x += true_speed * dt
        # true_y stays 0: straight-line motion along x

        noisy_x = true_x + random.gauss(0.0, gps_noise_std)
        noisy_y = true_y + random.gauss(0.0, gps_noise_std)
        if i == outlier_step:
            noisy_x += outlier_offset

        result = kf.update(noisy_x, noisy_y, dt, accuracy_m=gps_noise_std)

        raw_sq_err += (noisy_x - true_x) ** 2 + (noisy_y - true_y) ** 2
        filt_sq_err += (result["x"] - true_x) ** 2 + (result["y"] - true_y) ** 2

        tag = "  <-- outlier fix" if i == outlier_step else ""
        print(
            f"step {i:2d}: true=({true_x:7.2f},{true_y:6.2f})  "
            f"raw=({noisy_x:7.2f},{noisy_y:6.2f})  "
            f"filtered=({result['x']:7.2f},{result['y']:6.2f})  "
            f"speed={result['speed_mps']:.2f} m/s{tag}"
        )

    raw_rms = math.sqrt(raw_sq_err / n_steps)
    filt_rms = math.sqrt(filt_sq_err / n_steps)
    print()
    print(f"raw RMS error:      {raw_rms:.2f} m")
    print(f"filtered RMS error: {filt_rms:.2f} m")
    assert filt_rms < raw_rms, "filter should reduce RMS error vs raw GPS"
    print("OK: filter reduced RMS error and damped the injected outlier.")
