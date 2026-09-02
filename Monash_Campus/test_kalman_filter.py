"""
Self-test for kalman_filter.KalmanFilter, using synthetic noisy GPS data.

only synthetic data, to check the filter math on its own before trusting it inside the live pipeline.
Demonstrates two things a live ride would otherwise be needed to check:
    1. Under steady GPS noise, filtered RMS error is lower than raw.
    2. A single large outlier fix (simulated multipath) is damped
       instead of being followed exactly.

Run with `python test_kalman_filter.py`.
"""

import math
import random

from kalman_filter import KalmanFilter


def main():
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


if __name__ == "__main__":
    main()
