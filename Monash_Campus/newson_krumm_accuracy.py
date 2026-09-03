###
# Newson Krumm Accuracy Calculations
# Written by: Jordan BUdiono (GenAI assisted)
#  Date: 12/08/2026
###

import numpy as np
from shapely.geometry import LineString


def newson_krumm_accuracy(gt_x, gt_y, matched_x, matched_y, buffer_dist=10.0):
    """
    Newson & Krumm (2009) map-matching accuracy metric.

    Compares a ground-truth route against a map-matched route and returns
    how much of each route's length disagrees with the other, normalized
    by the ground-truth length.

        d0 = length of the ground-truth route
        d+ = length of matched route NOT overlapping the ground-truth route
             (spurious / over-matched length)
        d- = length of ground-truth route NOT covered by the matched route
             (missed / under-matched length)

        error    = (d+ + d-) / d0
        accuracy = 1 - error   <- this is the paper's actual "accuracy"
                                  (note: (d+ + d-)/d0 by itself is the
                                  ERROR term, not accuracy)

    Parameters
    ----------
    gt_x, gt_y : array-like
        x, y coordinates (same planar/metric units, e.g. meters) of the
        ground-truth / raw GPS route, in travel order.
    matched_x, matched_y : array-like
        x, y coordinates of the map-matched route, in travel order.
    buffer_dist : float
        Distance (same units as x/y) within which the two routes are
        considered "the same road" / overlapping. Tune to your GPS/road
        accuracy -- 5-15 (meters) is typical.

    Returns
    -------
    dict with keys: d0, d_plus, d_minus, error, accuracy

    Notes
    -----
    - x/y must already be in a consistent planar (metric) coordinate
      system -- e.g. UTM meters, or a local projection such as SUMO net
      coordinates. Do not pass raw lat/lon directly.
    - NaNs are dropped from each route independently before building the
      route geometry. If your data has gaps, this will "jump" over them.
    """
    gt_x, gt_y = np.asarray(gt_x, dtype=float), np.asarray(gt_y, dtype=float)
    matched_x, matched_y = np.asarray(matched_x, dtype=float), np.asarray(matched_y, dtype=float)

    gt_mask = ~(np.isnan(gt_x) | np.isnan(gt_y))
    matched_mask = ~(np.isnan(matched_x) | np.isnan(matched_y))

    gt_coords = list(zip(gt_x[gt_mask], gt_y[gt_mask]))
    matched_coords = list(zip(matched_x[matched_mask], matched_y[matched_mask]))

    if len(gt_coords) < 2:
        raise ValueError("ground-truth route needs at least 2 valid points")
    if len(matched_coords) < 2:
        raise ValueError("matched route needs at least 2 valid points")

    gt_line = LineString(gt_coords)
    matched_line = LineString(matched_coords)

    gt_buffer = gt_line.buffer(buffer_dist)
    matched_buffer = matched_line.buffer(buffer_dist)

    d0 = gt_line.length
    d_plus = matched_line.difference(gt_buffer).length
    d_minus = gt_line.difference(matched_buffer).length

    error = (d_plus + d_minus) / d0 if d0 > 0 else float("nan")
    accuracy = 1.0 - error

    return {
        "d0": d0,
        "d_plus": d_plus,
        "d_minus": d_minus,
        "error": error,
        "accuracy": accuracy,
    }


if __name__ == "__main__":
    # Example usage on full_campus_2.csv
    import pandas as pd

    df = pd.read_csv("full_campus_2.csv")

    result = newson_krumm_accuracy(
        df["gps_x"], df["gps_y"],
        df["keep0_sumo_x"], df["keep0_sumo_y"],
        buffer_dist=1.75,
    )
    print(result)