"""ROS-free depth comparison; callers supply durations from simulation time only."""

import numpy as np


def decode_depth(message):
    """Decode an owned 32FC1 image, preserving padded rows and byte order."""
    if message.encoding != "32FC1":
        raise ValueError("depth encoding must be 32FC1")
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step < width * 4:
        raise ValueError("invalid depth dimensions or step")
    buffer = memoryview(message.data)
    if buffer.nbytes < step * height:
        raise ValueError("depth data is shorter than height * step")
    dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
    view = np.ndarray((height, width), dtype=dtype, buffer=buffer, strides=(step, 4))
    return view.astype(np.float32, copy=True)


def _intrinsics(k):
    matrix = np.asarray(k, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("K needs finite values and positive focal lengths")
    if not np.allclose(matrix[[0, 1, 2, 2, 2], [1, 0, 0, 1, 2]], [0, 0, 0, 0, 1]):
        raise ValueError("only zero-skew pinhole K is supported")
    return matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]


def resample_reference(reference, reference_k, stereo_k, stereo_shape):
    """Nearest sampling at stereo pixel rays in the shared optical frame.

    K refers to integer pixel centres. No extra half-pixel correction is applied:
    resized K must already contain cx_out=(cx_in+.5)/factor-.5. Out-of-FOV
    rays become NaN; reference NaN and infinities remain unchanged in view.
    """
    reference = np.asarray(reference)
    if reference.ndim != 2 or min(reference.shape) <= 0:
        raise ValueError("reference must be a nonempty 2D depth array")
    height, width = (int(value) for value in stereo_shape)
    if height <= 0 or width <= 0:
        raise ValueError("stereo shape must be positive")
    rfx, rfy, rcx, rcy = _intrinsics(reference_k)
    sfx, sfy, scx, scy = _intrinsics(stereo_k)
    u = (np.arange(width, dtype=np.float64) - scx) * rfx / sfx + rcx
    v = (np.arange(height, dtype=np.float64) - scy) * rfy / sfy + rcy
    # Reject out-of-image rays before rounding; exact half-pixel ties go upward.
    inside_u = (u >= -0.5) & (u < reference.shape[1] - 0.5)
    inside_v = (v >= -0.5) & (v < reference.shape[0] - 0.5)
    indices_u = np.clip(np.floor(u + 0.5), 0, reference.shape[1] - 1).astype(np.int64)
    indices_v = np.clip(np.floor(v + 0.5), 0, reference.shape[0] - 1).astype(np.int64)
    output = reference[np.ix_(indices_v, indices_u)].astype(np.float32, copy=True)
    output[~(inside_v[:, None] & inside_u[None, :])] = np.nan
    return output


def _ratio(numerator, denominator):
    return float(numerator / denominator) if denominator else None


def _errors(stereo, reference, mask, threshold):
    count = int(np.count_nonzero(mask))
    valid = np.isfinite(stereo) & (stereo > 0) & mask
    valid_count = int(np.count_nonzero(valid))
    delta = stereo[valid].astype(np.float64) - reference[valid].astype(np.float64)
    absolute = np.abs(delta)
    return {
        "reference_count": count,
        "stereo_valid_count": valid_count,
        "stereo_valid_fraction": _ratio(valid_count, count),
        "signed_mean_m": float(np.mean(delta)) if valid_count else None,
        "signed_median_m": float(np.median(delta)) if valid_count else None,
        "absolute_p90_m": float(np.percentile(absolute, 90)) if valid_count else None,
        "absolute_p95_m": float(np.percentile(absolute, 95)) if valid_count else None,
        "rmse_m": float(np.sqrt(np.mean(delta * delta))) if valid_count else None,
        "too_near_count": int(np.count_nonzero(delta < -threshold)),
        "too_far_count": int(np.count_nonzero(delta > threshold)),
        "too_near_fraction": _ratio(int(np.count_nonzero(delta < -threshold)), valid_count),
        "too_far_fraction": _ratio(int(np.count_nonzero(delta > threshold)), valid_count),
    }


def depth_metrics(stereo, reference, min_depth=0.25, max_depth=3.0, error_threshold=0.1):
    """Summarize same-ray depths. Error rates use valid matched pixels.

    Reference validity includes both range endpoints. The bands partition it,
    with the last band including 3m. Finite positive stereo depths are valid
    regardless of distance; their signed errors reveal wrong distant matches.
    Empty populations use None, so JSON contains no NaN/Infinity values.
    """
    stereo, reference = np.asarray(stereo), np.asarray(reference)
    if stereo.ndim != 2 or stereo.shape != reference.shape or stereo.size == 0:
        raise ValueError("depth arrays must have the same nonempty 2D shape")
    if not np.all(np.isfinite([min_depth, max_depth, error_threshold])) or (
        min_depth <= 0 or max_depth <= min_depth or error_threshold <= 0
    ):
        raise ValueError("depth range and error threshold must be finite and positive")
    finite_positive = np.isfinite(reference) & (reference > 0)
    evaluated = finite_positive & (reference >= min_depth) & (reference <= max_depth)
    stereo_valid = np.isfinite(stereo) & (stereo > 0)
    categories = {
        "nan": np.isnan(reference),
        "positive_inf": np.isposinf(reference),
        "negative_inf": np.isneginf(reference),
        "nonpositive": np.isfinite(reference) & (reference <= 0),
        "below_range": finite_positive & (reference < min_depth),
        "above_range": finite_positive & (reference > max_depth),
    }
    result = _errors(stereo, reference, evaluated, error_threshold)
    result["total_pixels"] = int(stereo.size)
    result["reference_other_categories"] = {
        name: {
            "reference_count": int(np.count_nonzero(mask)),
            "stereo_valid_count": int(np.count_nonzero(mask & stereo_valid)),
        }
        for name, mask in categories.items()
    }
    # Invalid reference cannot establish a false match, but a valid stereo
    # output there is counted explicitly for inspection instead of discarded.
    unknown = ~np.isfinite(reference)
    result["stereo_valid_on_nonfinite_reference_count"] = int(
        np.count_nonzero(unknown & stereo_valid)
    )
    result["bands"] = {}
    for low, high in [(0.25, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 3.0)]:
        upper = reference <= high if high == 3.0 else reference < high
        mask = evaluated & (reference >= low) & upper
        result["bands"][f"{low:g}-{high:g}"] = _errors(stereo, reference, mask, error_threshold)
    return result


def timing_percentiles(seconds):
    """Summarize caller-provided simulation seconds; report rejected values."""
    values = np.asarray(seconds, dtype=np.float64).reshape(-1)
    finite = np.isfinite(values)
    nonnegative = finite & (values >= 0)
    valid = values[nonnegative]
    result = {
        "sample_count": int(values.size),
        "valid_count": int(valid.size),
        "excluded_negative_count": int(np.count_nonzero(finite & (values < 0))),
        "excluded_nonfinite_count": int(np.count_nonzero(~finite)),
        "mean_s": float(np.mean(valid)) if valid.size else None,
        "min_s": float(np.min(valid)) if valid.size else None,
        "max_s": float(np.max(valid)) if valid.size else None,
    }
    for percentile in [50, 90, 95, 99]:
        result[f"p{percentile}_s"] = (
            float(np.percentile(valid, percentile)) if valid.size else None
        )
    return result
