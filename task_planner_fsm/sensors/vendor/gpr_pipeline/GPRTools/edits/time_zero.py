import numpy as np


def time_zero(
    M,
    method=2,
    threshold=0.2,
    start_sample=0,
    backup_samples=0,
):
    """
    Estimate time-zero and crop the B-scan from that index.

    Parameters
    ----------
    M : array-like
        B-scan matrix with shape (n_samples, n_traces).

    method : int or str
        Time-zero detection method:

        1 or "threshold":
            Threshold breach - backup_samples.
            Finds the first sample whose mean absolute amplitude exceeds
            threshold * max(profile).

        2 or "peak":
            Peak response - backup_samples.
            Finds the maximum mean absolute amplitude.

        3 or "zero_crossing":
            Zero crossing - backup_samples.
            Finds the first zero crossing of the mean signed trace after
            the signal first exceeds the threshold.

    threshold : float
        Relative threshold. Example:
            0.2  -> 20% of maximum response
            0.05 -> 5% of maximum response

    start_sample : int
        First sample where the algorithm starts searching.

    backup_samples : int
        Number of samples to move upward before cropping.

    Returns
    -------
    tz : int
        Detected/corrected time-zero index.

    M_crop : ndarray
        Cropped matrix M[tz:, :].
    """

    M = np.asarray(M)

    if M.ndim != 2:
        raise ValueError("time_zero expects a 2D matrix with shape (n_samples, n_traces).")

    n_samples, _ = M.shape

    start_sample = int(start_sample)
    backup_samples = int(backup_samples)

    start_sample = max(0, min(start_sample, n_samples - 1))
    backup_samples = max(0, backup_samples)

    # Mean absolute amplitude profile across traces.
    # This is robust for global line-by-line time-zero estimation.
    profile = np.mean(np.abs(M), axis=1)

    search_profile = profile[start_sample:]

    if search_profile.size == 0:
        tz = 0
        return tz, M

    method_key = _normalize_time_zero_method(method)

    if method_key == "threshold":
        tz_detected = _detect_threshold_breach(
            profile=profile,
            start_sample=start_sample,
            threshold=threshold,
        )

    elif method_key == "peak":
        tz_detected = _detect_peak_response(
            profile=profile,
            start_sample=start_sample,
        )

    elif method_key == "zero_crossing":
        tz_detected = _detect_zero_crossing(
            M=M,
            profile=profile,
            start_sample=start_sample,
            threshold=threshold,
        )

    else:
        raise ValueError(f"Unknown time-zero method: {method}")

    tz = max(0, int(tz_detected) - backup_samples)
    tz = min(tz, n_samples - 1)

    return tz, M[tz:, :]


def _normalize_time_zero_method(method):
    if isinstance(method, str):
        method = method.lower().strip()

        aliases = {
            "1": "threshold",
            "threshold": "threshold",
            "threshold_breach": "threshold",

            "2": "peak",
            "peak": "peak",
            "peak_response": "peak",

            "3": "zero_crossing",
            "zero": "zero_crossing",
            "zero_crossing": "zero_crossing",
            "zero-crossing": "zero_crossing",
        }

        if method in aliases:
            return aliases[method]

    if method == 1:
        return "threshold"

    if method == 2:
        return "peak"

    if method == 3:
        return "zero_crossing"

    raise ValueError(f"Unknown time-zero method: {method}")


def _detect_threshold_breach(profile, start_sample, threshold):
    search_profile = profile[start_sample:]

    max_value = np.max(search_profile)

    if max_value <= 0:
        return start_sample

    threshold_value = float(threshold) * max_value

    candidates = np.where(search_profile >= threshold_value)[0]

    if candidates.size == 0:
        return int(start_sample + np.argmax(search_profile))

    return int(start_sample + candidates[0])


def _detect_peak_response(profile, start_sample):
    search_profile = profile[start_sample:]

    if search_profile.size == 0:
        return start_sample

    return int(start_sample + np.argmax(search_profile))


def _detect_zero_crossing(M, profile, start_sample, threshold):
    """
    Detect the first zero crossing after the signal becomes active.

    We first find an activation point using the same threshold logic on
    mean absolute amplitude. Then we search for the first sign change in
    the mean signed trace.
    """

    n_samples, _ = M.shape

    activation = _detect_threshold_breach(
        profile=profile,
        start_sample=start_sample,
        threshold=threshold,
    )

    mean_trace = np.mean(M, axis=1)

    # Avoid very early noise by starting at activation.
    search = mean_trace[activation:]

    if search.size < 2:
        return activation

    signs = np.sign(search)

    # Replace exact zeros with previous non-zero sign where possible.
    # This avoids missing a crossing because of exact zero samples.
    for i in range(1, len(signs)):
        if signs[i] == 0:
            signs[i] = signs[i - 1]

    crossing_candidates = np.where(signs[:-1] * signs[1:] < 0)[0]

    if crossing_candidates.size == 0:
        # Fallback: use activation if no zero crossing is found.
        return activation

    return int(activation + crossing_candidates[0] + 1)