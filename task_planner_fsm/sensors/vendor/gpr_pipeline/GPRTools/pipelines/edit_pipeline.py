from GPRTools.edits.background_removal import background_removal
from GPRTools.edits.linear_gain import linear_gain_db
from GPRTools.edits.time_zero import time_zero
from GPRTools.edits.trim import trim
from GPRTools.edits.dewow import dewow
from GPRTools.edits.agc import agc



def apply_edit_pipeline(M, edits):
    """
    Apply sequential edits:
      - background_removal
      - gain
      - time_zero
      - trim (automatically corrected using tz_offset)
    
    edits = [
        ("background_removal", {}),
        ("gain", {"gain_db":20}),
        ("time_zero", {}),
        ("trim", {"t1":20, "t2":120})
    ]
    
    Returns:
        M_out : edited matrix
        applied : list of applied edit descriptions
        tz_offset : total time-zero shift applied
    """

    applied = []
    tz_offset = 0

    for name, params in edits:

        # ---------------------------------------------------------
        # BACKGROUND REMOVAL
        # ---------------------------------------------------------
        if name == "background_removal":
            M = background_removal(M)
            applied.append("background_removal")
            continue
        
        # ---------------------------------------------------------
        # GAIN (linear in dB)
        # ---------------------------------------------------------
        if name == "gain":
            db = params.get("gain_db", 20)
            M = linear_gain_db(M, db)
            applied.append(f"gain({db} dB)")
            continue

        # ---------------------------------------------------------
        # TIME ZERO
        # ---------------------------------------------------------
        if name == "time_zero":
            method = params.get("method", 2)
            threshold = params.get("threshold", 0.2)
            start_sample = params.get("start_sample", 0)
            backup_samples = params.get("backup_samples", 0)

            tz, M = time_zero(
                M,
                method=method,
                threshold=threshold,
                start_sample=start_sample,
                backup_samples=backup_samples,
            )

            tz_offset += tz

            applied.append(
                "time_zero("
                f"method={method}, "
                f"threshold={threshold}, "
                f"start_sample={start_sample}, "
                f"backup_samples={backup_samples}, "
                f"tz_offset={tz}"
                ")"
            )
            continue

        # ---------------------------------------------------------
        # TRIM — corrected using tz_offset
        # ---------------------------------------------------------
        if name == "trim":
            # User-defined trim bounds (original coordinates)
            t1_orig = params["t1"]
            t2_orig = params["t2"]

            # Translate to post-time-zero coordinate system
            t1_new = t1_orig - tz_offset
            t2_new = t2_orig - tz_offset

            # Prevent negative values
            t1_new = max(0, t1_new)
            t2_new = max(0, t2_new)

            # Prevent exceeding matrix length
            t2_new = min(t2_new, M.shape[0])

            applied.append(f"trim(original=({t1_orig},{t2_orig}))")
            applied.append(f"trim(corrected=({t1_new},{t2_new}))")

            M = trim(M, t1_new, t2_new)
            continue

        # ---------------------------------------------------------
        # DEWOW
        # ---------------------------------------------------------
        if name == "dewow":
            w = params.get("window", 50)
            M = dewow(M, window=w)
            applied.append(f"dewow_window={w}")
            continue

        # ---------------------------------------------------------
        # AGC
        # ---------------------------------------------------------
        if name == "agc":
            w = params.get("window", 40)
            M = agc(M, window=w)
            applied.append(f"agc_window={w}")
            continue

        # ---------------------------------------------------------
        # Unknown edit
        # ---------------------------------------------------------
        raise ValueError(f"Unknown edit: {name}")

    return M, applied, tz_offset
