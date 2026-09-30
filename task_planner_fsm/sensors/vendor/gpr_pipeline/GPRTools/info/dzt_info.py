# ====================================================================
# dzt_info.py — metadata extraction for DZT (usa SIEMPRE samp_freq)
# ====================================================================

import os
from readgssi import readgssi

def extract_metadata_dzt(path):

    meta = {
        "n_samples": None,
        "n_traces": None,
        "trace_distance": None,   # m
        "time_window": None,      # ns
        "dx": None,               # m
        "dt": None,               # ns
    }

    # Normalizar path
    path = os.path.abspath(path)
    if not path.lower().endswith(".dzt"):
        if os.path.exists(path + ".dzt"):
            path += ".dzt"

    # Leer archivo
    try:
        header, arrays, gps = readgssi.readgssi(
            infile=path,
            frmt="object",
            verbose=False
        )
    except Exception as e:
        print("❌ readgssi error:", e)
        return meta

    # =========================================================
    # (1) n_samples
    # =========================================================
    meta["n_samples"] = header.get("rh_nsamp")

    # =========================================================
    # (2) n_traces desde shape
    # =========================================================
    shape = header.get("shape")
    if isinstance(shape, tuple) and len(shape) == 2:
        meta["n_traces"] = shape[1]

    # =========================================================
    # (3) dx = 1 / dzt_spm
    # =========================================================
    spm = header.get("dzt_spm")
    if spm and spm > 0:
        meta["dx"] = 1.0 / spm

    # =========================================================
    # (4) dt y time_window usando SIEMPRE samp_freq
    # =========================================================
    samp_freq = header.get("samp_freq")
    if samp_freq and samp_freq > 0:

        # dt en segundos
        dt_seconds = 1.0 / float(samp_freq)

        # convertir a ns
        dt_ns = dt_seconds * 1e9
        meta["dt"] = dt_ns

        # time_window = dt_ns * n_samples
        if meta["n_samples"]:
            meta["time_window"] = dt_ns * meta["n_samples"]

    # =========================================================
    # (5) trace_distance
    # =========================================================
    if meta["dx"] and meta["n_traces"] and meta["n_traces"] > 1:
        meta["trace_distance"] = meta["dx"] * (meta["n_traces"] - 1)

    return meta
