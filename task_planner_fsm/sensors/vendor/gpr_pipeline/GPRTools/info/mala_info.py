# ================================================================
# info_mala.py — metadata extraction for MALÅ RAD/RD3 (FINAL)
# ================================================================

import os
from GPRTools.loaders.mala_loader import MalaLoader  # usamos el loader para obtener M


def extract_metadata_mala(rad_or_rd3_path):
    """
    Devuelve SIEMPRE:
        - n_samples
        - n_traces
        - trace_distance  [m]
        - time_window     [ns]
        - dx              [m]
        - dt              [ns]

    Lógica:
        - Leer .rad para:
            trace_distance  ← STOP POSITION
            time_window     ← TIMEWINDOW
        - Usar MalaLoader para cargar M y sacar:
            n_samples = M.shape[0]
            n_traces  = M.shape[1]
        - Luego aplicar tu regla genérica:
            dt = time_window / n_samples
            dx = trace_distance / n_traces
    """

    base = os.path.splitext(rad_or_rd3_path)[0]
    rad_file = base + ".rad"

    meta = {
        "n_samples": None,
        "n_traces": None,
        "trace_distance": None,
        "time_window": None,
        "dx": None,
        "dt": None,
    }

    if not os.path.exists(rad_file):
        return meta

    raw = {}

    # ------------------------------------------------------------
    # Leer .rad (limpiando BOM y espacios)
    # ------------------------------------------------------------
    try:
        with open(rad_file, "r", encoding="utf-8-sig") as f:
            for ln in f:
                if ":" in ln:
                    key, value = ln.split(":", 1)

                    key = key.strip().upper().replace("\ufeff", "")
                    value = value.strip().replace(",", ".")

                    raw[key] = value
    except Exception:
        return meta

    # ------------------------------------------------------------
    # STOP POSITION → trace_distance [m]
    # ------------------------------------------------------------
    stop_key = "STOP POSITION"
    if stop_key in raw:
        try:
            meta["trace_distance"] = float(raw[stop_key])
        except Exception:
            pass

    # ------------------------------------------------------------
    # TIMEWINDOW → time_window [ns]
    # ------------------------------------------------------------
    tw_key = "TIMEWINDOW"
    if tw_key in raw:
        try:
            meta["time_window"] = float(raw[tw_key])
        except Exception:
            pass

    # ------------------------------------------------------------
    # Usar MalaLoader para obtener M y sacar n_samples, n_traces
    # ------------------------------------------------------------
    try:
        loader = MalaLoader(base)
        M = loader.load_matrix()  # (n_samples, n_traces)
        if M is not None and M.ndim == 2:
            n_samples, n_traces = M.shape
            meta["n_samples"] = n_samples
            meta["n_traces"] = n_traces
    except Exception as e:
        print(f"⚠ Error cargando matriz MALÅ: {e}")
        # seguimos, pero n_samples / n_traces se quedarán en None si falla

    # ------------------------------------------------------------
    # Regla GENÉRICA:
    #   dt = time_window / n_samples
    #   dx = trace_distance / n_traces
    # ------------------------------------------------------------
    if meta["time_window"] is not None and meta["n_samples"]:
        meta["dt"] = meta["time_window"] / meta["n_samples"]

    if meta["trace_distance"] is not None and meta["n_traces"]:
        meta["dx"] = meta["trace_distance"] / meta["n_traces"]

    return meta
