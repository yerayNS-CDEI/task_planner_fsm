# ================================================================
# info_segy.py — metadata extraction for SEGY SINGLE (non-mesh)
# ================================================================

import os
from GPRTools.loaders.segy_loader import SegyLoader  # usamos el loader para obtener M


def extract_metadata_segy_single(sgy_path):
    """
    Extract metadata required by info_writer:
        - n_samples
        - n_traces
        - trace_distance  [m]
        - time_window     [ns]
        - dx              [m]
        - dt              [ns]

    Logic:
        - Read associated CSV file:
            n_samples      <- "Scan Length [samples / scan]"
            trace_distance <- "SCAN DISTANCE [m]" value from next line
            time_window    <- "Time Window [ns]"
            repetition     <- "Repetition Rate [scans /cm]"

        - Compute:
            n_traces = trace_distance * 100 * repetition_rate
            dt = time_window / n_samples
            dx = trace_distance / n_traces
    """

    base = os.path.splitext(sgy_path)[0] # guarda solo la ruta sin extensión .sgy 
    csv_path = base + ".csv"

    meta = {
        "n_samples": None,
        "n_traces": None,
        "trace_distance": None,
        "time_window": None,
        "dx": None,
        "dt": None,
    }

    if not os.path.exists(csv_path):
        return meta

    # Read CSV with possible encodings
    lines = None
    for enc in ["utf-16", "utf-8", "latin-1"]:
        try:
            with open(csv_path, "r", encoding=enc) as f:
                lines = [ln.strip() for ln in f if ln.strip()]
            break
        except Exception:
            pass

    if lines is None:
        return meta

    repetition_rate = None

    # Extract metadata from CSV
    for i, line in enumerate(lines):
        parts = line.split("\t")
        value = parts[-1].strip().replace(",", ".") if len(parts) >= 2 else None

        try:
            if line.startswith("Scan Length [samples / scan]") and value is not None:
                meta["n_samples"] = int(float(value))

            elif line.startswith("Time Window [ns]") and value is not None:
                meta["time_window"] = float(value) - 4

            elif line.startswith("Repetition Rate [scans /cm]") and value is not None:
                repetition_rate = float(value)

            elif line.startswith("SCAN DISTANCE") and i + 1 < len(lines):
                next_parts = lines[i + 1].split("\t")
                if len(next_parts) >= 2:
                    dist_value = next_parts[1].strip().replace(",", ".")
                    meta["trace_distance"] = float(dist_value)

        except Exception:
            pass

    # Compute derived metadata
    if meta["trace_distance"] is not None and repetition_rate is not None:
        meta["n_traces"] = int(round(meta["trace_distance"] * 100.0 * repetition_rate))

    if meta["time_window"] is not None and meta["n_samples"]:
        meta["dt"] = meta["time_window"] / meta["n_samples"]

    if meta["trace_distance"] is not None and meta["n_traces"]:
        meta["dx"] = meta["trace_distance"] / meta["n_traces"]

    return meta