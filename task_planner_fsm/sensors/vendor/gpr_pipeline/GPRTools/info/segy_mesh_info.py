# ================================================================
# info_segy_mesh.py — metadata extraction for SEGY MESH
# ================================================================

import os
import re
from  GPRTools.loaders.segy_loader import SegyLoader  # usamos el loader para obtener M


def extract_metadata_segy_mesh(sgy_path):
    """
    Extract metadata for SEGY MESH files.
    
    Devuelve SIEMPRE:
        - n_samples
        - n_traces
        - trace_distance  [m]
        - time_window     [ns]
        - dx              [m]
        - dt              [ns]

    Pasos (tu lógica + regla genérica):
      1) Encontrar el master CSV en la misma carpeta.
      2) Parsear Time Window [ns] global.
      3) Parsear SCAN DISTANCE [m] por línea.
      4) Identificar la línea L### de este .sgy.
      5) Usar SegyLoader(sgy_path) para obtener M → n_samples, n_traces.
      6) Aplicar regla genérica:
         dt = time_window / n_samples
         dx = trace_distance / n_traces
    """

    # ------------------------------------------------------------
    # 0) Estructura de retorno
    # ------------------------------------------------------------
    meta = {
        "n_samples": None,
        "n_traces": None,
        "trace_distance": None,
        "time_window": None,
        "dx": None,
        "dt": None,
    }

    folder = os.path.dirname(sgy_path)

    # ------------------------------------------------------------
    # 1) Identificar L### de este sgy file
    # ------------------------------------------------------------
    fname = os.path.basename(sgy_path)
    m = re.search(r"_L(\d{3})_", fname, flags=re.IGNORECASE)
    if not m:
        # Mesh files ALWAYS must follow the pattern
        return meta

    line_id = f"L{int(m.group(1)):03d}"  # ejemplo: L004

    # ------------------------------------------------------------
    # 2) Localizar master CSV (no *_info.csv)
    # ------------------------------------------------------------
    master_csv = None
    for f in os.listdir(folder):
        fl = f.lower()
        if fl.endswith(".csv") and not fl.endswith("_info.csv"):
            master_csv = os.path.join(folder, f)
            break

    if master_csv is None or not os.path.exists(master_csv):
        return meta

    # ------------------------------------------------------------
    # 3) Leer CSV (UTF-16 → formato real, fallback utf-8/latin-1)
    # ------------------------------------------------------------
    try_encodings = ["utf-16", "utf-8", "latin-1"]
    lines = None

    for enc in try_encodings:
        try:
            with open(master_csv, "r", encoding=enc) as f:
                lines = [ln.strip() for ln in f]
            break
        except Exception:
            continue

    if lines is None:
        return meta

    # ------------------------------------------------------------
    # 4) Extraer Time Window [ns] (global)
    # ------------------------------------------------------------
    def get_last(label):
        for ln in lines:
            if ln.startswith(label):
                parts = ln.split("\t")
                if len(parts) >= 2:
                    return parts[-1].strip().replace(",", ".")
        return None

    tw = get_last("Time Window [ns]")
    if tw is not None:
        try:
            meta["time_window"] = float(tw)
        except Exception:
            pass

    # ------------------------------------------------------------
    # 5) Extraer Scan Distance [m] para esta línea L###
    # ------------------------------------------------------------
    try:
        idx = lines.index("SCAN DISTANCE [m]")
    except ValueError:
        idx = -1

    if idx >= 0:
        for ln in lines[idx + 1:]:
            if "\t" not in ln:
                continue
            key, val = ln.split("\t", 1)

            # match: "Line 1", "Line 2", ...
            m2 = re.match(r"Line\s+(\d+)", key.strip(), flags=re.IGNORECASE)
            if not m2:
                continue

            n = int(m2.group(1))
            L = f"L{n:03d}"

            if L == line_id:
                try:
                    meta["trace_distance"] = float(val.strip().replace(",", "."))
                except Exception:
                    pass
                break

    # ------------------------------------------------------------
    # 6) NOTE: We intentionally do NOT reload the SEGY here.
    # n_samples/n_traces (and dx/dt) are derived from the already-loaded matrix M in info_writer.
    # ------------------------------------------------------------

    return meta
