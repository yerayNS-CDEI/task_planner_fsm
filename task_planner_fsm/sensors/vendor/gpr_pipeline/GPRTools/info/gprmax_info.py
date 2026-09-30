# ================================================================
# info_gprmax.py — metadata extraction for gprMax (FINAL)
# ================================================================

import os
import re

from  GPRTools.loaders.gprmax_loader import GPRmaxLoader  # usamos el loader oficial


def extract_metadata_gprmax(out_or_base_path, component="Ez", rx=1):
    """
    Extract metadata for gprMax:
        - n_samples, n_traces
        - trace_distance  [m]
        - time_window     [ns]
        - dx              [m]
        - dt              [ns]

    Lógica original (la respetamos):
        - Leer <base>_merged.out (HDF5) usando GPRmaxLoader
        - n_samples, n_traces desde M.shape
        - dt (segundos) desde atributo HDF5 → time_window = dt * n_samples * 1e9
        - Leer .in y extraer dx_step de "current_model_run * X"
        - trace_distance = dx_step * (n_traces - 1)

    Y luego, de forma genérica (TU REGLA):
        dt = time_window / n_samples
        dx = trace_distance / n_traces
    """

    # ------------------------------------------------------------
    # Detect base path
    # ------------------------------------------------------------
    path = os.path.abspath(out_or_base_path)

    # detect_loader_type ya pasa algo tipo "..._merged.out"
    # así que quitamos el sufijo para obtener la base
    if path.endswith("_merged.out"):
        base = path[:-len("_merged.out")]
    else:
        base = path  # por si acaso llaman con la base directamente

    out_file = base + "_merged.out"
    in_file = base + ".in"

    meta = {
        "n_samples": None,
        "n_traces": None,
        "trace_distance": None,
        "time_window": None,
        "dx": None,
        "dt": None,
    }

    # ------------------------------------------------------------
    # 1) Usar el loader para obtener M (n_samples x n_traces)
    # ------------------------------------------------------------
    try:
        loader = GPRmaxLoader(base)
        M = loader.load_matrix(component=component, rx=rx)
    except Exception as e:
        print(f"❌ Error cargando matriz gprMax: {e}")
        return meta

    # n_samples, n_traces desde shape de M
    if M is not None and M.ndim == 2:
        n_samples, n_traces = M.shape
        meta["n_samples"] = n_samples
        meta["n_traces"] = n_traces
    else:
        # si algo raro ocurre, salimos
        return meta

    # ------------------------------------------------------------
    # 2) Obtener dt (segundos) desde el loader y calcular time_window [ns]
    # ------------------------------------------------------------
    dt_seconds = None
    try:
        dt_seconds = loader.get_dt()  # dt en segundos (atributo HDF5)
    except Exception as e:
        print(f"⚠ No se pudo leer dt de gprMax: {e}")

    if dt_seconds is not None and n_samples is not None:
        # time_window en ns (como ya hacías)
        time_window_ns = dt_seconds * n_samples * 1e9
        meta["time_window"] = time_window_ns

    # ------------------------------------------------------------
    # 3) Leer .in y extraer dx_step (lógica original)
    # ------------------------------------------------------------
    dx_step = None

    if os.path.exists(in_file):
        try:
            with open(in_file, "r", encoding="utf-8") as fin:
                txt = fin.read()

            # Ejemplo: current_model_run * 0.005
            m = re.search(r"current_model_run\s*\*\s*([0-9.+\-eE]+)", txt)
            if m:
                dx_step = float(m.group(1))
        except Exception as e:
            print(f"⚠ No se pudo extraer dx_step de {in_file}: {e}")

    # ------------------------------------------------------------
    # 4) Calcular trace_distance según tu lógica original
    #     trace_distance = dx_step * (n_traces - 1)
    # ------------------------------------------------------------
    if dx_step is not None and n_traces is not None and n_traces > 1:
        meta["trace_distance"] = dx_step * (n_traces - 1)

    # ------------------------------------------------------------
    # 5) Calcular dt y dx de forma GENÉRICA (TU REGLA)
    #    dt = time_window / n_samples
    #    dx = trace_distance / n_traces
    # ------------------------------------------------------------
    if meta["time_window"] is not None and meta["n_samples"]:
        # dt en ns (coherente con time_window en ns)
        meta["dt"] = meta["time_window"] / meta["n_samples"]

    if meta["trace_distance"] is not None and meta["n_traces"]:
        meta["dx"] = meta["trace_distance"] / meta["n_traces"]

    return meta
