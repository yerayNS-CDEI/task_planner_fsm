# =====================================================================
# loader_dispatcher.py — Central unified loader for all supported types
# =====================================================================

import os

from .segy_loader import SegyLoader
from .segy_mesh_loader import SegyMeshLoader
from .mala_loader import MalaLoader
from .dzt_loader import DZTLoader
from .gprmax_loader import GPRmaxLoader


# ============================================================
# DETECT LOADER TYPE
# ============================================================
def detect_loader_type(path):
    """
    Decide which loader should be used for a given file path.
    Returns a string in:
      "SEGY"
      "SEGY_MESH"
      "MALA"
      "DZT"
      "GPRMAX"
      "UNKNOWN"
    """
    path = os.path.abspath(path)
    folder = os.path.dirname(path)
    fname = os.path.basename(path).lower()

    # --------------------------------------------------------
    # SEGY or SEGY MESH
    # --------------------------------------------------------
    if fname.endswith(".sgy"):

        # If folder contains >=2 SGY → it's a mesh
        sgys = [f for f in os.listdir(folder) if f.lower().endswith(".sgy")]
        if len(sgys) >= 2:
            return "SEGY_MESH"
        else:
            return "SEGY"

    # --------------------------------------------------------
    # MALA (RAD + RD3)
    # --------------------------------------------------------
    if fname.endswith(".rad"):
        base = os.path.splitext(path)[0]
        if os.path.exists(base + ".rd3"):
            return "MALA"

    # --------------------------------------------------------
    # gprMax
    # --------------------------------------------------------
    if fname.endswith("_merged.out"):
        return "GPRMAX"

    # --------------------------------------------------------
    # DZT
    # --------------------------------------------------------
    if fname.endswith(".dzt"):
        return "DZT"

    return "UNKNOWN"


# ============================================================
# LOAD MATRIX FOR ANY TYPE
# ============================================================
def load_matrix_by_type(path, loader_type):
    """
    Unified entry point.
    Returns:
        M  (n_samples × n_traces)
        list_raw_files  (for mesh: list of sgy paths; for single: [path])
    """

    # =====================================
    # SEGY SINGLE
    # =====================================
    if loader_type == "SEGY":
        M = SegyLoader(path).load_matrix()
        return [(path, M)]  # list of tuples

    # =====================================
    # SEGY MESH
    # =====================================
    if loader_type == "SEGY_MESH":
        folder = os.path.dirname(path)
        mesh = SegyMeshLoader(folder)
        outs = []
        for line_id, sgy in mesh.list_bscans():
            M = SegyLoader(sgy).load_matrix()
            outs.append((sgy, M))
        return outs

    # =====================================
    # MALÅ (RAD+RD3)
    # =====================================
    if loader_type == "MALA":
        base = os.path.splitext(path)[0]
        M = MalaLoader(base).load_matrix()
        return [(base, M)]

    # =====================================
    # gprMax
    # =====================================
    if loader_type == "GPRMAX":
        base = path[:-len("_merged.out")]
        M = GPRmaxLoader(base).load_matrix(component="Ez", rx=1)
        return [(base, M)]

    # =====================================
    # DZT
    # =====================================
    if loader_type == "DZT":
        base = os.path.splitext(path)[0]
        M = DZTLoader(base).load_matrix()
        return [(base, M)]

    raise ValueError(f"Unsupported loader type: {loader_type}")
