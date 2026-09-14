"""Loaders for different GPR data formats.

Optional format-specific dependencies are imported defensively so using the
SEGY loader does not require installing unrelated DZT/GPRMax packages.
"""

from .segy_loader import SegyLoader
from .segy_mesh_loader import SegyMeshLoader
from .mala_loader import MalaLoader

try:
    from .gprmax_loader import GPRmaxLoader
except (ImportError, ModuleNotFoundError):  # optional dependency
    GPRmaxLoader = None

try:
    from .dzt_loader import DZTLoader
except (ImportError, ModuleNotFoundError):  # optional readgssi dependency
    DZTLoader = None

__all__ = ["SegyLoader", "SegyMeshLoader", "MalaLoader", "GPRmaxLoader", "DZTLoader"]
