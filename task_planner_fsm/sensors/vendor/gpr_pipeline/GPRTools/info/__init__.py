"""Metadata helpers.

SEGY helpers stay available without forcing optional dependencies for every
other GPR format to be installed.
"""

from .segy_info import extract_metadata_segy_single
from .info_writer import write_info_file

try:
    from .segy_mesh_info import extract_metadata_segy_mesh
except (ImportError, ModuleNotFoundError):
    extract_metadata_segy_mesh = None

try:
    from .mala_info import extract_metadata_mala
except (ImportError, ModuleNotFoundError):
    extract_metadata_mala = None

try:
    from .gprmax_info import extract_metadata_gprmax
except (ImportError, ModuleNotFoundError):
    extract_metadata_gprmax = None

try:
    from .dzt_info import extract_metadata_dzt
except (ImportError, ModuleNotFoundError):
    extract_metadata_dzt = None

# Dispatcher can pull optional formats. Keep it optional at package import time.
try:
    from .info_dispatcher import extract_metadata
except (ImportError, ModuleNotFoundError):
    extract_metadata = None

__all__ = [
    "extract_metadata_segy_single", "write_info_file", "extract_metadata",
    "extract_metadata_segy_mesh", "extract_metadata_mala", "extract_metadata_gprmax",
    "extract_metadata_dzt",
]
