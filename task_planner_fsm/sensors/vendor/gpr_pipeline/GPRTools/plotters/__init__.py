"""Plot utilities for GPR B-scans.

Core B-scan plotting is always available. Optional circular/depth utilities are
loaded only when their dependency chain is available.
"""

from .plot_bscan import plot_bscan, get_output_folder

try:
    from .plot_circular_depths import (
        plot_circular_bscan,
        plot_circular_bscans_for_depths,
        preview_bscan_for_circular_selection,
    )
except (ImportError, ModuleNotFoundError):
    plot_circular_bscan = None
    plot_circular_bscans_for_depths = None
    preview_bscan_for_circular_selection = None

__all__ = [
    "plot_bscan", "get_output_folder", "plot_circular_bscan",
    "plot_circular_bscans_for_depths", "preview_bscan_for_circular_selection",
]
