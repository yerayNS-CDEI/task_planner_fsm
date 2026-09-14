import os
import re
import numpy as np

# Dispatcher for loaders
from GPRTools.loaders.loader_dispatcher import (
    detect_loader_type,
    load_matrix_by_type,
)
from GPRTools.loaders.segy_loader import SegyLoader

# Info system
from GPRTools.info.info_dispatcher import extract_metadata
from GPRTools.info.info_writer import write_info_file

# Edit pipeline
from GPRTools.pipelines.edit_pipeline import apply_edit_pipeline

# Plot
from GPRTools.plotters.plot_bscan import plot_bscan


# ====================================================================
# Detect type of edit for filename labeling
# ====================================================================
def _detect_edit_type(applied_edits):
    if len(applied_edits) == 0:
        return "original"
    if len(applied_edits) == 1 and applied_edits[0].startswith("dewow"):
        return "dewow"
    if applied_edits == ["background_removal"]:
        return "background_removal"
    if len(applied_edits) == 1 and applied_edits[0].startswith("gain"):
        return "gain"
    if len(applied_edits) == 1 and applied_edits[0].startswith("time_zero"):
        return "time_zero"
    if len(applied_edits) == 1 and applied_edits[0].startswith("trim"):
        return "trim"
    if len(applied_edits) == 1 and applied_edits[0].startswith("agc"):
        return "agc"
    return "combined"


# ====================================================================
# Flat output paths
# ====================================================================
def _ensure_flat_output_dirs(output_base):
    plots_dir = os.path.join(output_base, "plots")
    info_dir = os.path.join(output_base, "info")
    matrices_dir = os.path.join(output_base, "matrices")

    os.makedirs(plots_dir, exist_ok=True)
    os.makedirs(info_dir, exist_ok=True)
    os.makedirs(matrices_dir, exist_ok=True)

    return plots_dir, info_dir, matrices_dir


# ====================================================================
# Name helpers
# ====================================================================
def _strip_known_suffixes(name):
    if "." in name:
        name = name.rsplit(".", 1)[0]

    if name.lower().endswith("_merged"):
        name = name[:-len("_merged")]

    return name


def _default_raw_name(raw_path, *, loader_type, use_mesh_batch_naming):
    raw_name = _strip_known_suffixes(os.path.basename(raw_path))

    if loader_type == "SEGY_MESH" and use_mesh_batch_naming:
        folder = os.path.basename(os.path.dirname(raw_path))
        m = re.search(r"_L(\d{3})_", raw_name, flags=re.IGNORECASE)
        line_id = f"L{m.group(1)}" if m else "LUNK"
        raw_name = f"{folder}_{line_id}"

    return raw_name


# ====================================================================
# Single-file loader
# ====================================================================
def _load_single_bscan(file_path, loader_type):
    """
    Load exactly one B-scan.

    Important detail:
    detect_loader_type(...) returns SEGY_MESH when a folder contains
    several .sgy files. For single-file processing we still load only
    the selected .sgy file, not the whole mesh folder.
    """
    if loader_type == "SEGY_MESH":
        M = SegyLoader(file_path).load_matrix()
        return file_path, M

    entries = load_matrix_by_type(file_path, loader_type)

    if not entries:
        raise ValueError(f"No B-scan matrix could be loaded from: {file_path}")

    if len(entries) > 1:
        print(
            "[WARNING] More than one B-scan was returned by the loader. "
            "Only the first one will be processed."
        )

    return entries[0]


# ====================================================================
# Single B-scan processor - flat output
# ====================================================================
def process_single_bscan_flat(
    file_path,
    *,
    do_original_plot=True,
    do_original_matrix=True,
    do_info=False,
    do_dewow_plot=False, do_dewow_matrix=False, dewow_window=50,
    do_background_removal_plot=False, do_background_removal_matrix=False,
    do_gain_plot=False, do_gain_matrix=False, linear_gain_db=20,
    do_time_zero_plot=False, do_time_zero_matrix=False,
    do_trim_plot=False, do_trim_matrix=False, trim_bounds=(0, 0),
    do_agc_plot=False, do_agc_matrix=False, agc_window=40,
    do_combine_plot=False, do_combine_matrix=False, combine_edits=None,
    plot_scale=2, aspect_ratio=(4, 2), plot_colormap="gray", plot_axes=False,
    plot_axes_in_meters=True, plot_axes_in_ns=True,
    overwrite=False,
    output_dir=None,
    raw_name=None,
    use_mesh_batch_naming=True,
    **kwargs,
):
    """
    Process one B-scan file and save plots, info, and/or amplitude matrices.

    This is the single-file equivalent of batch_process_bscans_flat(...).
    It supports the same preprocessing flags, but receives a single file_path
    instead of scanning a whole root_dir.

    Parameters
    ----------
    file_path : str
        Path to one input B-scan file. Supported inputs follow the existing
        GPRTools loaders: .sgy, .rad, .dzt, and *_merged.out.
    output_dir : str or None
        Folder where plots/, info/, and matrices/ will be created. If None,
        outputs are written next to the input file.
    raw_name : str or None
        Optional output base name. If None, the name is derived automatically
        from the input file.
    use_mesh_batch_naming : bool
        If True and the file belongs to a SEGY mesh folder, output names follow
        the batch convention: <folder>_L###. If False, the original file name is
        used instead.
    """
    file_path = os.path.abspath(file_path)

    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Input B-scan file does not exist: {file_path}")

    loader_type = detect_loader_type(file_path)
    if loader_type == "UNKNOWN":
        raise ValueError(f"Unsupported or unknown B-scan format: {file_path}")

    if combine_edits is None:
        combine_edits = []

    output_base = os.path.abspath(output_dir) if output_dir is not None else os.path.dirname(file_path)

    raw_path, M_original = _load_single_bscan(file_path, loader_type)
    raw_path = os.path.abspath(raw_path)

    if raw_name is None:
        raw_name = _default_raw_name(
            raw_path,
            loader_type=loader_type,
            use_mesh_batch_naming=use_mesh_batch_naming,
        )

    plots_dir, info_dir, matrices_dir = _ensure_flat_output_dirs(output_base)

    # Keep the real input path for metadata/axes, even when output_dir is elsewhere.
    src_file_path = raw_path

    summary = []

    print(f"\n[INFO] Processing {raw_name} (type={loader_type})")

    # ============================================================
    # INFO TXT
    # ============================================================
    if do_info:
        metadata = extract_metadata(
            file_path=src_file_path,
            loader_type=loader_type,
        )

        write_info_file(
            output_dir=info_dir,
            base_name=raw_name,
            M=M_original,
            trace_distance=metadata.get("trace_distance"),
            time_window=metadata.get("time_window"),
            dx=metadata.get("dx"),
            dt=metadata.get("dt"),
            n_samples_header=metadata.get("n_samples"),
            n_traces_header=metadata.get("n_traces"),
        )

    # ============================================================
    # 0. ORIGINAL
    # ============================================================
    if do_original_matrix:
        matrix_path = os.path.join(matrices_dir, f"{raw_name}_original.csv")
        np.savetxt(matrix_path, M_original, delimiter=",")

    if do_original_plot:
        plot_bscan(
            self=None,
            M=M_original,
            applied_edits=[],
            base_name=f"{raw_name}_original.png",
            base_dir=plots_dir,
            save=True,
            plot_scale=plot_scale,
            aspect_ratio=aspect_ratio,
            plot_colormap=plot_colormap,
            plot_axes=plot_axes,
            plot_axes_in_meters=plot_axes_in_meters,
            plot_axes_in_ns=plot_axes_in_ns,
            overwrite=overwrite,
            loader_type=loader_type,
            src_file_path=src_file_path,
        )

    # ============================================================
    # 1. INDIVIDUAL EDITS
    # ============================================================
    individual_edits = []

    if do_dewow_plot or do_dewow_matrix:
        individual_edits.append(("dewow", {"window": dewow_window}))

    if do_background_removal_plot or do_background_removal_matrix:
        individual_edits.append(("background_removal", {}))

    if do_gain_plot or do_gain_matrix:
        individual_edits.append(("gain", {"gain_db": linear_gain_db}))

    if do_time_zero_plot or do_time_zero_matrix:
        individual_edits.append(("time_zero", {}))

    if do_trim_plot or do_trim_matrix:
        t1, t2 = trim_bounds
        individual_edits.append(("trim", {"t1": t1, "t2": t2}))

    if do_agc_plot or do_agc_matrix:
        individual_edits.append(("agc", {"window": agc_window}))

    applied_edits_last = []

    for name, params in individual_edits:
        M_edit, applied_edits, tz_offset = apply_edit_pipeline(
            M_original.copy(),
            [(name, params)],
        )
        applied_edits_last = applied_edits
        edit_type = _detect_edit_type(applied_edits)

        if (
            (name == "dewow" and do_dewow_matrix) or
            (name == "background_removal" and do_background_removal_matrix) or
            (name == "gain" and do_gain_matrix) or
            (name == "time_zero" and do_time_zero_matrix) or
            (name == "trim" and do_trim_matrix) or
            (name == "agc" and do_agc_matrix)
        ):
            matrix_path = os.path.join(matrices_dir, f"{raw_name}_{edit_type}.csv")
            np.savetxt(matrix_path, M_edit, delimiter=",")

        if (
            (name == "dewow" and do_dewow_plot) or
            (name == "background_removal" and do_background_removal_plot) or
            (name == "gain" and do_gain_plot) or
            (name == "time_zero" and do_time_zero_plot) or
            (name == "trim" and do_trim_plot) or
            (name == "agc" and do_agc_plot)
        ):
            plot_bscan(
                self=None,
                M=M_edit,
                applied_edits=applied_edits,
                base_name=f"{raw_name}_{edit_type}.png",
                base_dir=plots_dir,
                save=True,
                plot_scale=plot_scale,
                aspect_ratio=aspect_ratio,
                plot_colormap=plot_colormap,
                plot_axes=plot_axes,
                plot_axes_in_meters=plot_axes_in_meters,
                plot_axes_in_ns=plot_axes_in_ns,
                overwrite=overwrite,
                loader_type=loader_type,
                src_file_path=src_file_path,
            )

    # ============================================================
    # 2. COMBINED
    # ============================================================
    if do_combine_plot or do_combine_matrix:
        M_combined, applied_edits_combined, tz_offset = apply_edit_pipeline(
            M_original.copy(),
            combine_edits,
        )

        if do_combine_matrix:
            matrix_path = os.path.join(matrices_dir, f"{raw_name}_combined.csv")
            np.savetxt(matrix_path, M_combined, delimiter=",")

        if do_combine_plot:
            plot_bscan(
                self=None,
                M=M_combined,
                applied_edits=applied_edits_combined,
                base_name=f"{raw_name}_combined.png",
                base_dir=plots_dir,
                save=True,
                plot_scale=plot_scale,
                aspect_ratio=aspect_ratio,
                plot_colormap=plot_colormap,
                plot_axes=plot_axes,
                plot_axes_in_meters=plot_axes_in_meters,
                plot_axes_in_ns=plot_axes_in_ns,
                overwrite=overwrite,
                loader_type=loader_type,
                src_file_path=src_file_path,
            )

    summary.append((raw_name, applied_edits_last))

    print("\n========== SUMMARY ==========")
    print(f"Input file                    : {file_path}")
    print(f"Flat output directory         : {output_base}")
    print(f"Successfully processed B-scans: {len(summary)}")
    print("\n--- PROCESSED B-SCAN ---")
    for fname, edits in summary:
        print(f"{fname}: {edits}")
    print("================================\n")

    return summary


# Short alias with the same meaning.
single_process_bscan_flat = process_single_bscan_flat
