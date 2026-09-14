import os
import re
import glob
import numpy as np
from pathlib import Path

# Dispatcher para loaders
from GPRTools.loaders.loader_dispatcher import (
    detect_loader_type,
    load_matrix_by_type
)

# Info system
from GPRTools.info.info_dispatcher import extract_metadata
from GPRTools.info.info_writer import write_info_file

# Edit pipeline
from GPRTools.pipelines.edit_pipeline import apply_edit_pipeline

# Plot
from GPRTools.plotters.plot_bscan import plot_bscan


#Creating the output dir
def _make_output_path(output_dir, scan_name, suffix, kind=None):
    if output_dir is None:
        return None

    out_dir = output_dir if kind is None else os.path.join(output_dir, kind)
    os.makedirs(out_dir, exist_ok=True)

    return os.path.join(out_dir, f"{scan_name}_{suffix}")


# ====================================================================
# Detect type of edit for folder placement
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
# UNIVERSAL BSCAN PROCESSOR
# ====================================================================
def process_bscan(
    M_original,
    output_base,
    raw_name,
    *,
    loader_type,
    do_original_plot,
    do_original_matrix,
    do_info,
    do_dewow_plot, do_dewow_matrix, dewow_window,
    do_background_removal_plot, do_background_removal_matrix,
    do_gain_plot, do_gain_matrix, linear_gain_db,
    do_time_zero_plot, do_time_zero_matrix,
    do_trim_plot, do_trim_matrix, trim_bounds,
    do_agc_plot, do_agc_matrix, agc_window,
    do_combine_plot, do_combine_matrix, combine_edits,
    plot_scale, aspect_ratio, plot_colormap, plot_axes,
    plot_axes_in_meters, plot_axes_in_ns,
    overwrite,
    summary
):

    out_filename = f"{raw_name}.csv"
    out_pngname = f"{raw_name}.png"

    # Base path used by plotters/info when they need a "source file path"
    # Convention: path without extension
    raw_path_base = os.path.join(output_base, raw_name)

    # ============================================================
    # INFO TXT
    # ============================================================
    if do_info:
        info_dir = os.path.join(output_base, "info")
        os.makedirs(info_dir, exist_ok=True)

        metadata = extract_metadata(
            file_path=raw_path_base,
            loader_type=loader_type
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
            n_traces_header=metadata.get("n_traces")
        )

    # ============================================================
    # 0. ORIGINAL
    # ============================================================
    if do_original_matrix:
        mdir = os.path.join(output_base, "matrices", "original")
        os.makedirs(mdir, exist_ok=True)
        np.savetxt(os.path.join(mdir, out_filename), M_original, delimiter=',')

    if do_original_plot:
        pdir = os.path.join(output_base, "plots", "original")
        os.makedirs(pdir, exist_ok=True)

        applied_edits_original = []

        plot_bscan(
            self=None,
            M=M_original,
            applied_edits=applied_edits_original,
            base_name=out_pngname,
            base_dir=pdir,
            save=True,
            plot_scale=plot_scale,
            aspect_ratio=aspect_ratio,
            plot_colormap=plot_colormap,
            plot_axes=plot_axes,
            plot_axes_in_meters=plot_axes_in_meters,
            plot_axes_in_ns=plot_axes_in_ns,
            overwrite=overwrite,
            loader_type=loader_type,
            src_file_path=raw_path_base,
        )

    # ============================================================
    # 1. INDIVIDUAL EDITS
    # ============================================================
    INDIVIDUAL_EDITS = []

    if do_dewow_plot or do_dewow_matrix:
        INDIVIDUAL_EDITS.append(("dewow", {"window": dewow_window}))

    if do_background_removal_plot or do_background_removal_matrix:
        INDIVIDUAL_EDITS.append(("background_removal", {}))

    if do_gain_plot or do_gain_matrix:
        INDIVIDUAL_EDITS.append(("gain", {"gain_db": linear_gain_db}))

    if do_time_zero_plot or do_time_zero_matrix:
        INDIVIDUAL_EDITS.append(("time_zero", {}))

    if do_trim_plot or do_trim_matrix:
        t1, t2 = trim_bounds
        INDIVIDUAL_EDITS.append(("trim", {"t1": t1, "t2": t2}))

    if do_agc_plot or do_agc_matrix:
        INDIVIDUAL_EDITS.append(("agc", {"window": agc_window}))

    applied_edits_last = []

    for name, params in INDIVIDUAL_EDITS:
        M_edit, applied_edits, tz_offset = apply_edit_pipeline(
            M_original.copy(),
            [(name, params)]
        )
        applied_edits_last = applied_edits
        edit_type = _detect_edit_type(applied_edits)

        # SAVE MATRIX
        if (
            (name == "dewow" and do_dewow_matrix) or
            (name == "background_removal" and do_background_removal_matrix) or
            (name == "gain" and do_gain_matrix) or
            (name == "time_zero" and do_time_zero_matrix) or
            (name == "trim" and do_trim_matrix) or
            (name == "agc" and do_agc_matrix)
        ):
            mdir = os.path.join(output_base, "matrices", edit_type)
            os.makedirs(mdir, exist_ok=True)
            np.savetxt(os.path.join(mdir, out_filename), M_edit, delimiter=',')

        # SAVE PLOT
        if (
            (name == "dewow" and do_dewow_plot) or
            (name == "background_removal" and do_background_removal_plot) or
            (name == "gain" and do_gain_plot) or
            (name == "time_zero" and do_time_zero_plot) or
            (name == "trim" and do_trim_plot) or
            (name == "agc" and do_agc_plot)
        ):
            pdir = os.path.join(output_base, "plots", edit_type)
            os.makedirs(pdir, exist_ok=True)
            plot_bscan(
                self=None,
                M=M_edit,
                applied_edits=applied_edits,
                base_name=out_pngname,
                base_dir=pdir,
                save=True,
                plot_scale=plot_scale,
                aspect_ratio=aspect_ratio,
                plot_colormap=plot_colormap,
                plot_axes=plot_axes,
                plot_axes_in_meters=plot_axes_in_meters,
                plot_axes_in_ns=plot_axes_in_ns,
                overwrite=overwrite,
                loader_type=loader_type,
                src_file_path=raw_path_base,
            )

    # ============================================================
    # 2. COMBINED
    # ============================================================
    if do_combine_plot or do_combine_matrix:
        M_combined, applied_edits_combined, tz_offset = apply_edit_pipeline(
            M_original.copy(),
            combine_edits
        )

        if do_combine_matrix:
            mdir = os.path.join(output_base, "matrices", "combined")
            os.makedirs(mdir, exist_ok=True)
            np.savetxt(os.path.join(mdir, out_filename), M_combined, delimiter=',')

        if do_combine_plot:
            pdir = os.path.join(output_base, "plots", "combined")
            os.makedirs(pdir, exist_ok=True)
            plot_bscan(
                self=None,
                M=M_combined,
                applied_edits=applied_edits_combined,
                base_name=out_pngname,
                base_dir=pdir,
                save=True,
                plot_scale=plot_scale,
                aspect_ratio=aspect_ratio,
                plot_colormap=plot_colormap,
                plot_axes=plot_axes,
                plot_axes_in_meters=plot_axes_in_meters,
                plot_axes_in_ns=plot_axes_in_ns,
                overwrite=overwrite,
                loader_type=loader_type,
                src_file_path=raw_path_base,
            )

    summary.append((f"{output_base}/{raw_name}", applied_edits_last))


# ====================================================================
# MAIN BATCH — MULTI-FORMAT (USING LOAD DISPATCHER)
# ====================================================================
def batch_process_bscans(
    root_dir,
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
    plot_scale=2, aspect_ratio=None, plot_colormap="gray", plot_axes=False,
    plot_axes_in_meters=True, plot_axes_in_ns=True,
    overwrite=False,
    output_dir=None,
    **kwargs
):
    root_dir = os.path.abspath(root_dir)
    summary = []
    errors = []

    # Track folders where we already reported naming mismatch
    reported_mesh_naming_folders = set()

    # Track SEGY_MESH directories already processed (critical!)
    processed_mesh_dirs = set()

    # Buscar TODOS los archivos válidos
    valid_ext = (".sgy", "_merged.out", ".rad", ".dzt")
    all_files = []

    for dirpath, _, files in os.walk(root_dir):
        for f in files:
            if f.lower().endswith(valid_ext):
                all_files.append(os.path.join(dirpath, f))

    if not all_files:
        print("No se encontraron archivos válidos.")
        return summary

def run_bscan(M, raw, dirpath, loader_type):
    final_output_base = output_dir if output_dir is not None else dirpath

    return process_bscan(
            M_original=M,
            output_base=final_output_base,
            raw_name=raw,
            loader_type=loader_type,
            do_original_plot=do_original_plot,
            do_original_matrix=do_original_matrix,
            do_info=do_info,
            do_dewow_plot=do_dewow_plot,
            do_dewow_matrix=do_dewow_matrix,
            dewow_window=dewow_window,
            do_background_removal_plot=do_background_removal_plot,
            do_background_removal_matrix=do_background_removal_matrix,
            do_gain_plot=do_gain_plot,
            do_gain_matrix=do_gain_matrix,
            linear_gain_db=linear_gain_db,
            do_time_zero_plot=do_time_zero_plot,
            do_time_zero_matrix=do_time_zero_matrix,
            do_trim_plot=do_trim_plot,
            do_trim_matrix=do_trim_matrix,
            trim_bounds=trim_bounds,
            do_agc_plot=do_agc_plot,
            do_agc_matrix=do_agc_matrix,
            agc_window=agc_window,
            do_combine_plot=do_combine_plot,
            do_combine_matrix=do_combine_matrix,
            combine_edits=combine_edits,
            plot_scale=plot_scale,
            aspect_ratio=aspect_ratio,
            plot_colormap=plot_colormap,
            plot_axes=plot_axes,
            plot_axes_in_meters=plot_axes_in_meters,
            plot_axes_in_ns=plot_axes_in_ns,
            overwrite=overwrite,
            summary=summary
    )

    # ============================================================
    # Procesar cada archivo usando DISPATCHER
    # ============================================================
    for full_path in all_files:
        dirpath = os.path.dirname(full_path)

        loader_type = detect_loader_type(full_path)
        if loader_type == "UNKNOWN":
            print(f"[WARNING] Tipo desconocido: {full_path}")
            continue

        # SEGY_MESH: process once per directory (otherwise each .sgy triggers the whole mesh)
        if loader_type == "SEGY_MESH":
            norm_mesh_dir = os.path.normpath(dirpath)
            if norm_mesh_dir in processed_mesh_dirs:
                continue
            processed_mesh_dirs.add(norm_mesh_dir)

        try:
            entries = load_matrix_by_type(full_path, loader_type)
        except Exception as e:
            errors.append(f"Loader failed for {full_path}: {e}")
            continue

        # SEGY_MESH naming mismatch: report once per folder
        if loader_type == "SEGY_MESH" and not entries:
            if dirpath in reported_mesh_naming_folders:
                continue
            reported_mesh_naming_folders.add(dirpath)

            segy_files = sorted(glob.glob(os.path.join(dirpath, "*.sgy")))
            examples = ", ".join([os.path.basename(x) for x in segy_files[:5]]) if segy_files else "(none found)"

            err_msg = (
                "Detected loader type SEGY_MESH (multiple .sgy files found), but no B-scans matched "
                "the required filename pattern '_L###_' (e.g., '..._L001_...'). "
                f"Examples of detected .sgy files: {examples}"
            )
            print(f"\n[ERROR] {err_msg}")
            errors.append(f"SEGY_MESH naming mismatch in folder '{dirpath}': {err_msg}")
            continue

        for raw_path, M in entries:
            basename = os.path.basename(raw_path)

            # Split only once from the right (robust for multiple dots)
            if "." in basename:
                name, ext = basename.rsplit(".", 1)
                ext = "." + ext
            else:
                name = basename
                ext = ""

            # Handle composed suffix like "_merged"
            if name.lower().endswith("_merged"):
                name = name[:-len("_merged")]

            raw_name = name  # default for non-mesh formats

            # SEGY_MESH: guarantee uniqueness per line
            if loader_type == "SEGY_MESH":
                m = re.search(r"_L(\d{3})_", raw_name)
                line_id = f"L{m.group(1)}" if m else "LUNK"
                raw_name = f"{os.path.basename(dirpath)}_{line_id}"

            print(f"\n[INFO] Procesando {raw_name} (tipo={loader_type})")

            try:
                run_bscan(M, raw_name, dirpath, loader_type)
            except Exception as e:
                errors.append(f"BSCAN failed for {raw_name}: {e}")

    # ============================================================
    # SUMMARY
    # ============================================================
    print("\n========== SUMMARY ==========")
    print(f"Root directory                : {root_dir}")
    print(f"Input files detected          : {len(all_files)}")
    print(f"Successfully processed B-scans: {len(summary)}")
    print(f"Errors                        : {len(errors)}")

    if summary:
        print("\n--- PROCESSED B-SCANS ---")
        for fname, edits in summary:
            print(f"{fname}: {edits}")

    if errors:
        print("\n--- ERRORS ---")
        for e in errors:
            print(f"- {e}")

    print("================================\n")

    return summary
