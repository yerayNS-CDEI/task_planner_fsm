import os
import re
import glob
import tempfile
import hashlib
import numpy as np

# Dispatcher para loaders
from GPRTools.loaders.loader_dispatcher import (
    detect_loader_type,
    load_matrix_by_type,
)

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
# FLAT OUTPUT PATHS
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
# COMMON FILE DISCOVERY / NAMING HELPERS
# ====================================================================
def _find_valid_bscan_files(root_dir):
    valid_ext = (".sgy", "_merged.out", ".rad", ".dzt")
    all_files = []

    for dirpath, _, files in os.walk(root_dir):
        for f in files:
            if f.lower().endswith(valid_ext):
                all_files.append(os.path.join(dirpath, f))

    return sorted(all_files)


def _get_raw_name_from_entry(raw_path, dirpath, loader_type):
    basename = os.path.basename(raw_path)

    if "." in basename:
        name, _ = basename.rsplit(".", 1)
    else:
        name = basename

    if name.lower().endswith("_merged"):
        name = name[:-len("_merged")]

    raw_name = name

    if loader_type == "SEGY_MESH":
        m = re.search(r"_L(\d{3})_", raw_name)
        line_id = f"L{m.group(1)}" if m else "LUNK"
        raw_name = f"{os.path.basename(dirpath)}_{line_id}"

    return raw_name


def _collect_preview_bscans(all_files, max_preview=None):
    """
    Load B-scans for interactive parameter preview.

    If max_preview is None, all available B-scans are loaded.
    """
    preview_entries = []
    processed_mesh_dirs = set()

    for full_path in all_files:
        dirpath = os.path.dirname(full_path)

        loader_type = detect_loader_type(full_path)
        if loader_type == "UNKNOWN":
            continue

        if loader_type == "SEGY_MESH":
            norm_mesh_dir = os.path.normpath(dirpath)
            if norm_mesh_dir in processed_mesh_dirs:
                continue
            processed_mesh_dirs.add(norm_mesh_dir)

        try:
            entries = load_matrix_by_type(full_path, loader_type)
        except Exception:
            continue

        for raw_path, M in entries:
            raw_name = _get_raw_name_from_entry(
                raw_path=raw_path,
                dirpath=dirpath,
                loader_type=loader_type,
            )

            preview_entries.append({
                "raw_name": raw_name,
                "raw_path": raw_path,
                "loader_type": loader_type,
                "M": M,
            })

            if max_preview is not None and len(preview_entries) >= max_preview:
                return preview_entries

    return preview_entries


# ====================================================================
# STANDALONE INTERACTIVE PREVIEW
# ====================================================================
def preview_bscan_processing_flat(root_dir):
    """
    Interactive preview for B-scan processing parameters.

    This function does NOT process or save the full batch. It only:
        - finds B-scan files in root_dir,
        - lets you select one B-scan from a dropdown,
        - loads the selected B-scan lazily,
        - renders original and edited versions side by side using plot_bscan(),
        - displays widgets for combined edits and plotting options,
        - returns a state dictionary with the selected parameters.

    Notes about speed:
        - Matrices are loaded only when selected.
        - Loaded matrices are cached.
        - Original/preview PNG renders are cached by parameters.
        - The edited matrix is cached by selected B-scan + combine_edits.
    """
    root_dir = os.path.abspath(root_dir)
    all_files = _find_valid_bscan_files(root_dir)

    default_plot_config = {
        "plot_scale": 8.0,  # hidden preview/final render scale used by plot_bscan
        "aspect_ratio": None,
        "plot_colormap": "gray",
        "plot_axes": False,
        "plot_axes_in_meters": True,
        "plot_axes_in_ns": True,
    }

    empty_state = {
        "combine_edits": [],
        "plot_config": default_plot_config.copy(),
        "preview_entries": [],
        "selected_entry": None,
    }

    if not all_files:
        print("No se encontraron archivos validos.")
        return empty_state

    try:
        import pprint
        import matplotlib.pyplot as plt
        import matplotlib.image as mpimg
        import ipywidgets as widgets
        from IPython.display import display, clear_output, HTML
    except Exception as e:
        raise ImportError(
            "The interactive preview requires a Jupyter environment with ipywidgets installed. "
            "Install it with: pip install ipywidgets"
        ) from e

    # ------------------------------------------------------------
    # Build a lightweight catalog. For normal single B-scans this does
    # not load the matrix yet. The matrix is loaded only when selected.
    # SEGY_MESH still needs load_matrix_by_type once per mesh folder.
    # ------------------------------------------------------------
    preview_entries = []
    processed_mesh_dirs = set()

    for full_path in all_files:
        dirpath = os.path.dirname(full_path)
        loader_type = detect_loader_type(full_path)

        if loader_type == "UNKNOWN":
            continue

        if loader_type == "SEGY_MESH":
            norm_mesh_dir = os.path.normpath(dirpath)
            if norm_mesh_dir in processed_mesh_dirs:
                continue
            processed_mesh_dirs.add(norm_mesh_dir)

            try:
                mesh_entries = load_matrix_by_type(full_path, loader_type)
            except Exception:
                continue

            for raw_path, M in mesh_entries:
                raw_name = _get_raw_name_from_entry(
                    raw_path=raw_path,
                    dirpath=dirpath,
                    loader_type=loader_type,
                )
                preview_entries.append({
                    "raw_name": raw_name,
                    "raw_path": raw_path,
                    "source_path": full_path,
                    "loader_type": loader_type,
                    "M": M,
                    "matrix_loaded": True,
                })
        else:
            raw_name = _get_raw_name_from_entry(
                raw_path=full_path,
                dirpath=dirpath,
                loader_type=loader_type,
            )
            preview_entries.append({
                "raw_name": raw_name,
                "raw_path": full_path,
                "source_path": full_path,
                "loader_type": loader_type,
                "M": None,
                "matrix_loaded": False,
            })

    if not preview_entries:
        print("No se pudieron indexar B-scans para preview.")
        return empty_state

    print(
        f"\n[PREVIEW] Indexados {len(preview_entries)} B-scan(s). "
        "Selecciona uno en el desplegable para cargarlo y previsualizarlo."
    )

    # ------------------------------------------------------------
    # Caches
    # ------------------------------------------------------------
    preview_render_dir = tempfile.mkdtemp(prefix="gprtools_preview_")
    edited_matrix_cache = {}
    rendered_image_cache = {}

    def _safe_filename(text):
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", text)

    def _stable_key(obj):
        return repr(obj)

    def _hash_key(obj):
        return hashlib.md5(_stable_key(obj).encode("utf-8")).hexdigest()[:12]

    def get_entry_id(entry):
        return f"{entry['loader_type']}::{entry['raw_path']}::{entry['raw_name']}"

    def load_selected_matrix(entry):
        """
        Load matrix only once for the selected entry.
        """
        if entry.get("matrix_loaded") and entry.get("M") is not None:
            return entry["M"]

        entries = load_matrix_by_type(entry["source_path"], entry["loader_type"])
        if not entries:
            raise RuntimeError(f"No data returned by loader for {entry['source_path']}")

        # Prefer exact raw_path match when possible.
        selected_M = None
        for raw_path, M in entries:
            if os.path.normpath(raw_path) == os.path.normpath(entry["raw_path"]):
                selected_M = M
                break

        if selected_M is None:
            selected_M = entries[0][1]

        entry["M"] = selected_M
        entry["matrix_loaded"] = True
        return selected_M

    def compute_preview_matrix(M_original, combine_edits, entry):
        if not combine_edits:
            state["applied_edits"] = []
            return M_original.copy()

        key = (
            get_entry_id(entry),
            tuple((name, tuple(sorted(params.items()))) for name, params in combine_edits),
        )

        if key in edited_matrix_cache:
            return edited_matrix_cache[key]

        try:
            M_preview, applied_edits, _ = apply_edit_pipeline(
                M_original.copy(),
                combine_edits
            )

            state["applied_edits"] = applied_edits

        except Exception as e:
            error_msg = (
                f"Preview failed for {entry['raw_name']}\n\n"
                f"combine_edits = {combine_edits}\n\n"
                f"error = {repr(e)}"
            )

            raise RuntimeError(error_msg) from e

        edited_matrix_cache[key] = M_preview
        return M_preview

    # ------------------------------------------------------------
    # Widgets
    # ------------------------------------------------------------
    bscan_dropdown = widgets.Dropdown(
        options=[(entry["raw_name"], i) for i, entry in enumerate(preview_entries)],
        value=0,
        description="B-scan",
        layout=widgets.Layout(width="950px"),
    )

    use_time_zero = widgets.Checkbox(value=False, description="time_zero")

    time_zero_method_dropdown = widgets.Dropdown(
        options=[
            ("method 1: threshold breach - Nsamp", 1),
            ("method 2: peak response - Nsamp", 2),
            ("method 3: zero crossing - Nsamp", 3),
        ],
        value=1,
        description="method",
        layout=widgets.Layout(width="360px"),
    )

    time_zero_threshold_box = widgets.FloatText(
        value=0.2,
        description="threshold",
        layout=widgets.Layout(width="180px"),
    )

    time_zero_start_sample_box = widgets.IntText(
        value=0,
        description="start",
        layout=widgets.Layout(width="160px"),
    )

    time_zero_backup_samples_box = widgets.IntText(
        value=5,
        description="backup",
        layout=widgets.Layout(width="160px"),
    )

    use_dewow = widgets.Checkbox(value=False, description="dewow")
    dewow_slider = widgets.IntSlider(
        value=50,
        min=1,
        max=500,
        step=1,
        description="window",
        continuous_update=False,
    )

    use_background = widgets.Checkbox(value=False, description="background_removal")

    preview_warning = widgets.HTML(
        value="""
        <div style="
            padding:8px;
            border:1px solid #d6d6d6;
            background:#fff8e1;
            color:#444;
            border-radius:6px;
            margin:6px 0;
        ">
            <b>Preview mode:</b> this preview uses a reduced render quality
            (<code>render_upsample=1.5</code>) for speed.
            Final saved plots will use higher render quality.
        </div>
        """
    )
    
    use_gain = widgets.Checkbox(value=False, description="gain")
    gain_slider = widgets.IntSlider(
        value=40,
        min=0,
        max=80,
        step=1,
        description="gain dB",
        continuous_update=False,
    )

    use_agc = widgets.Checkbox(value=False, description="agc")
    agc_slider = widgets.IntSlider(
        value=40,
        min=1,
        max=1000,
        step=1,
        description="window",
        continuous_update=False,
    )

    use_trim = widgets.Checkbox(value=False, description="trim")
    trim_t1_box = widgets.IntText(
        value=0,
        description="t1",
        layout=widgets.Layout(width="160px"),
    )
    trim_t2_box = widgets.IntText(
        value=0,
        description="t2",
        layout=widgets.Layout(width="160px"),
    )
    trim_info_html = widgets.HTML(value="")

    colormap_dropdown = widgets.Dropdown(
        options=[
            "gray",
            "binary",
            "seismic",
            "viridis",
            "plasma",
            "magma",
            "inferno",
            "turbo",
            "jet",
        ],
        value="gray",
        description="colormap",
    )

    aspect_mode = widgets.Dropdown(
        options=[
            ("default / Screening Eagle empirical", "default"),
            ("manual aspect_x/aspect_y", "manual"),
        ],
        value="default",
        description="aspect",
    )

    manual_aspect_x = widgets.FloatText(
        value=600.0,
        description="aspect_x",
        layout=widgets.Layout(width="220px"),
    )
    manual_aspect_y = widgets.FloatText(
        value=400.0,
        description="aspect_y",
        layout=widgets.Layout(width="220px"),
    )
    manual_aspect_box = widgets.HBox([manual_aspect_x, manual_aspect_y])

    plot_axes_check = widgets.Checkbox(value=False, description="show axes")
    plot_axes_meters_check = widgets.Checkbox(value=True, description="x in meters")
    plot_axes_ns_check = widgets.Checkbox(value=True, description="y in ns")

    status_html = widgets.HTML(value="")
    output = widgets.Output()
    code_box = widgets.Textarea(
        value="",
        description="selected",
        layout=widgets.Layout(width="950px", height="230px"),
    )

    state = {
        "combine_edits": [],
        "plot_config": default_plot_config.copy(),
        "preview_entries": preview_entries,
        "selected_entry": preview_entries[0],
    }

    # ------------------------------------------------------------
    # Current selections
    # ------------------------------------------------------------
    def get_current_combine_edits():
        edits = []

        if use_time_zero.value:
            edits.append(("time_zero", {
                "method": int(time_zero_method_dropdown.value),
                "threshold": float(time_zero_threshold_box.value),
                "start_sample": int(time_zero_start_sample_box.value),
                "backup_samples": int(time_zero_backup_samples_box.value),
            }))

        if use_dewow.value:
            edits.append(("dewow", {"window": int(dewow_slider.value)}))

        if use_background.value:
            edits.append(("background_removal", {}))

        if use_gain.value:
            edits.append(("gain", {"gain_db": int(gain_slider.value)}))

        if use_agc.value:
            edits.append(("agc", {"window": int(agc_slider.value)}))

        if use_trim.value:
            t1 = int(trim_t1_box.value)
            t2 = int(trim_t2_box.value)
            if t2 > t1:
                edits.append(("trim", {"t1": t1, "t2": t2}))

        return edits

    def get_current_aspect_ratio():
        if aspect_mode.value == "default":
            return None

        aspect_x = float(manual_aspect_x.value)
        aspect_y = float(manual_aspect_y.value)

        if aspect_x <= 0 or aspect_y <= 0:
            return None

        return (aspect_x, aspect_y)

    def get_current_plot_config():
        return {
            "plot_scale": default_plot_config["plot_scale"],
            "aspect_ratio": get_current_aspect_ratio(),
            "plot_colormap": colormap_dropdown.value,
            "plot_axes": bool(plot_axes_check.value),
            "plot_axes_in_meters": bool(plot_axes_meters_check.value),
            "plot_axes_in_ns": bool(plot_axes_ns_check.value),
        }

    def format_selected_text(combine_edits, plot_config):
        aspect_ratio = plot_config["aspect_ratio"]
        aspect_ratio_text = "None" if aspect_ratio is None else repr(aspect_ratio)

        applied_edits = state.get("applied_edits", [])

        return (
            "combine_edits = "
            + pprint.pformat(combine_edits, width=90)
            + "\n\n"
            + "applied_edits = "
            + pprint.pformat(applied_edits, width=90)
            + "\n\n"
            + f"plot_scale={plot_config['plot_scale']},\n"
            + f"aspect_ratio={aspect_ratio_text},\n"
            + f"plot_colormap=\"{plot_config['plot_colormap']}\",\n"
            + f"plot_axes={plot_config['plot_axes']},\n"
            + f"plot_axes_in_meters={plot_config['plot_axes_in_meters']},\n"
            + f"plot_axes_in_ns={plot_config['plot_axes_in_ns']},"
        )

    def update_manual_aspect_visibility(*args):
        manual_aspect_box.layout.display = "flex" if aspect_mode.value == "manual" else "none"

    def update_trim_info(entry, M):
        n_samples = M.shape[0]
        trim_info_html.value = f"<span style='color:#666;'>valid sample range: 0 - {n_samples - 1}</span>"

    def config_cache_key(plot_config):
        return (
            plot_config["plot_scale"],
            plot_config["aspect_ratio"],
            plot_config["plot_colormap"],
            plot_config["plot_axes"],
            plot_config["plot_axes_in_meters"],
            plot_config["plot_axes_in_ns"],
        )

    def edits_cache_key(combine_edits):
        return tuple((name, tuple(sorted(params.items()))) for name, params in combine_edits)

    def render_bscan_with_plotter(M, entry, suffix, plot_config, matrix_key):
        """
        Render a B-scan using the same plot_bscan() function used by the
        final batch output, with caching so repeated previews are fast.
        """
        render_key = (
            get_entry_id(entry),
            suffix,
            matrix_key,
            config_cache_key(plot_config),
        )

        if render_key in rendered_image_cache:
            cached_path = rendered_image_cache[render_key]
            if os.path.exists(cached_path):
                return cached_path

        hash_id = _hash_key(render_key)
        base_name = f"{_safe_filename(entry['raw_name'])}_{suffix}_{hash_id}.png"
        out_path = os.path.join(preview_render_dir, base_name)

        plot_bscan(
            self=None,
            M=M,
            applied_edits=[],
            base_name=base_name,
            base_dir=preview_render_dir,
            save=True,
            plot_scale=plot_config["plot_scale"],
            aspect_ratio=plot_config["aspect_ratio"],
            plot_colormap=plot_config["plot_colormap"],
            plot_axes=plot_config["plot_axes"],
            plot_axes_in_meters=plot_config["plot_axes_in_meters"],
            plot_axes_in_ns=plot_config["plot_axes_in_ns"],
            overwrite=True,
            loader_type=entry["loader_type"],
            src_file_path=entry["raw_path"],
            render_upsample=1.5,
        )

        rendered_image_cache[render_key] = out_path
        return out_path

    def show_rendered_bscan(ax, image_path, title):
        img = mpimg.imread(image_path)
        ax.imshow(img)
        ax.set_title(title, fontsize=9)
        ax.axis("off")
        if img.shape[1] > 0:
            ax.set_box_aspect(img.shape[0] / img.shape[1])

    # ------------------------------------------------------------
    # Redraw callback
    # ------------------------------------------------------------
    def redraw(*args):
        status_html.value = "<b>Cargando preview...</b>"
        with output:
            clear_output(wait=True)
            display(HTML("<b>Cargando preview...</b>"))

        combine_edits = get_current_combine_edits()
        plot_config = get_current_plot_config()
        selected_index = int(bscan_dropdown.value)
        selected_entry = preview_entries[selected_index]

        state["combine_edits"] = combine_edits
        state["plot_config"] = plot_config
        state["selected_entry"] = selected_entry
        code_box.value = format_selected_text(combine_edits, plot_config)

        try:
            M_original = load_selected_matrix(selected_entry)
            update_trim_info(selected_entry, M_original)

            edit_key = edits_cache_key(combine_edits)
            M_preview = compute_preview_matrix(M_original, combine_edits, selected_entry)
            code_box.value = format_selected_text(combine_edits, plot_config)

            original_path = render_bscan_with_plotter(
                M_original,
                selected_entry,
                suffix="original",
                plot_config=plot_config,
                matrix_key="original",
            )

            preview_path = render_bscan_with_plotter(
                M_preview,
                selected_entry,
                suffix="preview",
                plot_config=plot_config,
                matrix_key=edit_key,
            )

            original_img = mpimg.imread(original_path)
            image_h, image_w = original_img.shape[:2]
            single_panel_aspect = image_h / max(image_w, 1)

            total_fig_w = 15.0
            single_panel_w = total_fig_w / 2.0
            fig_h = single_panel_w * single_panel_aspect
            fig_h = min(max(fig_h, 4.0), 10.0)

            with output:
                clear_output(wait=True)
                fig, axes = plt.subplots(1, 2, figsize=(total_fig_w, fig_h), squeeze=False)

                show_rendered_bscan(
                    axes[0, 0],
                    original_path,
                    f"{selected_entry['raw_name']}\noriginal",
                )
                show_rendered_bscan(
                    axes[0, 1],
                    preview_path,
                    f"{selected_entry['raw_name']}\npreview",
                )

                plt.tight_layout()
                plt.show()

            status_html.value = "<span style='color: green;'>Preview actualizado.</span>"

        except Exception as e:
            status_html.value = "<span style='color: red;'>Error en preview.</span>"
            with output:
                clear_output(wait=True)
                print(f"[ERROR] Preview failed: {e}")

        # ------------------------------------------------------------
    # Widget layout
    # ------------------------------------------------------------
    time_zero_controls = widgets.VBox([
        use_time_zero,
        time_zero_method_dropdown,
        widgets.HBox([
            time_zero_threshold_box,
            time_zero_start_sample_box,
            time_zero_backup_samples_box,
        ]),
    ])

    edit_controls = widgets.VBox([
        widgets.HTML("<b>Combined edits</b>"),
        time_zero_controls,
        widgets.HBox([use_dewow, dewow_slider]),
        use_background,
        widgets.HBox([use_gain, gain_slider]),
        widgets.HBox([use_agc, agc_slider]),
        widgets.HBox([use_trim, trim_t1_box, trim_t2_box]),
        trim_info_html,
    ])

    plot_controls = widgets.VBox([
        widgets.HTML("<b>Plot options</b>"),
        colormap_dropdown,
        aspect_mode,
        manual_aspect_box,
        plot_axes_check,
        widgets.HBox([plot_axes_meters_check, plot_axes_ns_check]),
    ])

    update_button = widgets.Button(
        description="Update preview",
        button_style="primary",
        icon="refresh",
        layout=widgets.Layout(width="180px"),
    )

    def on_update_clicked(button):
        update_button.disabled = True
        update_button.description = "Loading..."
        status_html.value = "<b>Cargando preview...</b>"

        try:
            redraw()
        finally:
            update_button.disabled = False
            update_button.description = "Update preview"

    update_button.on_click(on_update_clicked)

    controls = widgets.VBox([
        widgets.HTML("<h3>B-scan processing preview</h3>"),
        bscan_dropdown,
        widgets.HBox([edit_controls, plot_controls]),
        widgets.HBox([update_button, status_html]),
    ])

    # Only lightweight UI behavior should update automatically.
    # Do NOT connect all widgets to redraw(), otherwise Jupyter may freeze
    # or behave inconsistently.
    update_manual_aspect_visibility()
    aspect_mode.observe(update_manual_aspect_visibility, names="value")

    display(controls)
    display(preview_warning)
    display(output)
    display(code_box)

    status_html.value = (
        "<span style='color:#666;'>Select a B-scan, choose the parameters, "
        "then click <b>Update preview</b>.</span>"
    )

    return state

# ====================================================================
# UNIVERSAL BSCAN PROCESSOR - FLAT OUTPUT
# ====================================================================
def process_bscan_flat(
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
    summary,
):
    plots_dir, info_dir, matrices_dir = _ensure_flat_output_dirs(output_base)

    # Base path used by plotters/info when they need a source file path.
    # Convention kept from the existing workflow.
    raw_path_base = os.path.join(output_base, raw_name)

    # ============================================================
    # INFO TXT
    # ============================================================
    if do_info:
        metadata = extract_metadata(
            file_path=raw_path_base,
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
            src_file_path=raw_path_base,
            render_upsample=3,
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
        M_edit, applied_edits, _ = apply_edit_pipeline(
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
                src_file_path=raw_path_base,
                render_upsample=3,
            )

    # ============================================================
    # 2. COMBINED
    # ============================================================
    if do_combine_plot or do_combine_matrix:
        combine_edits = combine_edits or []

        M_combined, applied_edits_combined, _ = apply_edit_pipeline(
            M_original.copy(),
            combine_edits,
        )
        applied_edits_last = applied_edits_combined

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
                src_file_path=raw_path_base,
                render_upsample=3,
            )

    summary.append((raw_name, applied_edits_last))


# ====================================================================
# MAIN BATCH - MULTI-FORMAT, FLAT OUTPUT
# ====================================================================
def batch_process_bscans_flat(
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
    plot_scale=2,
    aspect_ratio=None,
    plot_colormap="gray",
    plot_axes=False,
    plot_axes_in_meters=True,
    plot_axes_in_ns=True,
    overwrite=False,
    output_dir=None,
    **kwargs,
):
    root_dir = os.path.abspath(root_dir)
    final_output_base = os.path.abspath(output_dir) if output_dir is not None else root_dir

    summary = []
    errors = []

    reported_mesh_naming_folders = set()
    processed_mesh_dirs = set()

    all_files = _find_valid_bscan_files(root_dir)

    if not all_files:
        print("No se encontraron archivos válidos.")
        return summary

    for full_path in all_files:
        dirpath = os.path.dirname(full_path)

        loader_type = detect_loader_type(full_path)
        if loader_type == "UNKNOWN":
            print(f"[WARNING] Tipo desconocido: {full_path}")
            continue

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
            raw_name = _get_raw_name_from_entry(
                raw_path=raw_path,
                dirpath=dirpath,
                loader_type=loader_type,
            )

            print(f"\n[INFO] Procesando {raw_name} (tipo={loader_type})")

            try:
                process_bscan_flat(
                    M_original=M,
                    output_base=final_output_base,
                    raw_name=raw_name,
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
                    summary=summary,
                )
            except Exception as e:
                errors.append(f"BSCAN failed for {raw_name}: {e}")

    print("\n========== SUMMARY ==========")
    print(f"Root directory                : {root_dir}")
    print(f"Flat output directory         : {final_output_base}")
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
