import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import zoom, gaussian_filter

from GPRTools.info.info_dispatcher import extract_metadata


def get_output_folder(base_dir, applied_edits, mode="plots"):
    return base_dir, ""


def plot_bscan(
    self,
    M,
    applied_edits,
    *,
    base_name,
    base_dir,
    save=True,
    plot_scale=8,
    aspect_ratio=None,
    plot_colormap="gray",
    plot_axes=False,
    plot_axes_in_meters=True,
    plot_axes_in_ns=True,
    overwrite=False,

    auto_contrast=True,
    contrast_percentiles=(1, 99),
    vmin=None,
    vmax=None,

    loader_type=None,
    src_file_path=None,
    render_upsample=3,
    **kwargs
):
    """
    High-quality BSCAN plotting with axis in meters or ns.
    """

    os.makedirs(base_dir, exist_ok=True)
    out_path = os.path.join(base_dir, base_name)

    if (not overwrite) and os.path.exists(out_path):
        return out_path

    # ---------------------------------------------------------
    # READ METADATA
    # ---------------------------------------------------------
    dx = None
    dt = None

    if plot_axes and (plot_axes_in_meters or plot_axes_in_ns):

        # Si nos pasan la ruta original, la usamos. Si no, fallback antiguo.
        if src_file_path is not None:
            meta_path = src_file_path
        else:
            # Fallback (menos fiable, pero por compatibilidad)
            meta_path = os.path.join(base_dir, base_name.replace(".png", ""))

        metadata = extract_metadata(
            file_path=meta_path,
            loader_type=loader_type,
        )

        dx = metadata.get("dx")
        dt = metadata.get("dt")

    # Flags reales: solo usamos unidades físicas si dx/dt existen
    use_meters = plot_axes and plot_axes_in_meters and (dx is not None)
    use_ns     = plot_axes and plot_axes_in_ns     and (dt is not None)

    # ---------------------------------------------------------
    # PREPARE AXES VALUES
    # ---------------------------------------------------------
    n_samples, n_traces = M.shape

    # X axis
    if use_meters:
        xmin = 0.0
        xmax = (n_traces - 1) * dx
    else:
        xmin = 0.0
        xmax = float(n_traces - 1)

    # Y axis
    if use_ns:
        ymin = 0.0
        ymax = (n_samples - 1) * dt
    else:
        ymin = 0.0
        ymax = float(n_samples - 1)

    # ---------------------------------------------------------
    # PREPARE MATRIX FOR VISUAL RENDERING
    # ---------------------------------------------------------
    M_display = np.array(M, dtype=float)

    # Remove invalid values if any
    M_display = np.nan_to_num(M_display, nan=0.0, posinf=0.0, neginf=0.0)

    # Robust symmetric amplitude range around zero.
    # Use a high percentile to avoid hard saturation of strong reflections.
    clip = np.percentile(np.abs(M_display), 99.7)

    if clip > 0:
        # Soft amplitude compression.
        # This avoids large white/black saturated blocks.
        soft = 0.35 * clip
        M_display = np.arcsinh(M_display / soft)

    # Optional visual upsampling. This is only for rendering, not for data.
    if render_upsample is None:
        render_upsample = 1

    render_upsample = float(render_upsample)

    if render_upsample > 1:
        M_display = zoom(
            M_display,
            zoom=(render_upsample, render_upsample),
            order=3
        )

    # Light smoothing after upsampling
    M_display = gaussian_filter(M_display, sigma=0.35)

    # Symmetric display range after compression
    v = np.percentile(np.abs(M_display), 99.9)
    vmin = -v
    vmax = v

    # ---------------------------------------------------------
    # FIGURE ASPECT RATIO
    # ---------------------------------------------------------
    if aspect_ratio is None:
        # Screening Eagle-like empirical aspect ratio
        #
        # x = num_samples / num_traces
        # y = aspect_y / aspect_x
        #
        # Empirical fit:
        # y = 0.624 * x + 0.1985

        sample_trace_ratio = n_samples / n_traces
        aspect_y_over_x = 0.624 * sample_trace_ratio + 0.1985

        # Our plotting code expects:
        # aspect_x = figure width proportion
        # aspect_y = figure height proportion
        #
        # Since aspect_y_over_x = aspect_y / aspect_x,
        # we can set aspect_x = 1 and aspect_y = aspect_y_over_x.
        aspect_x = 1.0
        aspect_y = aspect_y_over_x

    else:
        aspect_x, aspect_y = aspect_ratio

    fig_w = plot_scale * aspect_x
    fig_h = plot_scale * aspect_y

    plt.figure(figsize=(fig_w, fig_h))

    # ---------------------------------------------------------
    # FIGURE
    # ---------------------------------------------------------
    plt.imshow(
        M_display,
        cmap=plot_colormap,
        aspect='auto',
        interpolation='bilinear',
        vmin=vmin,
        vmax=vmax,
        extent=[xmin, xmax, ymax, ymin],  # orden correcto
        origin='upper',                    # 0 arriba, tiempo crece hacia abajo
    )

    # ---------------------------------------------------------
    # AXIS LABELS
    # ---------------------------------------------------------
    if not plot_axes:
        plt.axis("off")
    else:
        plt.xlabel("Distance (m)" if use_meters else "Trace #")
        plt.ylabel("Time (ns)" if use_ns else "Sample")

    plt.tight_layout(pad=0)

    if save:
        plt.savefig(out_path, dpi=300, bbox_inches='tight', pad_inches=0)
        plt.close()

    return out_path
