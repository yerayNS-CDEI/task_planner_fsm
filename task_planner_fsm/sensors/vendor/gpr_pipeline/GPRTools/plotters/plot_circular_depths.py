import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import SymLogNorm
from scipy.interpolate import interp1d, RegularGridInterpolator

from GPRTools.info.info_dispatcher import extract_metadata
from GPRTools.loaders.loader_dispatcher import detect_loader_type, load_matrix_by_type
from GPRTools.pipelines.edit_pipeline import apply_edit_pipeline


DEFAULT_RING_THICKNESS = 0.09
_SLICE_REDUCERS = {
    "mean": lambda window: np.mean(window, axis=1),
    "median": lambda window: np.median(window, axis=1),
    "rms": lambda window: np.sqrt(np.mean(window ** 2, axis=1)),
    "gprslice": lambda window: np.mean(np.abs(window), axis=1),
}


def _as_trace_sample_matrix(M):
    """Convert (n_samples, n_traces) into (n_traces, n_samples)."""
    M = np.asarray(M)
    if M.ndim != 2:
        raise ValueError("Input matrix must be 2D")
    return M.T


def _load_bscan_items(file_path):
    """Load every matrix found in a source path together with its metadata."""
    loader_type = detect_loader_type(file_path)
    if loader_type == "UNKNOWN":
        return []

    loaded_items = load_matrix_by_type(file_path, loader_type) or []
    return [
        (
            extract_metadata(src_path, detect_loader_type(src_path)),
            np.asarray(M, dtype=float),
        )
        for src_path, M in loaded_items
    ]


def _apply_preprocess(M, preprocess_edits):
    if not preprocess_edits:
        return np.array(M, copy=True), []
    return apply_edit_pipeline(np.array(M, copy=True), preprocess_edits)[:2]


def _load_single_bscan(file_path, preprocess_edits=None):
    """Load one B-scan and return original/processed matrices."""
    loaded_items = _load_bscan_items(file_path)
    if not loaded_items:
        raise ValueError(f"No data could be loaded from: {file_path}")

    metadata, original_M = loaded_items[0]
    processed_M, applied_edits = _apply_preprocess(original_M, preprocess_edits)
    return metadata, _as_trace_sample_matrix(processed_M), original_M, processed_M, applied_edits


def _load_circular_inputs(file_paths, radii=None, preprocess_edits=None):
    """Load matrices and metadata ready for circular plotting."""
    if not file_paths:
        raise ValueError("file_paths cannot be empty")

    radii = list(range(1, len(file_paths) + 1)) if radii is None else radii
    if len(radii) != len(file_paths):
        raise ValueError("radii must have the same length as file_paths")

    metadata_list, data_list, expanded_radii = [], [], []
    for file_path, radius in zip(file_paths, radii):
        for metadata, M in _load_bscan_items(file_path):
            processed_M, _ = _apply_preprocess(M, preprocess_edits)
            metadata_list.append(metadata)
            data_list.append(_as_trace_sample_matrix(processed_M))
            expanded_radii.append(radius)

    if not data_list:
        raise ValueError("No valid B-scans could be loaded from file_paths")

    return metadata_list, data_list, expanded_radii


def _get_sample_bounds(data_list):
    """Return the common valid sample range across all B-scans."""
    sample_end = min(data.shape[1] for data in data_list) - 1
    if sample_end < 0:
        raise ValueError("Loaded B-scans do not contain valid samples")
    return 0, sample_end


def _resolve_samples(samples, n_slices, sample_start, sample_end, data_list):
    """Resolve explicit samples or an automatic sample range."""
    if samples is not None:
        if any(value is not None for value in (n_slices, sample_start, sample_end)):
            raise ValueError(
                "Use either samples=[...], or the automatic mode "
                "(sample_start, sample_end, n_slices), not both"
            )
        if not samples:
            raise ValueError("samples cannot be empty")
        return [int(sample) for sample in samples]

    if n_slices is None or n_slices <= 0:
        raise ValueError("n_slices must be provided and > 0 when samples is not used")

    common_start, common_end = _get_sample_bounds(data_list)
    sample_start = common_start if sample_start is None else int(sample_start)
    sample_end = common_end if sample_end is None else int(sample_end)

    if not (common_start <= sample_start <= common_end):
        raise ValueError(
            f"sample_start must be within the common range [{common_start}, {common_end}]"
        )
    if not (common_start <= sample_end <= common_end):
        raise ValueError(
            f"sample_end must be within the common range [{common_start}, {common_end}]"
        )
    if sample_end < sample_start:
        raise ValueError("sample_end must be >= sample_start")
    if n_slices == 1:
        return [sample_start]

    return np.linspace(sample_start, sample_end, int(n_slices)).round().astype(int).tolist()


def _resolve_preview_file_indices(preview_file_idx, n_files):
    if isinstance(preview_file_idx, (list, tuple, np.ndarray)):
        indices = [int(idx) for idx in preview_file_idx]
    else:
        indices = [int(preview_file_idx)]

    if not indices:
        raise ValueError("preview_file_idx cannot be empty")
    if any(idx < 0 or idx >= n_files for idx in indices):
        raise ValueError(f"preview_file_idx must be within [0, {n_files - 1}]")
    return indices


def _mark_trace_zero_on_bscan(ax):
    ax.axvline(0, color="white", linestyle="--", linewidth=1)


def _mark_trace_zero_on_polar(ax, radii, ring_thickness=DEFAULT_RING_THICKNESS):
    max_radius = max(radii) + ring_thickness
    theta0 = 0
    ax.plot(
        [theta0, theta0],
        [0.0, max_radius],
        color="white",
        linestyle="--",
        linewidth=2,
        zorder=10,
    )
    ax.scatter(
        np.full(len(radii), theta0),
        np.asarray(radii, dtype=float),
        color="white",
        s=12,
        zorder=11,
    )


def _mark_trace_zero_on_xy(ax, radii, ring_thickness=DEFAULT_RING_THICKNESS):
    max_radius = max(radii) + ring_thickness
    ax.plot(
        [0.0, max_radius],
        [0.0, 0.0],
        color="white",
        linestyle="--",
        linewidth=2,
        zorder=10,
    )
    ax.scatter(
        np.asarray(radii, dtype=float),
        np.zeros(len(radii)),
        color="white",
        s=12,
        zorder=11,
    )

def _plot_bscan_on_axis(ax, M, metadata, title=None, cmap="gray", log_scale=False):
    """Plot a standard B-scan using ns when dt is available."""
    M = np.asarray(M, dtype=float)
    n_samples, n_traces = M.shape
    dt = metadata.get("dt")
    ymax = float(n_samples - 1) * float(dt) if dt is not None else float(n_samples - 1)
    ylabel = "Time (ns)" if dt is not None else "Sample"
    vmin, vmax = np.percentile(M, [1, 99])

    image_kwargs = {
        "cmap": cmap,
        "aspect": "auto",
        "interpolation": "nearest",
        "extent": [0.0, float(n_traces - 1), ymax, 0.0],
        "origin": "upper",
    }
    if log_scale:
        image_kwargs["norm"] = SymLogNorm(
            max((vmax - vmin) * 0.03, 1e-12),
            vmin=vmin,
            vmax=vmax,
            base=10,
        )
    else:
        image_kwargs.update({"vmin": vmin, "vmax": vmax})

    im = ax.imshow(M, **image_kwargs)
    ax.set_xlabel("Trace #")
    ax.set_ylabel(ylabel)
    _mark_trace_zero_on_bscan(ax)
    if title:
        ax.set_title(title)
    return im


def preview_bscan_for_circular_selection(
    file_path,
    preprocess_edits=None,
    sample_lines=None,
    save_to=None,
    cmap="gray",
    log_scale=False,
    dpi=150,
):
    """Preview one B-scan before/after preprocessing."""
    metadata, _, original_M, processed_M, applied_edits = _load_single_bscan(
        file_path=file_path,
        preprocess_edits=preprocess_edits,
    )

    fig, axes = plt.subplots(1, 2, figsize=(14, 5), dpi=dpi, sharey=True)
    im0 = _plot_bscan_on_axis(
        axes[0],
        original_M,
        metadata,
        title="Original B-scan",
        cmap=cmap,
        log_scale=log_scale,
    )
    fig.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.04)

    processed_title = "Processed B-scan"
    if applied_edits:
        processed_title += "\n" + ", ".join(applied_edits)

    im1 = _plot_bscan_on_axis(
        axes[1],
        processed_M,
        metadata,
        title=processed_title,
        cmap=cmap,
        log_scale=log_scale,
    )
    fig.colorbar(im1, ax=axes[1], fraction=0.046, pad=0.04)

    for ax in axes:
        for sample in sample_lines or []:
            ax.axhline(int(sample), linestyle="--", linewidth=1)

    fig.tight_layout()
    if save_to:
        plt.savefig(save_to, dpi=dpi, bbox_inches="tight")
        plt.close()
        return save_to

    plt.show()
    return None


def _reduce_slice_window(window, slice_reduce):
    if slice_reduce == "maxabs":
        idx = np.argmax(np.abs(window), axis=1)
        return window[np.arange(window.shape[0]), idx]

    reducer = _SLICE_REDUCERS.get(slice_reduce)
    if reducer is None:
        raise ValueError("slice_reduce must be one of {'mean', 'median', 'maxabs', 'rms', 'gprslice'}")
    return reducer(window)


def _extract_layer_data(data, sample_idx, slice_mode="point", slice_thickness=0, slice_reduce="mean"):
    """Extract one circular slice from data shaped as (n_traces, n_samples)."""
    idx = int(sample_idx)
    if not 0 <= idx < data.shape[1]:
        return None
    if slice_mode == "point":
        return data[:, idx]
    if slice_mode != "band":
        raise ValueError("slice_mode must be one of {'point', 'band'}")

    slice_thickness = int(slice_thickness)
    if slice_thickness < 0:
        raise ValueError("slice_thickness must be >= 0")

    s0 = max(0, idx - slice_thickness)
    s1 = min(data.shape[1] - 1, idx + slice_thickness) + 1
    return _reduce_slice_window(data[:, s0:s1], slice_reduce)


def _iter_slice_values(data_list, sample_idx, slice_mode, slice_thickness, slice_reduce):
    if sample_idx is None:
        for data in data_list:
            if slice_mode == "point":
                yield data.ravel()
                continue

            _, sample_end = _get_sample_bounds([data])
            for current_idx in range(sample_end + 1):
                layer = _extract_layer_data(
                    data=data,
                    sample_idx=current_idx,
                    slice_mode=slice_mode,
                    slice_thickness=slice_thickness,
                    slice_reduce=slice_reduce,
                )
                if layer is not None:
                    yield layer
        return

    for data in data_list:
        layer = _extract_layer_data(
            data=data,
            sample_idx=sample_idx,
            slice_mode=slice_mode,
            slice_thickness=slice_thickness,
            slice_reduce=slice_reduce,
        )
        if layer is not None:
            yield layer


def _resolve_color_limits_for_slice_mode(
    color_scale,
    vmin,
    vmax,
    data_list,
    sample_idx=None,
    slice_mode="point",
    slice_thickness=0,
    slice_reduce="mean",
):
    """Resolve color limits for the requested slice mode."""
    if color_scale == "manual":
        if vmin is None or vmax is None:
            raise ValueError("For color_scale='manual', both vmin and vmax must be provided")
        return float(vmin), float(vmax)

    if color_scale not in {"global", "slice"}:
        raise ValueError("color_scale must be one of {'global', 'slice', 'manual'}")

    values = list(
        _iter_slice_values(
            data_list=data_list,
            sample_idx=None if color_scale == "global" else sample_idx,
            slice_mode=slice_mode,
            slice_thickness=slice_thickness,
            slice_reduce=slice_reduce,
        )
    )
    if not values:
        target = "global color scale" if color_scale == "global" else f"sample {sample_idx}"
        raise ValueError(f"No valid slice data found for {target}")

    values = np.concatenate(values)
    return float(np.min(values)), float(np.max(values))


def _interpolate_layer(layer_data, max_traces):
    original_theta = np.linspace(0, 2 * np.pi, len(layer_data), endpoint=False)
    target_theta = np.linspace(0, 2 * np.pi, max_traces, endpoint=False)
    interpolator = interp1d(original_theta, layer_data, kind="linear", fill_value="extrapolate")
    return target_theta, interpolator(target_theta)


def _collect_pixel_slice_points(
    data_list,
    radii,
    sample_idx,
    slice_mode="point",
    slice_thickness=0,
    slice_reduce="mean",
):
    xy_points, values = [], []
    for data, radius in zip(data_list, radii):
        layer_data = _extract_layer_data(
            data=data,
            sample_idx=sample_idx,
            slice_mode=slice_mode,
            slice_thickness=slice_thickness,
            slice_reduce=slice_reduce,
        )
        if layer_data is None:
            continue

        theta = np.linspace(0, 2 * np.pi, len(layer_data), endpoint=False)
        xy_points.append(np.column_stack([radius * np.cos(theta), radius * np.sin(theta)]))
        values.append(layer_data)

    if not values:
        return None, None
    return np.vstack(xy_points), np.concatenate(values)


def _idw_interpolate_grid(
    xy_points,
    values,
    x_grid,
    y_grid,
    power=2,
    eps=1e-12,
    max_neighbors=None,
    theta_window_deg=None,
    chunk_size=2048,
):
    target_points = np.column_stack([x_grid.ravel(), y_grid.ravel()])
    interpolated = np.empty(target_points.shape[0], dtype=float)

    if max_neighbors is not None:
        max_neighbors = max(1, min(int(max_neighbors), xy_points.shape[0]))

    source_theta = None
    theta_window = None
    if theta_window_deg is not None:
        theta_window = np.deg2rad(float(theta_window_deg))
        source_theta = np.arctan2(xy_points[:, 1], xy_points[:, 0])

    for start in range(0, target_points.shape[0], chunk_size):
        stop = min(start + chunk_size, target_points.shape[0])
        chunk_points = target_points[start:stop]
        deltas = chunk_points[:, None, :] - xy_points[None, :, :]
        distances = np.sqrt(np.sum(deltas ** 2, axis=2))
        valid_mask = np.ones_like(distances, dtype=bool)

        if theta_window is not None:
            target_theta = np.arctan2(chunk_points[:, 1], chunk_points[:, 0])[:, None]
            angular_diff = np.abs(np.angle(np.exp(1j * (target_theta - source_theta[None, :]))))
            valid_mask = angular_diff <= theta_window

        if max_neighbors is not None and max_neighbors < xy_points.shape[0]:
            masked_distances = np.where(valid_mask, distances, np.inf)
            nearest_idx = np.argpartition(masked_distances, kth=max_neighbors - 1, axis=1)[:, :max_neighbors]
            neighbor_mask = np.zeros_like(valid_mask, dtype=bool)
            row_idx = np.arange(nearest_idx.shape[0])[:, None]
            neighbor_mask[row_idx, nearest_idx] = np.isfinite(masked_distances[row_idx, nearest_idx])
            valid_mask &= neighbor_mask

        missing_rows = ~np.any(valid_mask, axis=1)
        if np.any(missing_rows):
            if max_neighbors is not None and max_neighbors < xy_points.shape[0]:
                fallback_idx = np.argpartition(distances[missing_rows], kth=max_neighbors - 1, axis=1)[:, :max_neighbors]
                fallback_mask = np.zeros_like(distances[missing_rows], dtype=bool)
                row_idx = np.arange(fallback_idx.shape[0])[:, None]
                fallback_mask[row_idx, fallback_idx] = True
                valid_mask[missing_rows] = fallback_mask
            else:
                valid_mask[missing_rows] = True

        exact_mask = (distances <= eps) & valid_mask
        safe_distances = np.where(exact_mask, eps, distances)
        weights = np.where(valid_mask, 1.0 / (safe_distances ** power), 0.0)
        chunk_values = (weights @ values) / np.sum(weights, axis=1)

        if np.any(exact_mask):
            exact_rows = np.any(exact_mask, axis=1)
            exact_cols = np.argmax(exact_mask[exact_rows], axis=1)
            chunk_values[exact_rows] = values[exact_cols]

        interpolated[start:stop] = chunk_values

    return interpolated.reshape(x_grid.shape)


def _build_pixel_slice_grid(
    data_list,
    radii,
    sample_idx,
    max_traces,
    ring_thickness=DEFAULT_RING_THICKNESS,
    slice_mode="point",
    slice_thickness=0,
    slice_reduce="mean",
    pixel_grid_size=None,
    pixel_max_neighbors=None,
    pixel_theta_window_deg=None,
):
    xy_points, values = _collect_pixel_slice_points(
        data_list=data_list,
        radii=radii,
        sample_idx=sample_idx,
        slice_mode=slice_mode,
        slice_thickness=slice_thickness,
        slice_reduce=slice_reduce,
    )
    if values is None:
        return None

    max_radius = max(radii) + ring_thickness
    grid_size = max(16, int(pixel_grid_size)) if pixel_grid_size is not None else max(128, int(max_traces))
    x_axis = np.linspace(-max_radius, max_radius, grid_size)
    y_axis = np.linspace(-max_radius, max_radius, grid_size)
    x_grid, y_grid = np.meshgrid(x_axis, y_axis)

    cartesian_values = _idw_interpolate_grid(
        xy_points=xy_points,
        values=values,
        x_grid=x_grid,
        y_grid=y_grid,
        power=2,
        max_neighbors=pixel_max_neighbors,
        theta_window_deg=pixel_theta_window_deg,
    )

    polar_theta = np.linspace(0, 2 * np.pi, max_traces, endpoint=False)
    polar_r = np.linspace(0, max_radius, grid_size)
    theta_grid, r_grid = np.meshgrid(polar_theta, polar_r)
    sample_points = np.column_stack([
        (r_grid * np.sin(theta_grid)).ravel(),
        (r_grid * np.cos(theta_grid)).ravel(),
    ])
    cartesian_to_polar = RegularGridInterpolator(
        (y_axis, x_axis),
        cartesian_values,
        bounds_error=False,
        fill_value=np.nan,
    )
    polar_values = cartesian_to_polar(sample_points).reshape(r_grid.shape)

    return {
        "x_grid": x_grid,
        "y_grid": y_grid,
        "cartesian_values": cartesian_values,
        "theta_grid": theta_grid,
        "r_grid": r_grid,
        "values": polar_values,
    }


def _resolve_pixel_color_limits(color_scale, vmin, vmax, pixel_cache, sample_idx=None):
    if color_scale == "manual":
        if vmin is None or vmax is None:
            raise ValueError("For color_scale='manual', both vmin and vmax must be provided")
        return float(vmin), float(vmax)

    if color_scale == "slice":
        slice_grid = pixel_cache.get(sample_idx)
        if slice_grid is None:
            raise ValueError(f"No valid interpolated slice data found for sample {sample_idx}")
        valid_values = slice_grid["values"][np.isfinite(slice_grid["values"])]
    elif color_scale == "global":
        valid_arrays = [
            grid["values"][np.isfinite(grid["values"])]
            for grid in pixel_cache.values()
            if grid is not None
        ]
        if not valid_arrays:
            raise ValueError("No valid interpolated values found to compute color limits")
        valid_values = np.concatenate(valid_arrays)
    else:
        raise ValueError("color_scale must be one of {'global', 'slice', 'manual'}")

    if valid_values.size == 0:
        raise ValueError("No valid interpolated values found to compute color limits")
    return float(np.min(valid_values)), float(np.max(valid_values))


def _plot_circular_bscan_linear(
    ax,
    data_list,
    radii,
    sample_idx,
    max_traces,
    cmap,
    log_scale=False,
    ring_thickness=DEFAULT_RING_THICKNESS,
    color_scale="global",
    vmin=None,
    vmax=None,
    slice_mode="point",
    slice_thickness=0,
    slice_reduce="mean",
    zero_angle="N",
):
    current_vmin, current_vmax = _resolve_color_limits_for_slice_mode(
        color_scale=color_scale,
        vmin=vmin,
        vmax=vmax,
        data_list=data_list,
        sample_idx=sample_idx,
        slice_mode=slice_mode,
        slice_thickness=slice_thickness,
        slice_reduce=slice_reduce,
    )
    norm = None
    if log_scale:
        norm = SymLogNorm(
            max((current_vmax - current_vmin) * 0.03, 1e-12),
            vmin=current_vmin,
            vmax=current_vmax,
            base=10,
        )

    mesh = None
    for data, radius in zip(data_list, radii):
        layer_data = _extract_layer_data(
            data=data,
            sample_idx=sample_idx,
            slice_mode=slice_mode,
            slice_thickness=slice_thickness,
            slice_reduce=slice_reduce,
        )
        if layer_data is None:
            continue

        theta, interpolated = _interpolate_layer(layer_data, max_traces)
        r = np.array([radius - ring_thickness, radius + ring_thickness])
        theta_grid, r_grid = np.meshgrid(theta, r)
        mesh_kwargs = {"cmap": cmap, "shading": "auto"}
        if norm is not None:
            mesh_kwargs["norm"] = norm
        else:
            mesh_kwargs.update({"vmin": current_vmin, "vmax": current_vmax})

        mesh = ax.pcolormesh(
            theta_grid,
            r_grid,
            np.tile(interpolated, (2, 1)),
            **mesh_kwargs,
        )

    ax.set_yticks([])
    ax.set_xticks([])
    _mark_trace_zero_on_polar(ax, radii, ring_thickness)
    ax.set_theta_zero_location(zero_angle)
    ax.set_theta_direction(-1)
    return mesh


def _plot_pixel_xy_bscan_on_axis(
    ax,
    pixel_grid,
    radii,
    cmap,
    log_scale=False,
    color_scale="global",
    vmin=None,
    vmax=None,
    sample_idx=None,
    pixel_cache=None,
    pixel_color_limits=None,
    ring_thickness=DEFAULT_RING_THICKNESS,
):
    if color_scale == "global" and pixel_color_limits is not None:
        current_vmin, current_vmax = pixel_color_limits
    else:
        current_vmin, current_vmax = _resolve_pixel_color_limits(
            color_scale=color_scale,
            vmin=vmin,
            vmax=vmax,
            pixel_cache=pixel_cache,
            sample_idx=sample_idx,
        )

    mesh_kwargs = {"cmap": cmap, "shading": "auto"}
    if log_scale:
        mesh_kwargs["norm"] = SymLogNorm(
            max((current_vmax - current_vmin) * 0.03, 1e-12),
            vmin=current_vmin,
            vmax=current_vmax,
            base=10,
        )
    else:
        mesh_kwargs.update({"vmin": current_vmin, "vmax": current_vmax})

    mesh = ax.pcolormesh(
        pixel_grid["x_grid"],
        pixel_grid["y_grid"],
        pixel_grid["cartesian_values"],
        **mesh_kwargs,
    )
    _mark_trace_zero_on_xy(ax, radii, ring_thickness)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal")
    return mesh

def _plot_circular_bscan_pixel_xy(
    ax,
    data_list,
    radii,
    sample_idx,
    max_traces,
    cmap,
    log_scale=False,
    ring_thickness=DEFAULT_RING_THICKNESS,
    color_scale="global",
    vmin=None,
    vmax=None,
    slice_mode="point",
    slice_thickness=0,
    slice_reduce="mean",
    zero_angle="N",
    pixel_cache=None,
    pixel_color_limits=None,
    pixel_grid_size=None,
    pixel_max_neighbors=None,
    pixel_theta_window_deg=None,
):
    if pixel_cache is None:
        pixel_cache = {}

    pixel_grid = pixel_cache.get(sample_idx)
    if pixel_grid is None:
        pixel_grid = _build_pixel_slice_grid(
            data_list=data_list,
            radii=radii,
            sample_idx=sample_idx,
            max_traces=max_traces,
            ring_thickness=ring_thickness,
            slice_mode=slice_mode,
            slice_thickness=slice_thickness,
            slice_reduce=slice_reduce,
            pixel_grid_size=pixel_grid_size,
            pixel_max_neighbors=pixel_max_neighbors,
            pixel_theta_window_deg=pixel_theta_window_deg,
        )
        pixel_cache[sample_idx] = pixel_grid

    if pixel_grid is None:
        return None

    if color_scale == "global" and pixel_color_limits is not None:
        current_vmin, current_vmax = pixel_color_limits
    else:
        current_vmin, current_vmax = _resolve_pixel_color_limits(
            color_scale=color_scale,
            vmin=vmin,
            vmax=vmax,
            pixel_cache=pixel_cache,
            sample_idx=sample_idx,
        )

    mesh_kwargs = {"cmap": cmap, "shading": "auto"}
    if log_scale:
        mesh_kwargs["norm"] = SymLogNorm(
            max((current_vmax - current_vmin) * 0.03, 1e-12),
            vmin=current_vmin,
            vmax=current_vmax,
            base=10,
        )
    else:
        mesh_kwargs.update({"vmin": current_vmin, "vmax": current_vmax})

    mesh = ax.pcolormesh(
        pixel_grid["theta_grid"],
        pixel_grid["r_grid"],
        pixel_grid["values"],
        **mesh_kwargs,
    )
    ax.set_yticks([])
    ax.set_xticks([])
    _mark_trace_zero_on_polar(ax, radii, ring_thickness)
    ax.set_theta_zero_location(zero_angle)
    ax.set_theta_direction(-1)
    return mesh


def plot_circular_bscan(
    ax,
    metadata_list,
    data_list,
    radii,
    sample_idx,
    max_traces,
    cmap,
    log_scale=False,
    ring_thickness=DEFAULT_RING_THICKNESS,
    color_scale="global",
    vmin=None,
    vmax=None,
    slice_mode="point",
    slice_thickness=0,
    slice_reduce="mean",
    zero_angle="N",
    interp_method="linear",
    pixel_cache=None,
    pixel_color_limits=None,
    pixel_grid_size=None,
    pixel_max_neighbors=None,
    pixel_theta_window_deg=None,
):
    """Draw one polar slice using multiple B-scans."""
    del metadata_list

    if interp_method == "linear":
        return _plot_circular_bscan_linear(
            ax=ax,
            data_list=data_list,
            radii=radii,
            sample_idx=sample_idx,
            max_traces=max_traces,
            cmap=cmap,
            log_scale=log_scale,
            ring_thickness=ring_thickness,
            color_scale=color_scale,
            vmin=vmin,
            vmax=vmax,
            slice_mode=slice_mode,
            slice_thickness=slice_thickness,
            slice_reduce=slice_reduce,
            zero_angle=zero_angle,
        )

    if interp_method == "pixel":
        return _plot_circular_bscan_pixel_xy(
            ax=ax,
            data_list=data_list,
            radii=radii,
            sample_idx=sample_idx,
            max_traces=max_traces,
            cmap=cmap,
            log_scale=log_scale,
            ring_thickness=ring_thickness,
            color_scale=color_scale,
            vmin=vmin,
            vmax=vmax,
            slice_mode=slice_mode,
            slice_thickness=slice_thickness,
            slice_reduce=slice_reduce,
            zero_angle=zero_angle,
            pixel_cache=pixel_cache,
            pixel_color_limits=pixel_color_limits,
            pixel_grid_size=pixel_grid_size,
            pixel_max_neighbors=pixel_max_neighbors,
            pixel_theta_window_deg=pixel_theta_window_deg,
        )

    raise ValueError("interp_method must be one of {'linear', 'pixel'}")


def plot_circular_bscans_for_depths(
    file_paths=None,
    samples=None,
    *,
    n_slices=None,
    sample_start=None,
    sample_end=None,
    metadata_list=None,
    data_list=None,
    radii=None,
    preprocess_edits=None,
    preview_bscan=False,
    preview_file_idx=0,
    preview_sample_lines=True,
    cmap="viridis",
    preview_cmap="gray",
    n_cols=5,
    save_to=None,
    log_scale=False,
    ring_thickness=DEFAULT_RING_THICKNESS,
    color_scale="global",
    vmin=None,
    vmax=None,
    slice_mode="point",
    slice_thickness=0,
    slice_reduce="mean",
    dpi=150,
    per_subplot_size=(2.4, 2.4),
    max_figsize=None,
    zero_angle="N",
    interp_method="linear",
    pixel_grid_size=None,
    pixel_max_neighbors=None,
    pixel_theta_window_deg=None,
    show_pixel_xy=False,
):
    """Plot circular B-scan views for the requested samples."""
    if file_paths is not None:
        metadata_list, data_list, radii = _load_circular_inputs(
            file_paths=file_paths,
            radii=radii,
            preprocess_edits=preprocess_edits,
        )
    elif metadata_list is None or data_list is None or radii is None:
        raise ValueError("Provide either file_paths, or metadata_list/data_list/radii explicitly")

    samples = _resolve_samples(
        samples=samples,
        n_slices=n_slices,
        sample_start=sample_start,
        sample_end=sample_end,
        data_list=data_list,
    )

    if file_paths is not None and preview_bscan:
        preview_indices = _resolve_preview_file_indices(preview_file_idx, len(file_paths))
        for preview_idx in preview_indices:
            preview_bscan_for_circular_selection(
                file_path=file_paths[preview_idx],
                preprocess_edits=preprocess_edits,
                sample_lines=samples if preview_sample_lines else None,
                cmap=preview_cmap,
                log_scale=log_scale,
                dpi=dpi,
            )

    max_traces = max(data.shape[0] for data in data_list)
    pixel_cache = None
    pixel_color_limits = None
    if interp_method == "pixel":
        pixel_cache = {
            sample_idx: _build_pixel_slice_grid(
                data_list=data_list,
                radii=radii,
                sample_idx=sample_idx,
                max_traces=max_traces,
                ring_thickness=ring_thickness,
                slice_mode=slice_mode,
                slice_thickness=slice_thickness,
                slice_reduce=slice_reduce,
                pixel_grid_size=pixel_grid_size,
                pixel_max_neighbors=pixel_max_neighbors,
                pixel_theta_window_deg=pixel_theta_window_deg,
            )
            for sample_idx in samples
        }
        if color_scale == "global":
            pixel_color_limits = _resolve_pixel_color_limits(
                color_scale=color_scale,
                vmin=vmin,
                vmax=vmax,
                pixel_cache=pixel_cache,
            )

    n_rows = int(np.ceil(len(samples) / n_cols))

    fig_width = n_cols * per_subplot_size[0]
    fig_height = n_rows * per_subplot_size[1]
    if max_figsize is not None:
        fig_width = min(fig_width, max_figsize[0])
        fig_height = min(fig_height, max_figsize[1])

    fig, axes = plt.subplots(
        n_rows,
        n_cols,
        figsize=(fig_width, fig_height),
        subplot_kw={"projection": "polar"},
    )
    axes = np.atleast_1d(axes).flatten()

    mesh = None
    for i, sample_idx in enumerate(samples):
        mesh = plot_circular_bscan(
            ax=axes[i],
            metadata_list=metadata_list,
            data_list=data_list,
            radii=radii,
            sample_idx=sample_idx,
            max_traces=max_traces,
            cmap=cmap,
            log_scale=log_scale,
            ring_thickness=ring_thickness,
            color_scale=color_scale,
            vmin=vmin,
            vmax=vmax,
            slice_mode=slice_mode,
            slice_thickness=slice_thickness,
            slice_reduce=slice_reduce,
            zero_angle=zero_angle,
            interp_method=interp_method,
            pixel_cache=pixel_cache,
            pixel_color_limits=pixel_color_limits,
            pixel_grid_size=pixel_grid_size,
            pixel_max_neighbors=pixel_max_neighbors,
            pixel_theta_window_deg=pixel_theta_window_deg,
        )
        title = f"Sample:\n{sample_idx}"
        if slice_mode != "point":
            title += f"\n+/-{int(slice_thickness)} ({slice_reduce})"
        axes[i].set_title(title, va="top", fontsize=8, y=1.15)

    for ax in axes[len(samples):]:
        fig.delaxes(ax)

    if mesh is not None:
        cbar_ax = fig.add_axes([0.82, 0.15, 0.02, 0.7])
        fig.colorbar(mesh, cax=cbar_ax, orientation="vertical", label="Amplitude")

    fig.subplots_adjust(left=0.1, right=0.8, top=0.9, bottom=0.1)

    if interp_method == "pixel" and show_pixel_xy:
        fig_xy, axes_xy = plt.subplots(
            n_rows,
            n_cols,
            figsize=(fig_width, fig_height),
        )
        axes_xy = np.atleast_1d(axes_xy).flatten()
        mesh_xy = None
        for i, sample_idx in enumerate(samples):
            pixel_grid = pixel_cache.get(sample_idx) if pixel_cache is not None else None
            if pixel_grid is None:
                continue
            mesh_xy = _plot_pixel_xy_bscan_on_axis(
                ax=axes_xy[i],
                pixel_grid=pixel_grid,
                radii=radii,
                cmap=cmap,
                log_scale=log_scale,
                color_scale=color_scale,
                vmin=vmin,
                vmax=vmax,
                sample_idx=sample_idx,
                pixel_cache=pixel_cache,
                pixel_color_limits=pixel_color_limits,
                ring_thickness=ring_thickness,
            )
            title = f"Sample:\n{sample_idx}"
            if slice_mode != "point":
                title += f"\n+/-{int(slice_thickness)} ({slice_reduce})"
            axes_xy[i].set_title(title, fontsize=8)

        for ax in axes_xy[len(samples):]:
            fig_xy.delaxes(ax)

        if mesh_xy is not None:
            cbar_ax_xy = fig_xy.add_axes([0.82, 0.15, 0.02, 0.7])
            fig_xy.colorbar(mesh_xy, cax=cbar_ax_xy, orientation="vertical", label="Amplitude")

        fig_xy.subplots_adjust(left=0.1, right=0.8, top=0.9, bottom=0.1)

    if save_to:
        plt.savefig(save_to, dpi=dpi, bbox_inches="tight")
        plt.close(fig)
        return save_to

    plt.show()
    return None






