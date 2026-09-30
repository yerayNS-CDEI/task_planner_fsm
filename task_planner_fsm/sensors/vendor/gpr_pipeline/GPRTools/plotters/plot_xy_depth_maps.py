import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import SymLogNorm
from scipy.interpolate import griddata

from GPRTools.info.info_dispatcher import extract_metadata
from GPRTools.loaders.loader_dispatcher import detect_loader_type, load_matrix_by_type
from GPRTools.pipelines.edit_pipeline import apply_edit_pipeline


DEFAULT_GRID_SIZE = 200
DEFAULT_IDW_POWER = 2.0
DEFAULT_SEARCH_TYPE = "elliptical"
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


def _load_scan_items(file_path):
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


def _merge_preprocess_edits(preprocess_edits=None, extra_changes=None):
    """Merge the main preprocessing pipeline with optional extra changes."""
    merged = list(preprocess_edits or [])
    if extra_changes:
        merged.extend(list(extra_changes))
    return merged


def _apply_preprocess(M, preprocess_edits):
    if not preprocess_edits:
        return np.array(M, copy=True), []
    return apply_edit_pipeline(np.array(M, copy=True), preprocess_edits)[:2]


def _interpolate_trace_xy_from_endpoints(endpoints, n_traces):
    """Build one XY coordinate per trace from line endpoints.

    Supported endpoint formats:
    - flat sequence ``[x1, x2, y1, y2]``
    - two points ``[[x1, y1], [x2, y2]]``
    """
    endpoints = np.asarray(endpoints, dtype=float)

    if endpoints.shape == (4,):
        x1, x2, y1, y2 = endpoints
    elif endpoints.shape == (2, 2):
        x1, y1 = endpoints[0]
        x2, y2 = endpoints[1]
    else:
        raise ValueError(
            "Trace endpoints must have shape (4,) as [x1, x2, y1, y2] "
            "or shape (2, 2) as [[x1, y1], [x2, y2]]"
        )

    if int(n_traces) <= 1:
        return np.array([[float(x1), float(y1)]], dtype=float)

    x_values = np.linspace(float(x1), float(x2), int(n_traces))
    y_values = np.linspace(float(y1), float(y2), int(n_traces))
    return np.column_stack([x_values, y_values])


def _build_trace_xy_points(metadata, n_traces):
    """Build XY coordinates for each trace.

    Preferred input is a line definition per B-scan, so the trace coordinates are
    interpolated from the start/end XY positions.
    """
    candidate_keys = ("xy_points", "trace_xy", "positions_xy")
    for key in candidate_keys:
        xy_points = metadata.get(key)
        if xy_points is None:
            continue
        xy_points = np.asarray(xy_points, dtype=float)
        if xy_points.shape == (n_traces, 2):
            return xy_points

    endpoint_keys = (
        ("trace_xy_endpoints",),
        ("xy_endpoints",),
        ("line_xy",),
        ("trace_line",),
        ("x1", "x2", "y1", "y2"),
        ("x_start", "x_end", "y_start", "y_end"),
        ("trace_x_start", "trace_x_end", "trace_y_start", "trace_y_end"),
        ("start_x", "end_x", "start_y", "end_y"),
    )
    for keys in endpoint_keys:
        if len(keys) == 1:
            endpoints = metadata.get(keys[0])
            if endpoints is None:
                continue
        else:
            values = [metadata.get(key) for key in keys]
            if any(value is None for value in values):
                continue
            endpoints = values

        try:
            return _interpolate_trace_xy_from_endpoints(endpoints, n_traces)
        except ValueError:
            continue

    axis_pairs = (
        ("trace_x", "trace_y"),
        ("x", "y"),
        ("scan_x", "scan_y"),
        ("pos_x", "pos_y"),
    )
    for x_key, y_key in axis_pairs:
        x_values = metadata.get(x_key)
        y_values = metadata.get(y_key)
        if x_values is None or y_values is None:
            continue
        x_values = np.asarray(x_values, dtype=float).ravel()
        y_values = np.asarray(y_values, dtype=float).ravel()
        if len(x_values) == n_traces and len(y_values) == n_traces:
            return np.column_stack([x_values, y_values])

    # Default fallback: traces laid out along X, all belonging to the same Y.
    return np.column_stack([np.arange(n_traces, dtype=float), np.zeros(n_traces, dtype=float)])


def _coerce_xy_points_input(xy_points, n_traces):
    """Accept either full XY coordinates or line endpoints."""
    xy_points = np.asarray(xy_points, dtype=float)
    if xy_points.shape == (n_traces, 2):
        return xy_points
    return _interpolate_trace_xy_from_endpoints(xy_points, n_traces)


def _load_single_depth_scan(file_path, preprocess_edits=None, xy_points=None):
    """Load one scan and return original/processed matrices plus XY coordinates."""
    loaded_items = _load_scan_items(file_path)
    if not loaded_items:
        raise ValueError(f"No data could be loaded from: {file_path}")

    metadata, original_M = loaded_items[0]
    processed_M, applied_edits = _apply_preprocess(original_M, preprocess_edits)
    data_ts = _as_trace_sample_matrix(processed_M)
    if xy_points is None:
        xy_points = _build_trace_xy_points(metadata, data_ts.shape[0])
    else:
        xy_points = _coerce_xy_points_input(xy_points, data_ts.shape[0])
    return metadata, data_ts, original_M, processed_M, applied_edits, xy_points


def _load_xy_inputs(file_paths, xy_points_list=None, preprocess_edits=None):
    """Load matrices, metadata and XY trace coordinates for XY depth mapping."""
    if not file_paths:
        raise ValueError("file_paths cannot be empty")

    if xy_points_list is not None and len(xy_points_list) != len(file_paths):
        raise ValueError("xy_points_list must have the same length as file_paths")

    metadata_list, data_list, loaded_xy_points = [], [], []
    for file_idx, file_path in enumerate(file_paths):
        custom_xy_points = None if xy_points_list is None else xy_points_list[file_idx]
        for metadata, M in _load_scan_items(file_path):
            processed_M, _ = _apply_preprocess(M, preprocess_edits)
            data_ts = _as_trace_sample_matrix(processed_M)
            if custom_xy_points is None:
                xy_points = _build_trace_xy_points(metadata, data_ts.shape[0])
            else:
                xy_points = _coerce_xy_points_input(custom_xy_points, data_ts.shape[0])
            if xy_points.shape != (data_ts.shape[0], 2):
                raise ValueError(
                    "Each xy_points_list entry must be either an array with shape (n_traces, 2) "
                    "or endpoints with shape (4,) as [x1, x2, y1, y2]"
                )

            metadata_list.append(metadata)
            data_list.append(data_ts)
            loaded_xy_points.append(xy_points)

    if not data_list:
        raise ValueError("No valid scans could be loaded from file_paths")

    return metadata_list, data_list, loaded_xy_points


def _get_sample_bounds(data_list):
    """Return the common valid sample range across all loaded scans."""
    sample_end = min(data.shape[1] for data in data_list) - 1
    if sample_end < 0:
        raise ValueError("Loaded scans do not contain valid samples")
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

    if n_slices is None or int(n_slices) <= 0:
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
    if int(n_slices) == 1:
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


def _plot_scan_on_axis(ax, M, metadata, title=None, cmap="gray", log_scale=False):
    """Plot one standard scan using ns if dt is available."""
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

    image = ax.imshow(M, **image_kwargs)
    ax.set_xlabel("Trace #")
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)
    return image


def preview_xy_map_selection(
    file_path,
    preprocess_edits=None,
    sample_lines=None,
    save_to=None,
    cmap="gray",
    log_scale=False,
    dpi=150,
    xy_points=None,
):
    """Preview one scan before/after preprocessing so the user can choose slices."""
    metadata, _, original_M, processed_M, applied_edits, _ = _load_single_depth_scan(
        file_path=file_path,
        preprocess_edits=preprocess_edits,
        xy_points=xy_points,
    )

    fig, axes = plt.subplots(1, 2, figsize=(14, 5), dpi=dpi, sharey=True)
    image_original = _plot_scan_on_axis(
        axes[0],
        original_M,
        metadata,
        title="Original scan",
        cmap=cmap,
        log_scale=log_scale,
    )
    fig.colorbar(image_original, ax=axes[0], fraction=0.046, pad=0.04)

    processed_title = "Processed scan"
    if applied_edits:
        processed_title += "\n" + ", ".join(applied_edits)

    image_processed = _plot_scan_on_axis(
        axes[1],
        processed_M,
        metadata,
        title=processed_title,
        cmap=cmap,
        log_scale=log_scale,
    )
    fig.colorbar(image_processed, ax=axes[1], fraction=0.046, pad=0.04)

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
    """Extract one depth slice from data shaped as (n_traces, n_samples)."""
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

    sample_start = max(0, idx - slice_thickness)
    sample_end = min(data.shape[1] - 1, idx + slice_thickness) + 1
    return _reduce_slice_window(data[:, sample_start:sample_end], slice_reduce)


def _bin_trace_slice_points(xy_points, values, bins_per_mark=1, bin_reduce="mean"):
    """Optionally average consecutive traces to mimic GPR-SLICE style binning."""
    bins_per_mark = int(bins_per_mark)
    if bins_per_mark <= 1:
        return np.asarray(xy_points, dtype=float), np.asarray(values, dtype=float)

    xy_points = np.asarray(xy_points, dtype=float)
    values = np.asarray(values, dtype=float)
    n_points = values.size
    if n_points == 0:
        return xy_points, values

    reducers = {
        "mean": np.mean,
        "median": np.median,
    }
    reducer = reducers.get(bin_reduce)
    if reducer is None:
        raise ValueError("bin_reduce must be one of {'mean', 'median'}")

    binned_xy, binned_values = [], []
    for start in range(0, n_points, bins_per_mark):
        stop = min(start + bins_per_mark, n_points)
        binned_xy.append(np.mean(xy_points[start:stop], axis=0))
        binned_values.append(reducer(values[start:stop]))

    return np.vstack(binned_xy), np.asarray(binned_values, dtype=float)


def _build_xy_slice_points(
    data_list,
    xy_points_list,
    sample_idx,
    slice_mode="point",
    slice_thickness=0,
    slice_reduce="mean",
    bins_per_mark=1,
    bin_reduce="mean",
    amplitude_mode="signed",
):
    xy_chunks, value_chunks = [], []
    for data, xy_points in zip(data_list, xy_points_list):
        layer_values = _extract_layer_data(
            data=data,
            sample_idx=sample_idx,
            slice_mode=slice_mode,
            slice_thickness=slice_thickness,
            slice_reduce=slice_reduce,
        )
        if layer_values is None:
            continue
        if len(layer_values) != len(xy_points):
            raise ValueError("Each XY coordinate array must match the number of traces")

        current_xy = np.asarray(xy_points, dtype=float)
        current_values = np.asarray(layer_values, dtype=float)
        current_xy, current_values = _bin_trace_slice_points(
            xy_points=current_xy,
            values=current_values,
            bins_per_mark=bins_per_mark,
            bin_reduce=bin_reduce,
        )

        if amplitude_mode == "absolute":
            current_values = np.abs(current_values)
        elif amplitude_mode != "signed":
            raise ValueError("amplitude_mode must be one of {'signed', 'absolute'}")

        xy_chunks.append(current_xy)
        value_chunks.append(current_values)

    if not value_chunks:
        return None, None
    return np.vstack(xy_chunks), np.concatenate(value_chunks)


def _resolve_xy_limits(xy_points_list, x_limits=None, y_limits=None, padding=0.0):
    if x_limits is not None and y_limits is not None:
        return tuple(map(float, x_limits)), tuple(map(float, y_limits))

    all_points = np.vstack([np.asarray(points, dtype=float) for points in xy_points_list])
    x_values = all_points[:, 0]
    y_values = all_points[:, 1]

    if x_limits is None:
        x_min, x_max = float(np.min(x_values)), float(np.max(x_values))
        x_range = x_max - x_min
        if x_range == 0.0:
            x_range = 1.0
        x_limits = (x_min - padding * x_range, x_max + padding * x_range)
    if y_limits is None:
        y_min, y_max = float(np.min(y_values)), float(np.max(y_values))
        y_range = y_max - y_min
        if y_range == 0.0:
            y_range = 1.0
        y_limits = (y_min - padding * y_range, y_max + padding * y_range)

    return tuple(map(float, x_limits)), tuple(map(float, y_limits))


def _resolve_grid_from_cell_size(x_limits, y_limits, grid_cell_size=None, grid_size=DEFAULT_GRID_SIZE):
    """Resolve the XY grid using either GPR-SLICE-like cell size or explicit grid size."""
    if grid_cell_size is not None:
        cell_size = float(grid_cell_size)
        if cell_size <= 0:
            raise ValueError("grid_cell_size must be > 0")
        x_span = max(float(x_limits[1]) - float(x_limits[0]), cell_size)
        y_span = max(float(y_limits[1]) - float(y_limits[0]), cell_size)
        grid_nx = max(2, int(np.floor(x_span / cell_size)) + 1)
        grid_ny = max(2, int(np.floor(y_span / cell_size)) + 1)
        return grid_nx, grid_ny, cell_size

    if isinstance(grid_size, int):
        grid_size = (grid_size, grid_size)
    if len(grid_size) != 2:
        raise ValueError("grid_size must be an int or a tuple like (nx, ny)")
    grid_nx = max(2, int(grid_size[0]))
    grid_ny = max(2, int(grid_size[1]))
    return grid_nx, grid_ny, None


def _build_search_mask(dx, dy, search_type="elliptical", search_radius_x=None, search_radius_y=None):
    """Build the interpolation neighborhood mask.

    - elliptical: (dx/rx)^2 + (dy/ry)^2 <= 1
    - circular: dx^2 + dy^2 <= r^2
    - rectangular: |dx| <= rx and |dy| <= ry
    - none/global: all points are valid
    """
    search_type = str(search_type).lower()
    if search_type in {"none", "global", "all"}:
        return np.ones_like(dx, dtype=bool)

    if search_radius_x is None and search_radius_y is None:
        return np.ones_like(dx, dtype=bool)

    if search_radius_x is None:
        search_radius_x = float(search_radius_y)
    if search_radius_y is None:
        search_radius_y = float(search_radius_x)

    rx = float(search_radius_x)
    ry = float(search_radius_y)
    if rx <= 0 or ry <= 0:
        raise ValueError("search_radius_x and search_radius_y must be > 0")

    if search_type == "elliptical":
        return ((dx / rx) ** 2 + (dy / ry) ** 2) <= 1.0
    if search_type == "circular":
        radius = min(rx, ry)
        return (dx ** 2 + dy ** 2) <= (radius ** 2)
    if search_type == "rectangular":
        return (np.abs(dx) <= rx) & (np.abs(dy) <= ry)

    raise ValueError("search_type must be one of {'elliptical', 'circular', 'rectangular', 'none'}")


def _idw_interpolate_grid(
    xy_points,
    values,
    x_grid,
    y_grid,
    power=DEFAULT_IDW_POWER,
    max_neighbors=None,
    search_type=DEFAULT_SEARCH_TYPE,
    search_radius_x=None,
    search_radius_y=None,
    fill_strategy="nearest",
    eps=1e-12,
    chunk_size=2048,
):
    """IDW interpolation with optional GPR-SLICE-like neighborhood search."""
    xy_points = np.asarray(xy_points, dtype=float)
    values = np.asarray(values, dtype=float)
    target_points = np.column_stack([x_grid.ravel(), y_grid.ravel()])
    interpolated = np.empty(target_points.shape[0], dtype=float)

    if max_neighbors is not None:
        max_neighbors = max(1, min(int(max_neighbors), xy_points.shape[0]))

    for start in range(0, target_points.shape[0], chunk_size):
        stop = min(start + chunk_size, target_points.shape[0])
        chunk_points = target_points[start:stop]
        deltas = chunk_points[:, None, :] - xy_points[None, :, :]
        dx = deltas[:, :, 0]
        dy = deltas[:, :, 1]
        distances = np.sqrt(dx ** 2 + dy ** 2)

        neighbor_mask = _build_search_mask(
            dx=dx,
            dy=dy,
            search_type=search_type,
            search_radius_x=search_radius_x,
            search_radius_y=search_radius_y,
        )

        if max_neighbors is not None and max_neighbors < xy_points.shape[0]:
            nearest_idx = np.argpartition(distances, kth=max_neighbors - 1, axis=1)[:, :max_neighbors]
            limited_mask = np.zeros_like(neighbor_mask, dtype=bool)
            row_idx = np.arange(nearest_idx.shape[0])[:, None]
            limited_mask[row_idx, nearest_idx] = True
            neighbor_mask &= limited_mask

        has_neighbors = np.any(neighbor_mask, axis=1)
        exact_mask = (distances <= eps) & neighbor_mask
        safe_distances = np.where(exact_mask, eps, distances)
        weights = np.where(neighbor_mask, 1.0 / (safe_distances ** float(power)), 0.0)

        weight_sum = np.sum(weights, axis=1)
        chunk_values = np.full(chunk_points.shape[0], np.nan, dtype=float)
        valid_rows = has_neighbors & (weight_sum > 0)
        if np.any(valid_rows):
            chunk_values[valid_rows] = (weights[valid_rows] @ values) / weight_sum[valid_rows]

        if np.any(exact_mask):
            exact_rows = np.any(exact_mask, axis=1)
            exact_cols = np.argmax(exact_mask[exact_rows], axis=1)
            chunk_values[exact_rows] = values[exact_cols]

        missing_rows = ~np.isfinite(chunk_values)
        if np.any(missing_rows):
            if fill_strategy == "nearest":
                nearest_global = np.argmin(distances[missing_rows], axis=1)
                chunk_values[missing_rows] = values[nearest_global]
            elif fill_strategy == "nan":
                pass
            else:
                raise ValueError("fill_strategy must be one of {'nearest', 'nan'}")

        interpolated[start:stop] = chunk_values

    return interpolated.reshape(x_grid.shape)


def _build_xy_slice_grid(
    xy_points,
    values,
    grid_size=DEFAULT_GRID_SIZE,
    grid_cell_size=None,
    x_limits=None,
    y_limits=None,
    interp_method="idw",
    idw_power=DEFAULT_IDW_POWER,
    idw_max_neighbors=None,
    search_type=DEFAULT_SEARCH_TYPE,
    search_radius_x=None,
    search_radius_y=None,
    idw_fill_strategy="nearest",
):
    """Interpolate one slice on a regular Cartesian XY grid."""
    x_limits = tuple(map(float, x_limits))
    y_limits = tuple(map(float, y_limits))
    grid_nx, grid_ny, used_cell_size = _resolve_grid_from_cell_size(
        x_limits=x_limits,
        y_limits=y_limits,
        grid_cell_size=grid_cell_size,
        grid_size=grid_size,
    )

    x_axis = np.linspace(x_limits[0], x_limits[1], grid_nx)
    y_axis = np.linspace(y_limits[0], y_limits[1], grid_ny)
    x_grid, y_grid = np.meshgrid(x_axis, y_axis)

    if interp_method == "idw":
        grid_values = _idw_interpolate_grid(
            xy_points=np.asarray(xy_points, dtype=float),
            values=np.asarray(values, dtype=float),
            x_grid=x_grid,
            y_grid=y_grid,
            power=idw_power,
            max_neighbors=idw_max_neighbors,
            search_type=search_type,
            search_radius_x=search_radius_x,
            search_radius_y=search_radius_y,
            fill_strategy=idw_fill_strategy,
        )
    elif interp_method in {"linear", "nearest", "cubic"}:
        grid_values = griddata(
            np.asarray(xy_points, dtype=float),
            np.asarray(values, dtype=float),
            (x_grid, y_grid),
            method=interp_method,
        )
    else:
        raise ValueError("interp_method must be one of {'idw', 'linear', 'nearest', 'cubic'}")

    return x_grid, y_grid, np.asarray(grid_values, dtype=float), used_cell_size


def _resolve_xy_color_limits(color_scale, vmin, vmax, grid_cache, sample_idx=None):
    if color_scale == "manual":
        if vmin is None or vmax is None:
            raise ValueError("For color_scale='manual', both vmin and vmax must be provided")
        return float(vmin), float(vmax)

    if color_scale == "slice":
        slice_grid = grid_cache.get(sample_idx)
        if slice_grid is None:
            raise ValueError(f"No valid interpolated grid found for sample {sample_idx}")
        valid_values = slice_grid["grid_values"][np.isfinite(slice_grid["grid_values"])]
    elif color_scale == "global":
        valid_arrays = [
            grid["grid_values"][np.isfinite(grid["grid_values"])]
            for grid in grid_cache.values()
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


def _plot_xy_slice_on_axis(
    ax,
    slice_grid,
    cmap,
    log_scale=False,
    color_scale="global",
    vmin=None,
    vmax=None,
    sample_idx=None,
    grid_cache=None,
    shared_color_limits=None,
):
    """Draw one interpolated XY slice on a standard Cartesian axis."""
    if color_scale == "global" and shared_color_limits is not None:
        current_vmin, current_vmax = shared_color_limits
    else:
        current_vmin, current_vmax = _resolve_xy_color_limits(
            color_scale=color_scale,
            vmin=vmin,
            vmax=vmax,
            grid_cache=grid_cache,
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
        slice_grid["x_grid"],
        slice_grid["y_grid"],
        slice_grid["grid_values"],
        **mesh_kwargs,
    )
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal")
    return mesh


def plot_xy_maps_for_depths(
    file_paths=None,
    samples=None,
    *,
    n_slices=None,
    sample_start=None,
    sample_end=None,
    metadata_list=None,
    data_list=None,
    xy_points_list=None,
    preprocess_edits=None,
    extra_changes=None,
    preview_data=False,
    preview_file_idx=0,
    preview_save_to=None,
    preview_sample_lines=True,
    cmap="viridis",
    preview_cmap="gray",
    n_cols=5,
    save_to=None,
    log_scale=False,
    color_scale="global",
    amplitude_mode="signed",
    vmin=None,
    vmax=None,
    slice_mode="point",
    slice_thickness=0,
    slice_reduce="mean",
    bins_per_mark=1,
    bin_reduce="mean",
    dpi=150,
    per_subplot_size=(3.0, 3.0),
    max_figsize=None,
    grid_size=DEFAULT_GRID_SIZE,
    grid_cell_size=None,
    x_limits=None,
    y_limits=None,
    interp_method="idw",
    idw_power=DEFAULT_IDW_POWER,
    idw_max_neighbors=None,
    search_type=DEFAULT_SEARCH_TYPE,
    search_radius_x=None,
    search_radius_y=None,
    idw_fill_strategy="nearest",
):
    """Plot interpolated XY maps for one or more depth slices.

    Parameters are designed to resemble a GPR-SLICE workflow more closely:
    - grid_cell_size: physical cell size of the output grid
    - search_type/search_radius_x/search_radius_y: GPR-SLICE-like IDW neighborhood
    - bins_per_mark: average consecutive traces before XY interpolation
    - amplitude_mode='absolute' + color_scale='global': similar to absolute global coloring
    """
    preprocess_pipeline = _merge_preprocess_edits(
        preprocess_edits=preprocess_edits,
        extra_changes=extra_changes,
    )

    if file_paths is not None:
        metadata_list, data_list, xy_points_list = _load_xy_inputs(
            file_paths=file_paths,
            xy_points_list=xy_points_list,
            preprocess_edits=preprocess_pipeline,
        )
    elif metadata_list is None or data_list is None or xy_points_list is None:
        raise ValueError(
            "Provide either file_paths, or metadata_list/data_list/xy_points_list explicitly"
        )

    samples = _resolve_samples(
        samples=samples,
        n_slices=n_slices,
        sample_start=sample_start,
        sample_end=sample_end,
        data_list=data_list,
    )

    if file_paths is not None and preview_data:
        preview_indices = _resolve_preview_file_indices(preview_file_idx, len(file_paths))
        for preview_idx in preview_indices:
            preview_xy_map_selection(
                file_path=file_paths[preview_idx],
                preprocess_edits=preprocess_pipeline,
                sample_lines=samples if preview_sample_lines else None,
                save_to=preview_save_to,
                cmap=preview_cmap,
                log_scale=log_scale,
                dpi=dpi,
                xy_points=None if xy_points_list is None else xy_points_list[preview_idx],
            )

    x_limits, y_limits = _resolve_xy_limits(
        xy_points_list=xy_points_list,
        x_limits=x_limits,
        y_limits=y_limits,
    )

    grid_cache = {}
    for sample_idx in samples:
        slice_xy_points, slice_values = _build_xy_slice_points(
            data_list=data_list,
            xy_points_list=xy_points_list,
            sample_idx=sample_idx,
            slice_mode=slice_mode,
            slice_thickness=slice_thickness,
            slice_reduce=slice_reduce,
            bins_per_mark=bins_per_mark,
            bin_reduce=bin_reduce,
            amplitude_mode=amplitude_mode,
        )
        if slice_values is None:
            grid_cache[sample_idx] = None
            continue

        x_grid, y_grid, grid_values, used_cell_size = _build_xy_slice_grid(
            xy_points=slice_xy_points,
            values=slice_values,
            grid_size=grid_size,
            grid_cell_size=grid_cell_size,
            x_limits=x_limits,
            y_limits=y_limits,
            interp_method=interp_method,
            idw_power=idw_power,
            idw_max_neighbors=idw_max_neighbors,
            search_type=search_type,
            search_radius_x=search_radius_x,
            search_radius_y=search_radius_y,
            idw_fill_strategy=idw_fill_strategy,
        )
        grid_cache[sample_idx] = {
            "x_grid": x_grid,
            "y_grid": y_grid,
            "grid_values": grid_values,
            "cell_size": used_cell_size,
        }

    shared_color_limits = None
    if color_scale == "global":
        shared_color_limits = _resolve_xy_color_limits(
            color_scale=color_scale,
            vmin=vmin,
            vmax=vmax,
            grid_cache=grid_cache,
        )

    n_rows = int(np.ceil(len(samples) / n_cols))
    fig_width = n_cols * per_subplot_size[0]
    fig_height = n_rows * per_subplot_size[1]
    if max_figsize is not None:
        fig_width = min(fig_width, max_figsize[0])
        fig_height = min(fig_height, max_figsize[1])

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(fig_width, fig_height), dpi=dpi)
    axes = np.atleast_1d(axes).flatten()

    mesh = None
    for plot_idx, sample_idx in enumerate(samples):
        slice_grid = grid_cache.get(sample_idx)
        if slice_grid is None:
            continue

        mesh = _plot_xy_slice_on_axis(
            ax=axes[plot_idx],
            slice_grid=slice_grid,
            cmap=cmap,
            log_scale=log_scale,
            color_scale=color_scale,
            vmin=vmin,
            vmax=vmax,
            sample_idx=sample_idx,
            grid_cache=grid_cache,
            shared_color_limits=shared_color_limits,
        )

        title = f"Sample {sample_idx}"
        if slice_mode != "point":
            total_thickness = 2 * int(slice_thickness) + 1
            title += f"\nband={total_thickness} ({slice_reduce})"
        if bins_per_mark > 1:
            title += f"\nbins={int(bins_per_mark)}"
        axes[plot_idx].set_title(title, fontsize=8)

    for ax in axes[len(samples):]:
        fig.delaxes(ax)

    if mesh is not None:
        cbar_ax = fig.add_axes([0.84, 0.15, 0.02, 0.7])
        cbar_label = "Absolute amplitude" if amplitude_mode == "absolute" else "Amplitude"
        fig.colorbar(mesh, cax=cbar_ax, orientation="vertical", label=cbar_label)

    fig.subplots_adjust(left=0.08, right=0.82, top=0.9, bottom=0.1)
    if save_to:
        plt.savefig(save_to, dpi=dpi, bbox_inches="tight")
        plt.close(fig)
        return save_to

    plt.show()
    return None


if __name__ == "__main__":
    # Example GPR-SLICE-like configuration
    plot_xy_maps_for_depths(
        file_paths=[],
        xy_points_list=[],
        preprocess_edits=[
            ("background_removal", {}),
            ("gain", {"gain_db": 10}),
        ],
        sample_start=23,
        n_slices=30,
        slice_mode="band",
        slice_thickness=24,          # 48 total samples ~= +/-24 around the center
        slice_reduce="gprslice",
        bins_per_mark=4,
        amplitude_mode="absolute",
        color_scale="global",
        interp_method="idw",
        grid_cell_size=0.01,
        search_type="elliptical",
        search_radius_x=0.25,
        search_radius_y=0.08,
        idw_power=2,
    )
