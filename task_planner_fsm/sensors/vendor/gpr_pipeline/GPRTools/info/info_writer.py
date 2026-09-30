import os

def write_info_file(
    output_dir,
    base_name,
    M,
    *,
    trace_distance=None,
    time_window=None,
    dx=None,
    dt=None,
    n_samples_header=None,
    n_traces_header=None,
):
    os.makedirs(output_dir, exist_ok=True)

    txt_path = os.path.join(output_dir, f"{base_name}_info.txt")

    # ============================================================
    # Num samples / num traces
    # Preferir valores del header (caso DZT u otros),
    # si no vienen, usar siempre M.shape
    # ============================================================
    if n_samples_header is not None:
        num_samples = n_samples_header
    else:
        num_samples = M.shape[0]

    if n_traces_header is not None:
        num_traces = n_traces_header
    else:
        num_traces = M.shape[1]

    # ============================================================
    # dx y dt
    # Si no vienen desde metadata (info_*) se calculan de forma genérica:
    #   dt = time_window / num_samples
    #   dx = trace_distance / num_traces
    # ============================================================
    if dx is None and trace_distance is not None and num_traces:
        dx = trace_distance / num_traces

    if dt is None and time_window is not None and num_samples:
        dt = time_window / num_samples

    # ============================================================
    # Escritura de fichero
    # ============================================================
    with open(txt_path, "w") as f:
        f.write("=== B-SCAN METADATA ===\n\n")
        f.write(f"Num samples: {num_samples}\n")
        f.write(f"Num traces:  {num_traces}\n\n")
        f.write(f"Trace distance [m]: {trace_distance}\n")
        f.write(f"Time window [ns]:   {time_window}\n\n")
        f.write(f"dx [m]: {dx}\n")
        f.write(f"dt [ns]: {dt}\n")

    return txt_path
