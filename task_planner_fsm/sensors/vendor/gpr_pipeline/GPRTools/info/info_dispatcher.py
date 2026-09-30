# ================================================================
# info_dispatcher.py — choose correct metadata extractor
# ================================================================
# Imports are intentionally lazy: SEGY processing should not require optional
# dependencies for DZT/GPRMax merely by importing the dispatcher.


def extract_metadata(file_path, loader_type):
    """Dispatch metadata extraction for the requested loader type."""
    if loader_type == "SEGY":
        from .segy_info import extract_metadata_segy_single
        return extract_metadata_segy_single(file_path)

    if loader_type == "SEGY_MESH":
        from .segy_mesh_info import extract_metadata_segy_mesh
        return extract_metadata_segy_mesh(file_path)

    if loader_type == "MALA":
        from .mala_info import extract_metadata_mala
        return extract_metadata_mala(file_path)

    if loader_type == "DZT":
        from .dzt_info import extract_metadata_dzt
        return extract_metadata_dzt(file_path)

    if loader_type == "GPRMAX":
        from .gprmax_info import extract_metadata_gprmax
        return extract_metadata_gprmax(file_path)

    return {"trace_distance": None, "time_window": None}
