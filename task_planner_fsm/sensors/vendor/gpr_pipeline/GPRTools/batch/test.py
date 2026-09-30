import os
from GPRTools.loaders.loader_dispatcher import detect_loader_type, load_matrix_by_type

full_path = r"X:\2-projectes\1-Recerca_robotics\GPR\1. Real data\real_data_construction\2026_03_12_C_FME\raw_data\SEGY_20260312_131356\CDEI_column_meetingroom_horizontal001_20260312_111757"

print("full_path:", full_path)
print("exists:", os.path.exists(full_path))

loader_type = detect_loader_type(full_path)
print("loader_type:", loader_type)

entries = load_matrix_by_type(full_path, loader_type)
print("entries:", type(entries), len(entries))

if entries:
    for raw_path, M in entries[:1]:
        print("raw_path:", raw_path)
        print("shape:", getattr(M, "shape", None))