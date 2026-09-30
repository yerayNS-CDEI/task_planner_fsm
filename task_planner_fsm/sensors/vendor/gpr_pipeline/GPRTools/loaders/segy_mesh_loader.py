import os
import glob
import re

class SegyMeshLoader:
    def __init__(self, folder):
        self.folder = folder
        self.files = sorted(glob.glob(os.path.join(folder, "*.sgy")))
        if len(self.files) < 2:
            raise ValueError("Not a SEGY mesh (need ≥2 .sgy files).")

    def list_bscans(self):
        """
        Returns list of (line_id, path)
        Example: [("L001", "…_L001_…sgy"), ("L002", "…_L002_…sgy"), …]
        """
        out = []
        for f in self.files:
            name = os.path.basename(f)
            m = re.search(r"_L(\d{3})_", name)
            if m:
                line_id = f"L{m.group(1)}"
                out.append((line_id, f))
        return out