import numpy as np
from obspy.io.segy.segy import _read_segy

class SegyLoader:
    def __init__(self, filepath):
        self.filepath = filepath

    def load_matrix(self):
        segy = _read_segy(self.filepath)
        M = np.stack([tr.data for tr in segy.traces], axis=1)
        return M