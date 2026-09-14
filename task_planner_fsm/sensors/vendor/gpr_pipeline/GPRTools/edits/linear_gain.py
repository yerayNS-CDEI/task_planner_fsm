import numpy as np

def linear_gain_db(M, gain_db=20):
    """
    Linear increasing gain in dB across samples.
    """
    M = np.asarray(M, dtype=np.float64)
    n = M.shape[0]
    gain_vec = 10 ** (np.linspace(0, gain_db, n).reshape(-1,1) / 20)
    return M * gain_vec