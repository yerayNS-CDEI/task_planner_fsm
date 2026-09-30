import numpy as np

def background_removal(M):
    """
    M is (n_samples x n_traces)
    background = mean over columns
    """
    M = np.asarray(M, dtype=np.float64)
    bg = np.mean(M, axis=1, keepdims=True)
    return M - bg
