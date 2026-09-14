import numpy as np

def trim(M, t1, t2):
    """
    Trim using original sample indices.
    """
    return np.asarray(M)[t1:t2, :]
