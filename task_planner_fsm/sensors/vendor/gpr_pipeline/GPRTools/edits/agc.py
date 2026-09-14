import numpy as np
import scipy.signal as sig


def agc(M, window=40, eps=1e-6):
    """
    Automatic Gain Control (AGC).
    Normaliza cada muestra por la RMS local dentro de una ventana móvil.

    Args:
        M (ndarray): matriz (n_samples × n_traces)
        window (int): tamaño ventana RMS (20–80 recomendado)
        eps (float): anti-división por cero.

    Returns:
        ndarray: M_agc
    """

    M = np.asarray(M, dtype=float)

    if window < 3:
        return M.copy()

    # Cuadrado de amplitudes
    M2 = M ** 2

    # RMS por ventana móvil (1D filter aplicado por columnas)
    kernel = np.ones(window) / window

    rms = np.apply_along_axis(
        lambda tr: np.sqrt(np.convolve(tr, kernel, mode="same") + eps),
        axis=0,
        arr=M2
    )

    # Normalizar señal
    M_agc = M / rms

    return M_agc
