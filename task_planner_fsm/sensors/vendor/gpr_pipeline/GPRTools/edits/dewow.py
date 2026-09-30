import numpy as np
import scipy.signal as sig


def dewow(M, window=50):
    """
    DEWOW por eliminación de componente de baja frecuencia.

    Args:
        M (ndarray): matriz (n_samples × n_traces)
        window (int): tamaño de ventana para suavizar (20–100 recomendado)

    Returns:
        M_dewow (ndarray): matriz filtrada
    """

    M = np.asarray(M, dtype=float)

    if window < 3:
        return M  # no filtrar si ventana absurda

    # ----------------------------------------------------------------------
    # 1) Para cada traza retiramos un suavizado (moving average)
    #    Esto es equivalente a un filtro paso alto (HP).
    # ----------------------------------------------------------------------
    kernel = np.ones(window) / window  # filtro de media móvil

    baseline = np.apply_along_axis(
        lambda tr: np.convolve(tr, kernel, mode='same'),
        axis=0,
        arr=M
    )

    M_hp = M - baseline

    # ----------------------------------------------------------------------
    # 2) NORMALIZAR LEVEMENTE PARA EVITAR SATURACIONES POR HP
    # ----------------------------------------------------------------------
    # Evita que el HP meta offset raro
    M_hp -= np.mean(M_hp, axis=0, keepdims=True)

    return M_hp
