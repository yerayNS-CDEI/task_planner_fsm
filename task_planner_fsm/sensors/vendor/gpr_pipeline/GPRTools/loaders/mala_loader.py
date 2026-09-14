import os
import numpy as np


class MalaLoader:
    """
    Loader sencillo para B-scans MALÅ (.rad + .rd3).

    Uso:
        loader = MalagsLoader(r"ruta/base_sin_extension")
        M = loader.load_matrix()   # M tiene forma (n_samples, n_traces)
    """

    def __init__(self, base_file_name, dtype=np.int16):
        # Acepta que el usuario pase o no extensión; nos quedamos con la base
        self.base_file_name = os.path.splitext(base_file_name)[0]
        self.dtype = dtype

        self.rad_path = self.base_file_name + ".rad"
        self.rd3_path = self.base_file_name + ".rd3"

    # ------------------------------------------------------------------
    # Lectura del .rad (cabecera de texto)
    # ------------------------------------------------------------------
    def _read_rad_metadata(self):
        metadata = {}

        # Algunos ficheros vienen como UTF-8 con BOM, otros como latin-1
        last_exception = None
        for enc in ("utf-8-sig", "latin-1"):
            try:
                with open(self.rad_path, "r", encoding=enc) as f:
                    for line in f:
                        if ":" in line:
                            key, value = line.strip().split(":", 1)
                            metadata[key.strip().upper()] = value.strip()
                break
            except Exception as e:
                last_exception = e
        else:
            raise IOError(f"Error reading {self.rad_path}: {last_exception}")

        # Campos típicos en MALÅ
        n_samples = int(metadata.get("SAMPLES", 0))
        n_traces = int(metadata.get("LAST TRACE", 0))

        if n_samples <= 0 or n_traces <= 0:
            raise ValueError(
                f"Invalid header values in {self.rad_path}: "
                f"SAMPLES={n_samples}, LAST TRACE={n_traces}"
            )

        return n_samples, n_traces, metadata

    # ------------------------------------------------------------------
    # Lectura del binario .rd3 y construcción de la matriz
    # ------------------------------------------------------------------
    def load_matrix(self, return_metadata: bool = False):
        """
        Devuelve la matriz M con forma (n_samples, n_traces).

        Si return_metadata=True, devuelve (M, metadata_dict).
        """
        n_samples, n_traces, metadata = self._read_rad_metadata()

        with open(self.rd3_path, "rb") as f:
            data = np.fromfile(f, dtype=self.dtype)

        expected = n_samples * n_traces
        if data.size != expected:
            raise ValueError(
                f"Unexpected data length in {self.rd3_path}: "
                f"read {data.size}, expected {expected} "
                f"(n_samples={n_samples}, n_traces={n_traces})"
            )

        # Reshape a (n_traces, n_samples) y transponer a (n_samples, n_traces)
        M = data.reshape((n_traces, n_samples)).T.astype(float)

        if return_metadata:
            return M, metadata
        return M
