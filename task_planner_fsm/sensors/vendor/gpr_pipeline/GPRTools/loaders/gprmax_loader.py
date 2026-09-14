import os
import numpy as np
import h5py


class GPRmaxLoader:
    """
    Loader para B-scans de gprMax generados con el archivo:
    
        <base>_merged.out  (HDF5)

    Uso:
        loader = GPRmaxLoader(r"ruta/base_sin_extension")
        M = loader.load_matrix(component="Ez", rx=1)
    """

    def __init__(self, base_file_name):
        # El usuario puede pasar "carpeta/modelo" y buscaremos modelo_merged.out
        self.base_file_name = base_file_name
        self.out_path = self.base_file_name + "_merged.out"

        self.base_dir = os.path.dirname(self.base_file_name)
        self.file_name = os.path.basename(self.base_file_name)

    # ------------------------------------------------------------------
    def load_matrix(self, component="Ez", rx=1):
        """
        Devuelve M con forma (n_samples, n_traces)

        component: 'Ex', 'Ey', 'Ez' ...
        rx: número de receptor (1-indexed)
        """
        if not os.path.exists(self.out_path):
            raise FileNotFoundError(f"Archivo gprMax no encontrado: {self.out_path}")

        dataset_name = f"/rxs/rx{rx}/{component}"

        try:
            with h5py.File(self.out_path, "r") as f:
                if dataset_name not in f:
                    raise KeyError(f"Dataset no encontrado: {dataset_name}")

                data = f[dataset_name][:]
        except Exception as e:
            raise RuntimeError(f"Error leyendo {self.out_path}: {e}")

        # data viene como [n_traces, n_samples] → transponemos
        if data.ndim == 2:
            M = data.T.astype(float)
        else:
            raise ValueError(f"Formato inesperado en {dataset_name}: shape={data.shape}")

        return M.T

    # ------------------------------------------------------------------
    def get_dt(self):
        """Devuelve dt almacenado en el archivo HDF5."""
        try:
            with h5py.File(self.out_path, "r") as f:
                return f.attrs["dt"]
        except:
            return None
