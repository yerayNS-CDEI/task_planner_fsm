import os
import struct
import numpy as np


class DZTLoader:
    """
    Loader nativo para archivos DZT (Sensors & Software).

    Este loader:

    - Detecta automáticamente endianness ('<' o '>')
    - Detecta automáticamente el offset de cabecera (1024, 4096, 512)
    - Soporta 8/16/32 bits por muestra
    - Devuelve matriz lista para pipeline (n_samples x n_traces)
    """

    def __init__(self, base_file_name):
        """
        base_file_name puede ser:

        - Ruta completa a un archivo .DZT / .dzt
        - Ruta sin extensión (se buscan .DZT y .dzt)
        """
        base_abs = os.path.abspath(base_file_name)
        root, ext = os.path.splitext(base_abs)

        if ext.lower() == ".dzt":
            self.dzt_path = base_abs
            self.base_name = root
        else:
            # prueba ambas extensiones
            cand1 = base_abs + ".DZT"
            cand2 = base_abs + ".dzt"
            if os.path.exists(cand1):
                self.dzt_path = cand1
            elif os.path.exists(cand2):
                self.dzt_path = cand2
            else:
                raise FileNotFoundError(f"No existe archivo DZT: {cand1} ni {cand2}")

            self.base_name = base_abs

    # --------------------------------------------------------------------------
    #   HEADER PARSER
    # --------------------------------------------------------------------------
    def _read_header(self, f, endian):
        try:
            f.seek(0)
            h = {}
            h["rh_tag"]  = struct.unpack(endian + "h", f.read(2))[0]
            h["rh_data"] = struct.unpack(endian + "h", f.read(2))[0]
            h["rh_nsamp"] = struct.unpack(endian + "h", f.read(2))[0]
            h["rh_bits"]  = struct.unpack(endian + "h", f.read(2))[0]
            h["rh_zero"]  = struct.unpack(endian + "h", f.read(2))[0]
            h["rhf_sps"]  = struct.unpack(endian + "f", f.read(4))[0]
            h["rhf_spm"]  = struct.unpack(endian + "f", f.read(4))[0]
            h["rhf_mpm"]  = struct.unpack(endian + "f", f.read(4))[0]
            h["rhf_position"] = struct.unpack(endian + "f", f.read(4))[0]
            h["rhf_range"] = struct.unpack(endian + "f", f.read(4))[0]

            # file size
            cur = f.tell()
            f.seek(0, os.SEEK_END)
            h["file_size_bytes"] = f.tell()
            f.seek(cur)

            # sanity checks
            if not (16 <= h["rh_nsamp"] <= 65535):
                return False, h
            if h["rh_bits"] not in (8, 16, 32):
                return False, h

            return True, h

        except:
            return False, {}

    # --------------------------------------------------------------------------
    #   DTYPE DECISION
    # --------------------------------------------------------------------------
    def _pick_dtype(self, rh_bits):
        if rh_bits == 8:
            return np.uint8, True, 8
        if rh_bits == 16:
            return np.uint16, True, 16
        if rh_bits == 32:
            return np.int32, False, 32

        raise ValueError(f"Formato DZT no soportado: {rh_bits} bits")

    # --------------------------------------------------------------------------
    #   DATA READER
    # --------------------------------------------------------------------------
    def _read_data(self, header, endian, expected_traces=None, header_bytes=None):

        ns = header["rh_nsamp"]
        rh_bits = header["rh_bits"]
        dtype, needs_centering, bits = self._pick_dtype(rh_bits)
        bytes_per_sample = bits // 8

        raw = np.fromfile(self.dzt_path, dtype=dtype)

        # offsets a probar
        offsets = [1024, 4096, 512] if header_bytes is None else [header_bytes]

        total_words = raw.size

        for hb in offsets:

            header_words = hb // bytes_per_sample
            if header_words >= total_words:
                continue

            data_words = raw[header_words:]

            # n_traces detectado automáticamente
            if data_words.size % ns != 0:
                continue

            nt = data_words.size // ns
            M = data_words.reshape((nt, ns)).T  # (ns x nt)

            # centrar 8/16 bits
            if needs_centering:
                M = M.astype(np.float32)
                M -= (2 ** rh_bits) / 2

            return M

        raise ValueError("No se pudo leer la matriz DZT (offset incorrecto).")

    # --------------------------------------------------------------------------
    #   PUBLIC API
    # --------------------------------------------------------------------------
    def load_matrix(self):
        """
        Devuelve matriz (n_samples x n_traces)
        """

        if not os.path.exists(self.dzt_path):
            raise FileNotFoundError(f"No existe archivo DZT: {self.dzt_path}")

        # 1. Detectar endianness
        with open(self.dzt_path, "rb") as f:
            ok_le, hdr_le = self._read_header(f, "<")
            if ok_le:
                header = hdr_le
                endian = "<"
            else:
                ok_be, hdr_be = self._read_header(f, ">")
                if not ok_be:
                    raise ValueError("Header DZT inválido (ni < ni >)")
                header = hdr_be
                endian = ">"

        # 2. Leer matriz completa
        M = self._read_data(header, endian)

        return M
