#!/usr/bin/env python3
"""
Clasificación de muestras con un modelo XGBoost pre-entrenado (.joblib)
=========================================================================

Este script:
  1. Carga un modelo XGBoost ya entrenado desde un archivo .joblib
     (debe ser un XGBClassifier de la API de scikit-learn).
  2. Carga un CSV con las muestras a clasificar.
  3. Detecta automáticamente qué columnas son features (usa las que el
     modelo espera; si el modelo no las expone, usa todas las columnas
     numéricas del CSV que no sean metadatos típicos como
     Date/Time/Label/Class).
  4. Aplica el mismo FILTRO DE CALIDAD ESPECTRAL usado al preparar los
     datos de entrenamiento (segmentación VIS/SWIR, imputación de valores
     faltantes, recorte de reflectancia, control físico y detección de
     picos/ruido). Las muestras que no lo superan NO se clasifican: se
     marcan con el motivo del descarte.
  5. A las muestras que sí pasan el filtro les aplica la misma
     TRANSFORMACIÓN DE FEATURES 'SNV+D1' usada al entrenar el modelo
     (corrección de bandas de artefacto, suavizado VIS, SNV por segmento
     y primera derivada por segmento).
  6. Genera una predicción por cada muestra transformada, junto con su
     nivel de confianza (la probabilidad máxima entre todas las clases).
  7. Si esa confianza está por debajo del umbral indicado, la predicción
     se sustituye por "no se ha podido detectar".

USO BÁSICO
----------
    python predict.py --csv datos.csv --model modelo.joblib --confidence 0.8

Esto imprime los resultados por pantalla. Para guardarlos en un CSV:

    python predict.py --csv datos.csv --model modelo.joblib --confidence 0.8 --output resultados.csv

Para desactivar el filtro de calidad (clasificar todas las filas tal cual
vienen en el CSV, sin filtrar ni imputar nada):

    python predict.py --csv datos.csv --model modelo.joblib --skip-quality-filter

SOBRE EL FILTRO DE CALIDAD
---------------------------
El filtro solo puede aplicarse si las columnas de features son longitudes
de onda numéricas (p. ej. "400.5", "402.1"...), igual que en el dataset de
entrenamiento (LENZ). Si el modelo expone columnas no numéricas, el script
avisa y clasifica sin filtrar.

Una muestra se descarta (no se clasifica) si:
  - Alguno de sus segmentos de sensor (VIS o SWIR) está incompleto
    (demasiados valores faltantes o un hueco demasiado largo).
  - No supera el control de calidad físico (rango, media, máximo o mínimo
    de reflectancia fuera de los umbrales esperados).
  - Contiene un pico puntual (spike) o ruido excesivo.

Las muestras que sí pasan el filtro se imputan (valores faltantes
residuales interpolados, pequeños excesos de reflectancia recortados a
1.0) exactamente igual que se hizo con los datos de entrenamiento, antes
de pasarlas al modelo.

SOBRE LOS NOMBRES DE CLASE (IMPORTANTE)
----------------------------------------
Muchos flujos de trabajo con XGBoost entrenan el modelo tras convertir las
etiquetas de texto (p. ej. "wood", "metal"...) a números con un
LabelEncoder de scikit-learn, porque XGBoost lo exige en algunas versiones.
Este script contempla varios escenarios, de más a menos automático:

  1. El .joblib guarda directamente el clasificador (XGBClassifier) y este
     ya conoce los nombres originales de las clases (atributo `classes_`
     con texto). No hay que hacer nada más.
  2. El .joblib guarda un diccionario con el modelo y el encoder juntos,
     por ejemplo: joblib.dump({"model": modelo, "label_encoder": le}, ...).
     El script los detecta automáticamente (prueba las claves
     "model"/"modelo"/"clf" y "label_encoder"/"le").
  3. El modelo y el LabelEncoder se guardaron en dos archivos .joblib
     distintos. En ese caso, indica el segundo con --label-encoder:

       python predict.py --csv datos.csv --model modelo.joblib \
           --label-encoder encoder.joblib --confidence 0.8

  4. El modelo solo predice índices numéricos y no hay ningún LabelEncoder
     guardado. En ese caso se usa el mapeo clase->índice que ya conoces
     (por defecto, el de este proyecto: brick=0, cementitious=1, ceramic=2,
     gypsum=3, metal=4, polymer=5, wood=6). Si algún día cambia, se puede
     sobrescribir con --class-mapping, aceptando una ruta a .json o un
     JSON inline:

       python predict.py --csv datos.csv --model modelo.joblib \
           --class-mapping '{"brick":0,"cementitious":1,"ceramic":2,"gypsum":3,"metal":4,"polymer":5,"wood":6}'

SOBRE LA TRANSFORMACIÓN DE FEATURES (SNV + D1) Y EL ESCALADO
---------------------------------------------------------------
El modelo no se entrenó con la reflectancia cruda, sino con el conjunto de
features 'SNV+D1': corrección de bandas de artefacto puntual, suavizado
Savitzky-Golay ligero en el VIS, Standard Normal Variate (SNV) por segmento
de sensor y, por último, primera derivada (Savitzky-Golay) por segmento.
El script aplica esa misma transformación, en el mismo orden, después del
filtro de calidad y antes de llamar a predict_proba.

Además, esas features SNV+D1 NO se pasaron "crudas" al clasificador: se
normalizaron con un StandardScaler ajustado sobre los datos de
entrenamiento. Si el .joblib incluye ese scaler (clave "scaler"), el script
lo aplica automáticamente (solo .transform(), nunca se reajusta) justo
después de SNV+D1 y antes de predict_proba. Si el .joblib guarda también el
array de longitudes de onda de entrenamiento (clave "wl"), se usa para
seleccionar exactamente esas mismas bandas del CSV nuevo, en el mismo orden,
en vez de recalcularlas por nombre de columna.

REQUISITOS
----------
    pip install pandas numpy scipy scikit-learn xgboost joblib
"""

import argparse
import json
import sys
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
import joblib
from scipy.signal import savgol_filter


# Columnas que, si existen en el CSV, se consideran metadatos (no features).
# Se ignoran automáticamente a la hora de predecir.
COLUMNAS_METADATO_CANDIDATAS = {"Measure Type", "Date", "Time", "Counter", "Label", "Class"}

# Mapeo de clase -> índice usado por este modelo (confirmado por el usuario).
# Se usa como valor por defecto cuando el modelo solo predice índices
# numéricos (0, 1, 2...) y no lleva un LabelEncoder incluido. Se puede
# sobrescribir con --class-mapping si en el futuro cambia.
CLASS_MAPPING_DEFAULT = {
    "brick": 0,
    "cementitious": 1,
    "ceramic": 2,
    "gypsum": 3,
    "metal": 4,
    "polymer": 5,
    "wood": 6,
}

# ── Umbrales del filtro de calidad espectral (mismos que en el pipeline de
#    entrenamiento, ver filtro_calidad_lenz.py) ──────────────────────────────
WL_MIN               = 400.0   # nm — mismo recorte de borde UV usado en entrenamiento
WL_MAX               = 1650.0  # nm — mismo recorte de borde SWIR usado en entrenamiento
VALID_MEAN_MIN      = 0.005   # media mínima de señal aceptable
R_VALID_MIN         = -0.5    # elimina artefactos severos; el ruido normal se conserva
R_RANGE_MIN         = 0.03    # espectros planos (max-min por debajo de esto) no aportan información
ROUGHNESS_THRESHOLD = 1.0     # umbral de "rugosidad" para considerar un espectro ruidoso
SENSOR_GAP_NM       = 50.0    # salto de longitud de onda mayor a esto = frontera entre sensores
SEG_MISSING_MAX     = 0.15    # segmento con >=15% de valores faltantes se considera "muerto"
SEG_GAP_MAX         = 10      # hueco contiguo de más de 10 bandas no se interpola (segmento muerto)
R_CLIP_MAX          = 1.05    # exceso leve (1.0, 1.05] -> se recorta a 1.0 (ruido de calibración)
R_HARD_MAX          = 1.20    # reflectancia por encima de esto -> se descarta (artefacto real)


# ─────────────────────────────────────────────────────────────────────────
# FILTRO DE CALIDAD ESPECTRAL
# (idéntico en lógica al usado para limpiar los datos de entrenamiento)
# ─────────────────────────────────────────────────────────────────────────

def sensor_segments(wl: np.ndarray, gap_nm: float = SENSOR_GAP_NM):
    """
    Devuelve una lista de pares (inicio, fin) de índices [inicio, fin)
    para cada segmento contiguo de sensor, cortando donde el salto entre
    longitudes de onda consecutivas supera gap_nm (separa VIS de SWIR).
    """
    breaks = np.where(np.diff(wl) > gap_nm)[0]
    bounds = [0] + [b + 1 for b in breaks] + [len(wl)]
    return [(bounds[i], bounds[i + 1]) for i in range(len(bounds) - 1)]


def has_spike(row: np.ndarray, wl: np.ndarray,
              spike_z: float = 8.0, consec_bands: int = 2) -> bool:
    """True si la muestra contiene un artefacto de tipo pico de una sola banda."""
    v = row.astype(float)
    for s, e in sensor_segments(wl):
        seg = v[s:e]
        if len(seg) < 5:
            continue
        d = np.diff(seg)
        std_d = np.nanstd(d)
        if std_d < 1e-10:
            continue
        z = np.abs(d) / std_d
        for i in range(len(d) - consec_bands):
            if z[i] > spike_z:
                reversal = any(
                    np.sign(d[i + k]) != np.sign(d[i]) and z[i + k] > spike_z * 0.5
                    for k in range(1, consec_bands + 1)
                )
                if reversal:
                    return True
    return False


def is_noisy(row: np.ndarray, wl: np.ndarray, noise_z: float = 4.0) -> bool:
    """True si la rugosidad media del espectro supera ROUGHNESS_THRESHOLD."""
    v = row.astype(float)
    roughness_scores = []
    for s, e in sensor_segments(wl):
        seg = v[s:e]
        if len(seg) < 5:
            continue
        d = np.abs(np.diff(seg))
        mean_sig = np.nanmean(np.abs(seg)) + 1e-10
        roughness_scores.append(float(np.nanmean(d) / mean_sig))
    if not roughness_scores:
        return False
    return float(np.mean(roughness_scores)) > ROUGHNESS_THRESHOLD


def _max_gap(nanmask: np.ndarray) -> int:
    """Racha contigua más larga de True (valores faltantes) en una máscara 1D."""
    best = run = 0
    for x in nanmask:
        run = run + 1 if x else 0
        if run > best:
            best = run
    return best


def aplicar_filtro_calidad(X: np.ndarray, wl: np.ndarray):
    """
    Aplica el filtro de calidad espectral a un array de muestras nuevas
    (mismo criterio que en el entrenamiento). No reentrena ni recalcula
    umbrales: solo aplica los mismos ya fijados arriba.

    Devuelve:
        X_filtrado : array con ceros->NaN corregidos, huecos interpolados
                     y pequeños excesos de reflectancia recortados a 1.0
                     (igual en las filas válidas y no válidas; en las no
                     válidas puede seguir teniendo NaN si el segmento
                     estaba muerto).
        valid_mask : array booleano, True si la muestra supera el filtro.
        motivos    : lista de strings con el motivo del descarte
                     ("" si la muestra es válida).
    """
    X = X.copy()
    n = len(X)
    X = np.where(X == 0, np.nan, X)
    segments = sensor_segments(wl)

    # (1) Gate de completitud por segmento
    gate = np.ones(n, dtype=bool)
    motivos = np.array([""] * n, dtype=object)
    for s, e in segments:
        seg_nan = np.isnan(X[:, s:e])
        miss = seg_nan.mean(axis=1)
        gaps = np.array([_max_gap(seg_nan[k]) for k in range(n)])
        seg_bad = (miss >= SEG_MISSING_MAX) | (gaps > SEG_GAP_MAX)
        gate &= ~seg_bad
        for k in np.where(seg_bad)[0]:
            if not motivos[k]:
                motivos[k] = "segmento de sensor incompleto (demasiados valores faltantes o hueco largo)"

    # (2) Interpolación intra-segmento (solo filas que pasan el gate)
    for s, e in segments:
        w = wl[s:e]
        for k in np.where(gate)[0]:
            v = X[k, s:e]
            nanmask = np.isnan(v)
            if nanmask.any() and (~nanmask).sum() >= 2:
                v[nanmask] = np.interp(w[nanmask], w[~nanmask], v[~nanmask])
                X[k, s:e] = v

    # (3) Recorte de exceso leve de reflectancia
    X[(X > 1.0) & (X <= R_CLIP_MAX)] = 1.0

    # (4) Control de calidad físico
    with np.errstate(all="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        vmean = np.nanmean(X, axis=1)
        vmax = np.nanmax(X, axis=1)
        vmin = np.nanmin(X, axis=1)

    ok_phys = (
        (vmax <= R_HARD_MAX) &
        (vmin > R_VALID_MIN) &
        (vmean > VALID_MEAN_MIN) &
        ((vmax - vmin) > R_RANGE_MIN)
    )
    for k in np.where(gate & ~ok_phys)[0]:
        motivos[k] = "control de calidad físico (rango, media o máximo de reflectancia fuera de umbral)"

    # (5) Picos / ruido, evaluados solo sobre candidatas restantes
    cand = gate & ok_phys
    spike = np.array([has_spike(X[k], wl) if cand[k] else False for k in range(n)])
    noisy = np.array([is_noisy(X[k], wl) if cand[k] else False for k in range(n)])
    for k in np.where(cand & spike)[0]:
        motivos[k] = "pico puntual (spike) detectado en el espectro"
    for k in np.where(cand & noisy & ~spike)[0]:
        motivos[k] = "espectro demasiado ruidoso"

    valid_mask = cand & ~spike & ~noisy
    return X, valid_mask, list(motivos)


def obtener_wavelengths(features):
    """
    Intenta interpretar los nombres de las columnas de features como
    longitudes de onda numéricas (necesario para poder aplicar el filtro
    de calidad y la transformación de features, que se basan en segmentar
    por sensor VIS/SWIR).
    Devuelve un array de floats, o None si alguna columna no es numérica.
    """
    try:
        return np.array([float(c) for c in features])
    except (TypeError, ValueError):
        return None


# ─────────────────────────────────────────────────────────────────────────
# TRANSFORMACIÓN DE FEATURES 'SNV+D1'
# (idéntica en lógica a la usada al entrenar: corrección de bandas de
#  artefacto, suavizado VIS, SNV por segmento y primera derivada por segmento)
# ─────────────────────────────────────────────────────────────────────────

# Bandas con artefacto puntual conocido (picos/hundimientos de un solo
# canal, confirmados en el entrenamiento). Se corrigen por interpolación
# lineal usando las bandas vecinas dentro del mismo segmento de sensor.
ARTIFACT_BANDS_NM = [
    411.4, 435.3, 444.4,                      # borde UV (región ya ruidosa)
    994.0, 1089.0, 1345.0, 1435.0, 1477.0,    # SWIR (región espectralmente suave)
]


def correct_artifact_bands(X: np.ndarray, wl: np.ndarray,
                            artifact_wls=ARTIFACT_BANDS_NM,
                            half_window: int = 3) -> np.ndarray:
    """
    Interpola linealmente, muestra a muestra, las bandas de artefacto
    conocidas, usando las bandas vecinas (dentro del mismo segmento de
    sensor, excluyendo el propio objetivo y otras bandas de artefacto).
    """
    X_out = X.copy()
    art_idx = {int(np.argmin(np.abs(wl - t))) for t in artifact_wls}
    segs = sensor_segments(wl)

    def seg_of(i):
        for s, e in segs:
            if s <= i < e:
                return s, e
        return 0, len(wl)

    for target in artifact_wls:
        idx = int(np.argmin(np.abs(wl - target)))
        s, e = seg_of(idx)
        lo = max(s, idx - half_window)
        hi = min(e - 1, idx + half_window)
        neigh = [j for j in range(lo, hi + 1) if j != idx and j not in art_idx]
        if len(neigh) < 2:
            continue
        wl_n = wl[neigh]
        for k in range(len(X_out)):
            X_out[k, idx] = float(np.interp(wl[idx], wl_n, X_out[k, neigh]))
    return X_out


def smooth_vis_dientes(X: np.ndarray, wl: np.ndarray,
                        window: int = 15, poly: int = 2) -> np.ndarray:
    """
    Suaviza artefactos del VIS con un Savitzky-Golay ligero, aplicado
    SOLO al segmento VIS (el SWIR ya se corrige con interpolación de
    bandas puntuales en correct_artifact_bands).
    """
    out = X.copy()
    segs = sensor_segments(wl)
    vis_s, vis_e = segs[0]                    # primer segmento = VIS
    block = X[:, vis_s:vis_e]
    n = block.shape[1]
    win = min(window, n if n % 2 == 1 else n - 1)
    if win > poly:
        out[:, vis_s:vis_e] = savgol_filter(block, window_length=win,
                                             polyorder=poly, axis=1)
    return out


def snv(X: np.ndarray) -> np.ndarray:
    """Standard Normal Variate sobre un bloque contiguo (un solo segmento de sensor)."""
    mu = np.nanmean(X, axis=1, keepdims=True)
    std = np.nanstd(X, axis=1, keepdims=True)
    std[std == 0] = 1.0
    return (X - mu) / std


def snv_segmented(X: np.ndarray, wl: np.ndarray) -> np.ndarray:
    """SNV aplicado por segmento de sensor y luego concatenado."""
    out = np.empty_like(X, dtype=float)
    for s, e in sensor_segments(wl):
        out[:, s:e] = snv(X[:, s:e])
    return out


def _sg_block(X, wl_block, order, window, poly):
    """Savitzky-Golay sobre un bloque; ajusta la ventana si el bloque es corto."""
    n = X.shape[1]
    win = min(window, n if n % 2 == 1 else n - 1)   # ventana impar y <= nº de bandas
    if win <= poly:                                  # bloque demasiado corto
        return np.zeros_like(X) if order > 0 else X.copy()
    delta = float(np.median(np.diff(wl_block))) if n > 1 else 1.0
    return savgol_filter(X, window_length=win, polyorder=poly,
                          deriv=order, delta=delta, axis=1)


def sg_deriv_segmented(X: np.ndarray, wl: np.ndarray,
                        order: int = 1, window: int = 11, poly: int = 2) -> np.ndarray:
    """Savitzky-Golay (suavizado o derivada) aplicado por segmento de sensor."""
    out = np.empty_like(X, dtype=float)
    for s, e in sensor_segments(wl):
        out[:, s:e] = _sg_block(X[:, s:e], wl[s:e], order, window, poly)
    return out


def calcular_snv_d1(X: np.ndarray, wl: np.ndarray) -> np.ndarray:
    """
    Reproduce el conjunto de features 'SNV+D1' usado al entrenar el modelo,
    en el mismo orden:
      1. Corrige bandas de artefacto puntual (interpolación banda a banda).
      2. Suaviza artefactos del VIS con Savitzky-Golay ligero.
      3. Aplica SNV por segmento de sensor.
      4. Aplica la primera derivada (Savitzky-Golay) por segmento de sensor.
    Debe aplicarse DESPUÉS del filtro de calidad (sobre datos ya sin
    ceros/NaN) y ANTES de llamar a predict_proba.
    """
    X = correct_artifact_bands(X, wl)
    X = smooth_vis_dientes(X, wl)
    X = snv_segmented(X, wl)
    X = sg_deriv_segmented(X, wl, order=1)
    return X


# ─────────────────────────────────────────────────────────────────────────
# CARGA DE MODELO Y METADATOS DE CLASE
# ─────────────────────────────────────────────────────────────────────────

def cargar_modelo(ruta_modelo: str, ruta_label_encoder: str = None):
    """
    Carga el modelo .joblib desde disco. Soporta tanto un .joblib que
    contenga directamente el clasificador, como uno que contenga un
    diccionario {"model": ..., "label_encoder": ...} (patrón habitual).

    Además del clasificador y el label encoder, si el .joblib es un
    diccionario también se extraen (cuando existen):
      - "scaler": el StandardScaler (u objeto con .transform) que se
        ajustó sobre las features SNV+D1 de entrenamiento. El modelo NO
        se entrenó con las features SNV+D1 "crudas", sino con esas
        features ya escaladas por este scaler, así que hay que aplicarlo
        también en inferencia o las predicciones son básicamente ruido.
      - "wl": el array de longitudes de onda (float) exactamente en el
        orden usado al entrenar. Si está disponible, se usa para
        seleccionar/ordenar las columnas del CSV nuevo en vez de
        recalcularlas desde los nombres de columna, evitando
        desajustes de orden o de bandas.
      - "feat": el nombre del conjunto de features con el que se
        entrenó (se espera "SNV+D1"; si es otro, se avisa).

    Devuelve (modelo, label_encoder_o_None, scaler_o_None, wl_o_None,
    feat_o_None).
    """
    ruta = Path(ruta_modelo)
    if not ruta.exists():
        sys.exit(f"ERROR: no se encuentra el archivo del modelo: {ruta_modelo}")
    objeto = joblib.load(ruta)

    label_encoder = None
    scaler = None
    wl_guardado = None
    feat_guardado = None
    if isinstance(objeto, dict):
        modelo = objeto.get("model") or objeto.get("modelo") or objeto.get("clf")
        if modelo is None:
            sys.exit(
                "ERROR: el .joblib contiene un diccionario pero no se "
                "encuentra el clasificador (se buscaron las claves "
                "'model', 'modelo' o 'clf'). Claves disponibles: "
                f"{list(objeto.keys())}"
            )
        label_encoder = objeto.get("label_encoder") or objeto.get("le")
        scaler = objeto.get("scaler")
        wl_guardado = objeto.get("wl")
        feat_guardado = objeto.get("feat")
    else:
        modelo = objeto

    if ruta_label_encoder:
        ruta_le = Path(ruta_label_encoder)
        if not ruta_le.exists():
            sys.exit(f"ERROR: no se encuentra el label encoder: {ruta_label_encoder}")
        label_encoder = joblib.load(ruta_le)

    if scaler is not None and not hasattr(scaler, "transform"):
        print(
            "AVISO: el .joblib contiene una clave 'scaler' pero el objeto "
            "no tiene método .transform(); se ignora.",
            file=sys.stderr,
        )
        scaler = None

    if feat_guardado and feat_guardado != "SNV+D1":
        print(
            f"AVISO: el modelo se entrenó con el conjunto de features "
            f"'{feat_guardado}', pero este script solo sabe reproducir "
            "'SNV+D1'. Las predicciones pueden ser incorrectas.",
            file=sys.stderr,
        )

    return modelo, label_encoder, scaler, wl_guardado, feat_guardado


def obtener_features_desde_wl_guardado(wl_guardado, columnas_csv, tolerancia=1e-6):
    """
    Si el .joblib guarda el array 'wl' de longitudes de onda exactamente
    como se usaron en el entrenamiento, esta función localiza, para cada
    valor de wl_guardado, la columna del CSV nuevo cuyo nombre (interpretado
    como float) coincide dentro de una tolerancia. Esto reproduce el orden y
    el subconjunto EXACTOS de bandas de entrenamiento, con independencia de
    en qué orden vengan las columnas en el CSV nuevo o de si trae bandas de
    más.
    Devuelve (nombres_columnas, wl) o (None, None) si falta alguna banda.
    """
    if wl_guardado is None:
        return None, None

    col_por_valor = []
    for c in columnas_csv:
        try:
            col_por_valor.append((float(c), c))
        except (TypeError, ValueError):
            continue

    nombres = []
    faltantes = []
    for objetivo in wl_guardado:
        candidatas = [c for v, c in col_por_valor if abs(v - objetivo) <= tolerancia]
        if not candidatas:
            faltantes.append(objetivo)
        else:
            nombres.append(candidatas[0])

    if faltantes:
        print(
            "AVISO: el CSV no está en la misma rejilla de bandas que el "
            f"entrenamiento ({len(faltantes)} de {len(wl_guardado)} bandas no "
            "coinciden exactamente). Se usarán las bandas nativas del CSV y "
            "se reproyectarán después a la rejilla del modelo.",
            file=sys.stderr,
        )
        return None, None

    return nombres, np.array(wl_guardado, dtype=float)


def obtener_features_esperadas(modelo, columnas_csv):
    """
    Averigua qué columnas de features espera el modelo (en el orden correcto).
    Si el modelo no expone esa información, usa como fallback todas las
    columnas del CSV que no sean metadatos conocidos.
    """
    nombres = None

    # XGBClassifier moderno (API sklearn) guarda esto tras el fit()
    if hasattr(modelo, "feature_names_in_"):
        nombres = list(modelo.feature_names_in_)
    # Alternativa: nombres guardados en el Booster interno
    elif hasattr(modelo, "get_booster"):
        try:
            nombres = modelo.get_booster().feature_names
        except Exception:
            nombres = None

    if nombres:
        faltantes = [c for c in nombres if c not in columnas_csv]
        if faltantes:
            sys.exit(
                "ERROR: el CSV no contiene todas las columnas que el modelo "
                f"espera. Faltan {len(faltantes)} columnas de features, por "
                f"ejemplo: {faltantes[:5]}"
            )
        return nombres

    # Fallback: el modelo no expone sus nombres de features. Se intenta
    # primero replicar el recorte espectral usado en entrenamiento
    # (columnas numéricas = longitudes de onda dentro de [WL_MIN, WL_MAX]),
    # porque un CSV nuevo puede traer más bandas de las que el sensor
    # entregó al modelo (bordes UV/SWIR ya descartados al entrenar).
    candidatas_wl = []
    for c in columnas_csv:
        if c in COLUMNAS_METADATO_CANDIDATAS:
            continue
        try:
            valor = float(c)
        except (TypeError, ValueError):
            continue
        if WL_MIN <= valor <= WL_MAX:
            candidatas_wl.append(c)

    if candidatas_wl:
        print(
            "AVISO: el modelo no especifica los nombres de sus features; se "
            f"están usando las {len(candidatas_wl)} columnas cuyo nombre es "
            f"una longitud de onda entre {WL_MIN} y {WL_MAX} nm (mismo "
            "recorte aplicado en el entrenamiento).",
            file=sys.stderr,
        )
        return candidatas_wl

    # Si ninguna columna es interpretable como longitud de onda, se usa
    # como último recurso todo lo que no sea metadato conocido.
    candidatas = [c for c in columnas_csv if c not in COLUMNAS_METADATO_CANDIDATAS]
    if not candidatas:
        sys.exit("ERROR: no se han podido identificar columnas de features en el CSV.")
    print(
        "AVISO: el modelo no especifica los nombres de sus features y "
        "ninguna columna es interpretable como longitud de onda; se están "
        f"usando las {len(candidatas)} columnas no-metadato del CSV tal cual.",
        file=sys.stderr,
    )
    return candidatas


def cargar_mapeo_clases(valor: str = None):
    """
    Devuelve un dict {nombre_clase: indice}. Si no se indica --class-mapping,
    se usa CLASS_MAPPING_DEFAULT (definido arriba). Si se indica, puede ser
    una ruta a un .json o un JSON inline en la propia línea de comandos.
    """
    if valor is None:
        return CLASS_MAPPING_DEFAULT

    ruta = Path(valor)
    if ruta.exists():
        with open(ruta, "r", encoding="utf-8") as f:
            mapeo = json.load(f)
    else:
        try:
            mapeo = json.loads(valor)
        except json.JSONDecodeError:
            sys.exit(
                "ERROR: --class-mapping debe ser una ruta a un .json válido "
                'o un JSON inline, ej: \'{"brick":0,"wood":6}\''
            )
    return {str(k): int(v) for k, v in mapeo.items()}


def obtener_nombres_clases(modelo, label_encoder, mapeo_clases, n_clases):
    """
    Devuelve los nombres de clase en el mismo orden que usa predict_proba.
    Prioridad: LabelEncoder explícito > mapeo de clases (--class-mapping o
    el valor por defecto) > classes_ del propio modelo (si son texto) >
    índices numéricos como último recurso.
    """
    if label_encoder is not None and hasattr(label_encoder, "classes_"):
        return [str(c) for c in label_encoder.classes_]

    if mapeo_clases:
        if len(mapeo_clases) != n_clases:
            print(
                f"AVISO: el mapeo de clases tiene {len(mapeo_clases)} "
                f"clases, pero el modelo predice {n_clases}. Se ignora el "
                "mapeo y se usa el siguiente método disponible.",
                file=sys.stderr,
            )
        else:
            nombres = [None] * n_clases
            for nombre, idx in mapeo_clases.items():
                if not (0 <= idx < n_clases):
                    sys.exit(
                        f"ERROR: el índice {idx} de la clase '{nombre}' está "
                        f"fuera de rango (0-{n_clases - 1})."
                    )
                nombres[idx] = nombre
            if all(n is not None for n in nombres):
                return nombres

    if hasattr(modelo, "classes_"):
        clases = list(modelo.classes_)
        # Si classes_ ya son texto (no números), se pueden usar directamente.
        if not all(isinstance(c, (int, np.integer, float, np.floating)) for c in clases):
            return [str(c) for c in clases]

    print(
        "AVISO: no se ha encontrado ningún mapeo de clases a texto (ni "
        "label_encoder, ni --class-mapping, ni classes_ con nombres). Se "
        "mostrarán índices numéricos.",
        file=sys.stderr,
    )
    return [str(i) for i in range(n_clases)]


# ─────────────────────────────────────────────────────────────────────────
# PIPELINE PRINCIPAL: filtro de calidad + predicción
# ─────────────────────────────────────────────────────────────────────────

def remuestrear_a_grid_entrenamiento(X, wl_origen, wl_destino):
    """
    Reproyecta los espectros de la rejilla de longitudes de onda del CSV
    (wl_origen) a la rejilla EXACTA con la que se entrenó el modelo
    (wl_destino), mediante interpolación lineal.

    Esto hace falta porque distintos equipos/calibraciones del mismo sensor
    no entregan exactamente las mismas bandas: p.ej. el CSV puede muestrear
    el VIS en 401.32, 403.22, 405.11... mientras que el entrenamiento usó
    400.43, 402.26, 404.09... Aunque ambos cubran 400-1650 nm, ni el número
    de bandas ni sus posiciones coinciden, así que no se pueden emparejar
    columna a columna: hay que interpolar.

    La interpolación se hace POR SEGMENTO de sensor (VIS y SWIR por
    separado), igual que el resto del preprocesado, para no interpolar
    nunca a través del hueco entre sensores. Cualquier banda de destino que
    caiga fuera del rango cubierto por su segmento de origen se rellena con
    el valor del extremo más cercano de ese segmento (sin extrapolar).
    """
    X = np.asarray(X, dtype=float)
    wl_origen = np.asarray(wl_origen, dtype=float)
    wl_destino = np.asarray(wl_destino, dtype=float)

    seg_origen = sensor_segments(wl_origen)
    seg_destino = sensor_segments(wl_destino)

    if len(seg_origen) != len(seg_destino):
        sys.exit(
            "ERROR: el CSV tiene "
            f"{len(seg_origen)} segmento(s) de sensor y el modelo se entrenó "
            f"con {len(seg_destino)}. No se puede reproyectar el espectro de "
            "forma fiable. Revisa que el CSV cubra el mismo rango VIS+SWIR."
        )

    out = np.empty((X.shape[0], len(wl_destino)), dtype=float)
    for (so, eo), (sd, ed) in zip(seg_origen, seg_destino):
        wl_o = wl_origen[so:eo]
        wl_d = wl_destino[sd:ed]
        bloque = X[:, so:eo]
        for k in range(X.shape[0]):
            fila = bloque[k]
            finitos = np.isfinite(fila)
            if finitos.sum() < 2:
                out[k, sd:ed] = np.nan
                continue
            out[k, sd:ed] = np.interp(wl_d, wl_o[finitos], fila[finitos])
    return out


def predecir(csv_path: str, model_path: str, confianza_minima: float,
             output_path: str = None, columnas_a_conservar=None,
             label_encoder_path: str = None, class_mapping_arg: str = None,
             aplicar_filtro: bool = True, aplicar_transform: bool = True):
    """Ejecuta el filtro de calidad + la transformación SNV+D1 + la clasificación,
    y devuelve un DataFrame con los resultados."""

    if not (0 < confianza_minima <= 1):
        sys.exit("ERROR: --confidence debe estar entre 0 y 1 (ej. 0.8 para el 80%).")

    modelo, label_encoder, scaler, wl_guardado, feat_guardado = cargar_modelo(
        model_path, label_encoder_path
    )
    mapeo_clases = cargar_mapeo_clases(class_mapping_arg)

    ruta_csv = Path(csv_path)
    if not ruta_csv.exists():
        sys.exit(f"ERROR: no se encuentra el archivo CSV: {csv_path}")
    df = pd.read_csv(ruta_csv)

    if not hasattr(modelo, "predict_proba"):
        sys.exit(
            "ERROR: el modelo cargado no dispone de 'predict_proba'. Se "
            "necesita un XGBClassifier (API de scikit-learn) entrenado para "
            "poder aplicar un umbral de confianza."
        )

    # Preferir el 'wl' exacto guardado en el .joblib (mismas bandas y mismo
    # orden que en el entrenamiento). Si no está disponible o no encaja con
    # el CSV, se recurre al método anterior (detectar por nombre de columna).
    features, wl = obtener_features_desde_wl_guardado(wl_guardado, df.columns.tolist())
    if features is None:
        features = obtener_features_esperadas(modelo, df.columns.tolist())

    X = df[features].apply(pd.to_numeric, errors="coerce").values.astype(float)
    n_muestras = len(df)

    # wl (longitudes de onda) hace falta tanto para el filtro de calidad
    # como para la transformación SNV+D1: ambos segmentan por sensor VIS/SWIR.
    necesita_wl = aplicar_filtro or aplicar_transform
    if wl is None and necesita_wl:
        wl = obtener_wavelengths(features)
    if necesita_wl and wl is None:
        print(
            "AVISO: las columnas de features no son longitudes de onda "
            "numéricas; no se pueden aplicar ni el filtro de calidad ni la "
            "transformación SNV+D1. Se clasificará la reflectancia cruda "
            "sin filtrar.",
            file=sys.stderr,
        )

    # ── Filtro de calidad espectral ─────────────────────────────────────────
    if aplicar_filtro and wl is not None:
        X, valid_mask, motivos = aplicar_filtro_calidad(X, wl)
        n_descartadas = int((~valid_mask).sum())
        if n_descartadas:
            print(
                f"Filtro de calidad: {n_descartadas} / {n_muestras} muestras "
                "descartadas (no se clasifican).",
                file=sys.stderr,
            )
    else:
        valid_mask = np.ones(n_muestras, dtype=bool)
        motivos = [""] * n_muestras
        # XGBoost maneja de forma nativa los valores NaN, no hace falta imputar.

    # ── Predicción (solo sobre las muestras que pasan el filtro) ───────────
    clase_predicha = [None] * n_muestras
    confianza_predicha = np.full(n_muestras, np.nan)

    idx_validas = np.where(valid_mask)[0]
    if len(idx_validas) > 0:
        X_validas = X[idx_validas]

        # ── Reproyección a la rejilla de bandas del entrenamiento ────────────
        # El filtro de calidad se aplica sobre la rejilla nativa del CSV (que
        # es donde los huecos/dropouts del sensor tienen sentido). Pero el
        # modelo espera exactamente las bandas de wl_guardado. Si el CSV
        # viene en otra rejilla (otra calibración del equipo), se interpola
        # aquí, por segmento, antes de calcular SNV+D1.
        if wl_guardado is not None and wl is not None:
            wl_modelo = np.asarray(wl_guardado, dtype=float)
            if len(wl) != len(wl_modelo) or not np.allclose(wl, wl_modelo, atol=1e-6):
                print(
                    f"AVISO: el CSV tiene {len(wl)} bandas en 400-1650 nm y el "
                    f"modelo se entrenó con {len(wl_modelo)}, o las posiciones "
                    "no coinciden. Se reproyectan los espectros a la rejilla "
                    "de entrenamiento por interpolación lineal (por segmento).",
                    file=sys.stderr,
                )
                X_validas = remuestrear_a_grid_entrenamiento(
                    X_validas, wl, wl_modelo
                )
                wl = wl_modelo

        # ── Transformación de features SNV+D1 (mismo pipeline del entrenamiento) ──
        if aplicar_transform and wl is not None:
            X_validas = calcular_snv_d1(X_validas, wl)

        # ── Escalado (StandardScaler) ────────────────────────────────────────
        # El modelo NO se entrenó sobre las features SNV+D1 crudas: se
        # entrenó sobre esas features ya normalizadas con un StandardScaler
        # ajustado en entrenamiento (guardado junto al modelo en el .joblib,
        # bajo la clave "scaler"). Sin este paso el modelo recibe datos en
        # una escala completamente distinta a la que aprendió y las
        # predicciones no son fiables. Se aplica scaler.transform() (nunca
        # fit_transform) para no volver a ajustar el escalador con los datos
        # nuevos.
        if scaler is not None:
            X_validas = scaler.transform(X_validas)
        elif aplicar_transform:
            print(
                "AVISO: el .joblib no incluye un 'scaler'. Si el modelo se "
                "entrenó sobre features SNV+D1 escaladas (StandardScaler), "
                "las predicciones sin ese escalado no serán fiables.",
                file=sys.stderr,
            )

        probabilidades = modelo.predict_proba(X_validas)
        nombres_clases = obtener_nombres_clases(
            modelo, label_encoder, mapeo_clases, probabilidades.shape[1]
        )
        idx_predicho = np.argmax(probabilidades, axis=1)
        conf_predicha = probabilidades[np.arange(len(idx_validas)), idx_predicho]
        for pos, i in enumerate(idx_validas):
            clase_predicha[i] = nombres_clases[idx_predicho[pos]]
            confianza_predicha[i] = conf_predicha[pos]

    prediccion_final = []
    for i in range(n_muestras):
        if not valid_mask[i]:
            prediccion_final.append(f"muestra descartada por filtro de calidad: {motivos[i]}")
        elif confianza_predicha[i] >= confianza_minima:
            prediccion_final.append(clase_predicha[i])
        else:
            prediccion_final.append("no se ha podido detectar")

    resultados = pd.DataFrame({
        "clase_predicha": clase_predicha,
        "confianza": np.round(confianza_predicha, 4),
        "filtro_calidad": ["OK" if v else motivos[i] for i, v in enumerate(valid_mask)],
        "prediccion_final": prediccion_final,
    })

    # Antepone columnas originales que se quieran conservar como referencia
    # (solo si existen en el CSV; si no existen, se ignoran sin dar error).
    if columnas_a_conservar:
        existentes = [c for c in columnas_a_conservar if c in df.columns]
        if existentes:
            resultados = pd.concat(
                [df[existentes].reset_index(drop=True), resultados], axis=1
            )

    if output_path:
        resultados.to_csv(output_path, index=False)
        print(f"Resultados guardados en: {output_path}")

    return resultados


def main():
    parser = argparse.ArgumentParser(
        description="Clasifica muestras de un CSV con un modelo XGBoost "
                     "pre-entrenado (.joblib), aplicando primero el filtro "
                     "de calidad espectral y después un umbral de confianza."
    )
    parser.add_argument("--csv", required=True,
                         help="Ruta al CSV con las muestras a clasificar.")
    parser.add_argument("--model", required=True,
                         help="Ruta al modelo .joblib ya entrenado.")
    parser.add_argument("--label-encoder", default=None,
                         help="Ruta a un LabelEncoder .joblib guardado aparte, "
                              "si tus clases se codificaron como números al "
                              "entrenar (ver cabecera del script para más info).")
    parser.add_argument("--class-mapping", default=None,
                         help="Ruta a un .json o JSON inline con el mapeo "
                              "clase->índice, ej: '{\"brick\":0,\"wood\":6}'. "
                              "Si no se indica, se usa el mapeo por defecto "
                              "ya configurado en el script (ver cabecera).")
    parser.add_argument("--confidence", type=float, default=0.8,
                         help="Umbral mínimo de confianza, entre 0 y 1. "
                              "Por defecto 0.8 (80%%).")
    parser.add_argument("--output", default=None,
                         help="Ruta donde guardar el CSV de resultados. "
                              "Si no se indica, solo se imprime por pantalla.")
    parser.add_argument("--keep-columns", nargs="*",
                         default=["Measure Type", "Date", "Time", "Counter", "Label", "Class"],
                         help="Columnas originales del CSV a mantener en el "
                              "resultado como referencia, si existen "
                              "(por defecto: Measure Type, Date, Time, Counter, Label, Class).")
    parser.add_argument("--skip-quality-filter", action="store_true",
                         help="Desactiva el filtro de calidad espectral: "
                              "clasifica todas las filas del CSV tal cual, "
                              "sin descartar ni imputar nada.")
    args = parser.parse_args()

    resultados = predecir(
        csv_path=args.csv,
        model_path=args.model,
        confianza_minima=args.confidence,
        output_path=args.output,
        columnas_a_conservar=args.keep_columns,
        label_encoder_path=args.label_encoder,
        class_mapping_arg=args.class_mapping,
        aplicar_filtro=not args.skip_quality_filter
    )

    with pd.option_context("display.max_rows", 50, "display.width", 120):
        print(resultados.to_string(index=False))


if __name__ == "__main__":
    main()
