from pathlib import Path
import csv
import math

import cv2
import numpy as np


# ============================================================
# CONFIGURACION
# ============================================================

CARPETA_RAIZ = Path(
    r"X:\2-projectes\1-Recerca_robotics\GPR\3. IA_models\Line_Segmentation\Data"
)

CARPETA_SALIDA = Path(
    r"X:\2-projectes\1-Recerca_robotics\GPR\3. IA_models\Line_Segmentation\Results"
)

EXTENSIONES = {
    ".png",
    ".jpg",
    ".jpeg",
    ".tif",
    ".tiff",
    ".bmp",
}


# ============================================================
# PARAMETROS DEL DETECTOR
# ============================================================

CONTRASTE_MIN = 12

COBERTURA_BLOQUE_MIN = 0.45

SOPORTE_BLOQUES_MIN = 0.55

GROSOR_MAX_LINEA = 40

FUERZA_RELATIVA_BLOQUE = 0.25


# Una linea debe tener como minimo este porcentaje
# de la fuerza de la linea mas fuerte encontrada.
FUERZA_RELATIVA_LINEA_MIN = 0.55


# ============================================================
# NORMALIZACION DE RESOLUCION
# ============================================================

# Las imagenes mayores que esta altura se reducen
# proporcionalmente antes de hacer la deteccion.
#
# Las coordenadas encontradas se convierten despues
# al tamaño original.
ALTURA_ANALISIS_MAX = 2500


# ============================================================
# DIVISION EN BLOQUES
# ============================================================

ANCHO_BLOQUE_OBJETIVO = 70

NUM_BLOQUES_MIN = 4

NUM_BLOQUES_MAX = 16

ANCHO_BLOQUE_MIN = 20


# ============================================================
# TOLERANCIAS
# ============================================================

TOLERANCIA_Y = 4

MARGEN_BUSQUEDA = 6

ANGULO_MAX = 4.0

ERROR_RECTITUD_MAX = 5.0


# ============================================================
# DIBUJO
# ============================================================

DIBUJAR_SOLO_PRINCIPAL = False

DIBUJAR_PUNTOS_BLOQUES = True


# ============================================================
# LEER IMAGEN
# ============================================================

def leer_imagen_gris(ruta):

    try:

        datos = np.fromfile(
            str(ruta),
            dtype=np.uint8,
        )

        return cv2.imdecode(
            datos,
            cv2.IMREAD_GRAYSCALE,
        )

    except Exception:

        return None


# ============================================================
# GUARDAR PNG
# ============================================================

def guardar_png(ruta, imagen):

    ruta = Path(ruta)

    ruta.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    correcto, buffer = cv2.imencode(
        ".png",
        imagen,
    )

    if not correcto:

        raise RuntimeError(
            f"No se pudo guardar: {ruta}"
        )

    buffer.tofile(
        str(ruta)
    )


# ============================================================
# GUARDAR CSV
# ============================================================

def guardar_csv(ruta, columnas, filas):

    with open(
        ruta,
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as archivo:

        writer = csv.DictWriter(
            archivo,
            fieldnames=columnas,
        )

        writer.writeheader()

        writer.writerows(
            filas
        )


# ============================================================
# COMPROBAR SI UNA RUTA ESTA DENTRO DE OTRA
# ============================================================

def esta_dentro_de(ruta, carpeta):

    try:

        ruta.resolve().relative_to(
            carpeta.resolve()
        )

        return True

    except ValueError:

        return False


# ============================================================
# LINEA PRINCIPAL
# ============================================================

def obtener_linea_principal(lineas):

    return max(
        lineas,
        key=lambda linea: linea["fuerza"],
        default=None,
    )


# ============================================================
# METRICAS RELATIVAS (0-100)
# ============================================================

def coordenada_y_relativa(
    y,
    alto,
):

    # Convencion:
    # y = 0          -> 0.0
    # y = alto - 1   -> 100.0
    if alto <= 1:

        return 0.0

    valor = (
        float(y)
        /
        float(alto - 1)
        *
        100.0
    )

    return float(
        np.clip(
            valor,
            0.0,
            100.0,
        )
    )


def añadir_metricas_relativas(
    lineas,
    alto,
):

    # Las lineas ya llegan ordenadas por su coordenada y.
    # Se añaden las medidas tanto en pixeles como en escala
    # relativa 0-100 para que el CSV se pueda abrir en Excel.
    for linea in lineas:

        y0 = int(
            linea["region_y0"]
        )

        y1 = int(
            linea["region_y1"]
        )

        grosor_px = max(
            0,
            y1 - y0,
        )

        linea["y_relativa"] = (
            coordenada_y_relativa(
                linea["y"],
                alto,
            )
        )

        linea["limite_superior_y_relativa"] = (
            coordenada_y_relativa(
                y0,
                alto,
            )
        )

        linea["limite_inferior_y_relativa"] = (
            coordenada_y_relativa(
                y1,
                alto,
            )
        )

        linea["grosor_px"] = (
            grosor_px
        )

        if alto <= 1:

            linea["grosor_relativo"] = 0.0

        else:

            linea["grosor_relativo"] = (
                grosor_px
                /
                float(alto - 1)
                *
                100.0
            )

        # Para cada linea guardamos la distancia a la siguiente.
        # En la ultima linea queda vacia porque no hay otra debajo.
        linea["distancia_siguiente_px"] = ""

        linea[
            "distancia_siguiente_relativa"
        ] = ""

    for indice in range(
        len(lineas) - 1
    ):

        actual = lineas[indice]

        siguiente = lineas[
            indice + 1
        ]

        distancia_px = abs(
            int(siguiente["y"])
            -
            int(actual["y"])
        )

        actual[
            "distancia_siguiente_px"
        ] = distancia_px

        if alto <= 1:

            distancia_relativa = 0.0

        else:

            distancia_relativa = (
                distancia_px
                /
                float(alto - 1)
                *
                100.0
            )

        actual[
            "distancia_siguiente_relativa"
        ] = float(
            distancia_relativa
        )

    return lineas


# ============================================================
# NUMERO DE BLOQUES
# ============================================================

def calcular_numero_bloques(ancho):

    numero = int(
        round(
            ancho
            /
            ANCHO_BLOQUE_OBJETIVO
        )
    )

    numero = max(
        NUM_BLOQUES_MIN,
        numero,
    )

    numero = min(
        NUM_BLOQUES_MAX,
        numero,
    )

    max_por_ancho = max(
        1,
        ancho // ANCHO_BLOQUE_MIN,
    )

    numero = min(
        numero,
        max_por_ancho,
    )

    return max(
        1,
        numero,
    )


# ============================================================
# DIMENSIONES DE ANALISIS
# ============================================================

def calcular_dimensiones_analisis(
    alto,
    ancho,
):

    # Imagen suficientemente pequeña:
    # no modificarla.
    if alto <= ALTURA_ANALISIS_MAX:

        return (
            alto,
            ancho,
            1.0,
        )

    escala = (
        ALTURA_ANALISIS_MAX
        /
        alto
    )

    nuevo_alto = (
        ALTURA_ANALISIS_MAX
    )

    nuevo_ancho = max(
        1,
        int(
            round(
                ancho
                *
                escala
            )
        ),
    )

    return (
        nuevo_alto,
        nuevo_ancho,
        escala,
    )


# ============================================================
# PREPARAR IMAGEN PARA ANALISIS
# ============================================================

def preparar_imagen_analisis(
    gray_original,
):

    alto_original, ancho_original = (
        gray_original.shape
    )

    (
        alto_analisis,
        ancho_analisis,
        escala,
    ) = calcular_dimensiones_analisis(
        alto_original,
        ancho_original,
    )

    if escala == 1.0:

        return gray_original

    gray_analisis = cv2.resize(
        gray_original,
        (
            ancho_analisis,
            alto_analisis,
        ),
        interpolation=cv2.INTER_AREA,
    )

    return gray_analisis


# ============================================================
# AJUSTE ROBUSTO DE UNA RECTA
# ============================================================

def ajustar_recta_robusta(
    xs,
    ys,
    error_max,
):

    xs = np.asarray(
        xs,
        dtype=np.float64,
    )

    ys = np.asarray(
        ys,
        dtype=np.float64,
    )

    if len(xs) < 2:

        return None

    buenos = np.ones(
        len(xs),
        dtype=bool,
    )

    # Quitar puntos anomalos
    for _ in range(3):

        if buenos.sum() < 2:

            return None

        a, b = np.polyfit(
            xs[buenos],
            ys[buenos],
            1,
        )

        prediccion = (
            a * xs
            +
            b
        )

        residuos = np.abs(
            ys
            -
            prediccion
        )

        residuos_buenos = (
            residuos[buenos]
        )

        mediana = np.median(
            residuos_buenos
        )

        mad = np.median(
            np.abs(
                residuos_buenos
                -
                mediana
            )
        )

        mad = max(
            mad,
            1e-6,
        )

        limite_estadistico = (
            mediana
            +
            3.0
            *
            1.4826
            *
            mad
        )

        limite = max(
            error_max * 1.5,
            limite_estadistico,
        )

        nuevos_buenos = (
            residuos <= limite
        )

        if np.array_equal(
            buenos,
            nuevos_buenos,
        ):

            break

        buenos = nuevos_buenos

    if buenos.sum() < 2:

        return None

    # Ajuste final
    a, b = np.polyfit(
        xs[buenos],
        ys[buenos],
        1,
    )

    prediccion = (
        a * xs[buenos]
        +
        b
    )

    residuos = np.abs(
        ys[buenos]
        -
        prediccion
    )

    error = float(
        np.percentile(
            residuos,
            90,
        )
    )

    return {
        "a": float(a),
        "b": float(b),
        "buenos": buenos,
        "error": error,
    }


# ============================================================
# CONVERTIR RESULTADOS AL TAMAÑO ORIGINAL
# ============================================================

def convertir_lineas_a_original(
    lineas,
    alto_original,
    ancho_original,
    alto_analisis,
    ancho_analisis,
):

    escala_x = (
        ancho_analisis
        /
        ancho_original
    )

    escala_y = (
        alto_analisis
        /
        alto_original
    )

    # Si no se ha redimensionado, solamente
    # eliminamos las variables internas.
    if (
        alto_original == alto_analisis
        and
        ancho_original == ancho_analisis
    ):

        for linea in lineas:

            linea.pop(
                "_a_analisis",
                None,
            )

            linea.pop(
                "_b_analisis",
                None,
            )

        return lineas

    factor_x = (
        1.0
        /
        escala_x
    )

    factor_y = (
        1.0
        /
        escala_y
    )

    for linea in lineas:

        # ----------------------------------------------------
        # RECONSTRUIR RECTA EN COORDENADAS ORIGINALES
        # ----------------------------------------------------

        a_analisis = linea.pop(
            "_a_analisis"
        )

        b_analisis = linea.pop(
            "_b_analisis"
        )

        # y_analisis = a*x_analisis + b
        #
        # x_analisis = escala_x*x_original
        # y_analisis = escala_y*y_original
        #
        # por tanto:
        #
        # y_original =
        # a*(escala_x/escala_y)*x_original
        # + b/escala_y

        a_original = (
            a_analisis
            *
            escala_x
            /
            escala_y
        )

        b_original = (
            b_analisis
            /
            escala_y
        )

        linea["x1"] = 0

        linea["x2"] = (
            ancho_original - 1
        )

        linea["y1"] = int(
            round(
                b_original
            )
        )

        linea["y2"] = int(
            round(
                a_original
                *
                (ancho_original - 1)
                +
                b_original
            )
        )

        linea["y1"] = int(
            np.clip(
                linea["y1"],
                0,
                alto_original - 1,
            )
        )

        linea["y2"] = int(
            np.clip(
                linea["y2"],
                0,
                alto_original - 1,
            )
        )

        # Angulo real en la imagen original
        linea["angulo"] = float(
            np.degrees(
                np.arctan(
                    a_original
                )
            )
        )

        # ----------------------------------------------------
        # POSICION CENTRAL
        # ----------------------------------------------------

        linea["y"] = int(
            round(
                linea["y"]
                *
                factor_y
            )
        )

        linea["y"] = int(
            np.clip(
                linea["y"],
                0,
                alto_original - 1,
            )
        )

        # ----------------------------------------------------
        # ERROR DE RECTITUD EN PIXELES ORIGINALES
        # ----------------------------------------------------

        linea["error_rectitud"] = (
            linea["error_rectitud"]
            *
            factor_y
        )

        # ----------------------------------------------------
        # REGION CANDIDATA
        # ----------------------------------------------------

        linea["region_y0"] = int(
            round(
                linea["region_y0"]
                *
                factor_y
            )
        )

        linea["region_y1"] = int(
            round(
                linea["region_y1"]
                *
                factor_y
            )
        )

        linea["region_y0"] = int(
            np.clip(
                linea["region_y0"],
                0,
                alto_original - 1,
            )
        )

        linea["region_y1"] = int(
            np.clip(
                linea["region_y1"],
                0,
                alto_original - 1,
            )
        )

        # ----------------------------------------------------
        # PUNTOS DE LOS BLOQUES
        # ----------------------------------------------------

        puntos_originales = []

        for x, y in linea[
            "puntos_bloques"
        ]:

            x_original = int(
                round(
                    x
                    *
                    factor_x
                )
            )

            y_original = int(
                round(
                    y
                    *
                    factor_y
                )
            )

            x_original = int(
                np.clip(
                    x_original,
                    0,
                    ancho_original - 1,
                )
            )

            y_original = int(
                np.clip(
                    y_original,
                    0,
                    alto_original - 1,
                )
            )

            puntos_originales.append(
                (
                    x_original,
                    y_original,
                )
            )

        linea[
            "puntos_bloques"
        ] = puntos_originales

    return lineas


# ============================================================
# DETECTOR DE LINEAS NEGRAS HORIZONTALES
# ============================================================

def detectar_lineas_horizontales(
    ruta,
):

    # ========================================================
    # LEER IMAGEN ORIGINAL
    # ========================================================

    gray_original = leer_imagen_gris(
        ruta
    )

    if gray_original is None:

        raise ValueError(
            f"No se pudo leer la imagen: {ruta}"
        )

    alto_original, ancho_original = (
        gray_original.shape
    )

    # ========================================================
    # NORMALIZAR RESOLUCION
    # ========================================================

    gray = preparar_imagen_analisis(
        gray_original
    )

    alto, ancho = (
        gray.shape
    )

    # ========================================================
    # SUAVIZADO
    # ========================================================

    gray_suave = cv2.GaussianBlur(
        gray,
        (3, 3),
        0,
    )

    # ========================================================
    # FONDO LOCAL VERTICAL
    # ========================================================

    altura_kernel = (
        2
        *
        GROSOR_MAX_LINEA
        +
        1
    )

    altura_kernel = min(
        altura_kernel,
        alto,
    )

    if altura_kernel % 2 == 0:

        altura_kernel -= 1

    altura_kernel = max(
        3,
        altura_kernel,
    )

    kernel_vertical = (
        cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (
                1,
                altura_kernel,
            ),
        )
    )

    fondo = cv2.morphologyEx(
        gray_suave,
        cv2.MORPH_CLOSE,
        kernel_vertical,
    )

    # ========================================================
    # RESPUESTA A NEGRO
    # ========================================================

    respuesta = cv2.subtract(
        fondo,
        gray_suave,
    ).astype(
        np.float32
    )

    # ========================================================
    # DIVIDIR EN BLOQUES
    # ========================================================

    numero_bloques = (
        calcular_numero_bloques(
            ancho
        )
    )

    limites_x = np.linspace(
        0,
        ancho,
        numero_bloques + 1,
        dtype=int,
    )

    kernel_cierre = np.ones(
        (3, 1),
        dtype=np.uint8,
    )

    kernel_tolerancia = np.ones(
        (
            2 * TOLERANCIA_Y + 1,
            1,
        ),
        dtype=np.uint8,
    )

    scores_bloques = []

    coberturas_bloques = []

    mascaras_bloques = []

    umbrales_bloques = []

    centros_x = []

    # ========================================================
    # ANALIZAR CADA BLOQUE
    # ========================================================

    for numero in range(
        numero_bloques
    ):

        x0 = int(
            limites_x[numero]
        )

        x1 = int(
            limites_x[numero + 1]
        )

        if x1 <= x0:

            continue

        zona = respuesta[
            :,
            x0:x1,
        ]

        # ----------------------------------------------------
        # SCORE VERTICAL
        # ----------------------------------------------------

        score = np.percentile(
            zona,
            40,
            axis=1,
        ).astype(
            np.float32
        )

        score = cv2.GaussianBlur(
            score.reshape(
                -1,
                1,
            ),
            (1, 5),
            sigmaX=0,
            sigmaY=1.0,
        ).ravel()

        # ----------------------------------------------------
        # COBERTURA
        # ----------------------------------------------------

        cobertura = (
            zona >= CONTRASTE_MIN
        ).mean(
            axis=1
        )

        # ----------------------------------------------------
        # UMBRAL DEL BLOQUE
        # ----------------------------------------------------

        score_fuerte = float(
            np.percentile(
                score,
                99.8,
            )
        )

        umbral = max(
            CONTRASTE_MIN * 0.8,
            score_fuerte
            *
            FUERZA_RELATIVA_BLOQUE,
        )

        candidata = (
            (score >= umbral)
            &
            (
                cobertura
                >=
                COBERTURA_BLOQUE_MIN
            )
        )

        # ----------------------------------------------------
        # CERRAR HUECOS
        # ----------------------------------------------------

        mascara = (
            candidata.astype(
                np.uint8
            )
            *
            255
        ).reshape(
            -1,
            1,
        )

        mascara = cv2.morphologyEx(
            mascara,
            cv2.MORPH_CLOSE,
            kernel_cierre,
        )

        # ----------------------------------------------------
        # TOLERANCIA EN Y
        # ----------------------------------------------------

        mascara_tolerante = cv2.dilate(
            mascara,
            kernel_tolerancia,
        )

        mascara_tolerante = (
            mascara_tolerante.ravel()
            >
            0
        )

        scores_bloques.append(
            score
        )

        coberturas_bloques.append(
            cobertura
        )

        mascaras_bloques.append(
            mascara_tolerante
        )

        umbrales_bloques.append(
            umbral
        )

        centro_x = (
            x0
            +
            x1
            -
            1
        ) / 2.0

        centros_x.append(
            centro_x
        )

    if not scores_bloques:

        return (
            [],
            gray_original,
        )

    # ========================================================
    # CONVERTIR A ARRAYS
    # ========================================================

    scores_bloques = np.stack(
        scores_bloques,
        axis=1,
    )

    coberturas_bloques = np.stack(
        coberturas_bloques,
        axis=1,
    )

    mascaras_bloques = np.stack(
        mascaras_bloques,
        axis=1,
    )

    centros_x = np.asarray(
        centros_x,
        dtype=np.float32,
    )

    numero_bloques_real = (
        scores_bloques.shape[1]
    )

    bloques_minimos = max(
        2,
        math.ceil(
            numero_bloques_real
            *
            SOPORTE_BLOQUES_MIN
        ),
    )

    # ========================================================
    # ACUERDO ENTRE BLOQUES
    # ========================================================

    soporte = mascaras_bloques.mean(
        axis=1
    )

    candidata_global = (
        soporte
        >=
        SOPORTE_BLOQUES_MIN
    )

    candidata_global_u8 = (
        candidata_global.astype(
            np.uint8
        )
        *
        255
    ).reshape(
        -1,
        1,
    )

    candidata_global_u8 = (
        cv2.morphologyEx(
            candidata_global_u8,
            cv2.MORPH_CLOSE,
            kernel_cierre,
        )
    )

    candidata_global = (
        candidata_global_u8.ravel()
        >
        0
    )

    ys = np.flatnonzero(
        candidata_global
    )

    if len(ys) == 0:

        return (
            [],
            gray_original,
        )

    # ========================================================
    # AGRUPAR REGIONES
    # ========================================================

    cortes = np.where(
        np.diff(ys) > 1
    )[0]

    inicios = np.r_[
        ys[0],
        ys[cortes + 1],
    ]

    finales = np.r_[
        ys[cortes],
        ys[-1],
    ]

    lineas = []

    # ========================================================
    # ANALIZAR CADA POSIBLE LINEA
    # ========================================================

    for inicio_region, final_region in zip(
        inicios,
        finales,
    ):

        inicio_region = int(
            inicio_region
        )

        final_region = int(
            final_region
        )

        y_busqueda_0 = max(
            0,
            inicio_region
            -
            TOLERANCIA_Y
            -
            MARGEN_BUSQUEDA,
        )

        y_busqueda_1 = min(
            alto,
            final_region
            +
            TOLERANCIA_Y
            +
            MARGEN_BUSQUEDA
            +
            1,
        )

        xs_encontrados = []

        ys_encontrados = []

        scores_encontrados = []

        coberturas_encontradas = []

        # ====================================================
        # BUSCAR Y EXACTA EN CADA BLOQUE
        # ====================================================

        for bloque in range(
            numero_bloques_real
        ):

            score_bloque = (
                scores_bloques[
                    :,
                    bloque,
                ]
            )

            cobertura_bloque = (
                coberturas_bloques[
                    :,
                    bloque,
                ]
            )

            score_zona = (
                score_bloque[
                    y_busqueda_0:
                    y_busqueda_1
                ]
            )

            cobertura_zona = (
                cobertura_bloque[
                    y_busqueda_0:
                    y_busqueda_1
                ]
            )

            if len(score_zona) == 0:

                continue

            factor_cobertura = np.clip(
                cobertura_zona
                /
                max(
                    COBERTURA_BLOQUE_MIN,
                    1e-6,
                ),
                0,
                1.5,
            )

            metrica = (
                score_zona
                *
                factor_cobertura
            )

            indice_local = int(
                np.argmax(
                    metrica
                )
            )

            y_local = (
                y_busqueda_0
                +
                indice_local
            )

            umbral_bloque = (
                umbrales_bloques[
                    bloque
                ]
            )

            if (
                score_bloque[y_local]
                <
                umbral_bloque
            ):

                continue

            if (
                cobertura_bloque[y_local]
                <
                COBERTURA_BLOQUE_MIN
            ):

                continue

            xs_encontrados.append(
                centros_x[bloque]
            )

            ys_encontrados.append(
                y_local
            )

            scores_encontrados.append(
                float(
                    score_bloque[
                        y_local
                    ]
                )
            )

            coberturas_encontradas.append(
                float(
                    cobertura_bloque[
                        y_local
                    ]
                )
            )

        # ====================================================
        # EXIGIR SUFICIENTES BLOQUES
        # ====================================================

        if (
            len(xs_encontrados)
            <
            bloques_minimos
        ):

            continue

        # ====================================================
        # AJUSTAR RECTA
        # ====================================================

        ajuste = ajustar_recta_robusta(
            xs_encontrados,
            ys_encontrados,
            ERROR_RECTITUD_MAX,
        )

        if ajuste is None:

            continue

        a = ajuste["a"]

        b = ajuste["b"]

        buenos = ajuste["buenos"]

        error_rectitud = (
            ajuste["error"]
        )

        # ====================================================
        # ANGULO
        # ====================================================

        angulo = float(
            np.degrees(
                np.arctan(
                    a
                )
            )
        )

        if abs(angulo) > ANGULO_MAX:

            continue

        # ====================================================
        # RECTITUD
        # ====================================================

        if (
            error_rectitud
            >
            ERROR_RECTITUD_MAX
        ):

            continue

        xs_array = np.asarray(
            xs_encontrados,
            dtype=np.float64,
        )

        ys_array = np.asarray(
            ys_encontrados,
            dtype=np.float64,
        )

        scores_array = np.asarray(
            scores_encontrados,
            dtype=np.float64,
        )

        coberturas_array = np.asarray(
            coberturas_encontradas,
            dtype=np.float64,
        )

        ys_buenos = (
            ys_array[buenos]
        )

        # ====================================================
        # SOPORTE REAL
        # ====================================================

        soporte_real = float(
            buenos.sum()
            /
            numero_bloques_real
        )

        if (
            soporte_real
            <
            SOPORTE_BLOQUES_MIN
        ):

            continue

        # ====================================================
        # POSICION CENTRAL
        # ====================================================

        y_central = int(
            round(
                np.median(
                    ys_buenos
                )
            )
        )

        # ====================================================
        # EXTREMOS
        # ====================================================

        x1 = 0

        x2 = (
            ancho - 1
        )

        y1 = int(
            round(
                b
            )
        )

        y2 = int(
            round(
                a
                *
                (ancho - 1)
                +
                b
            )
        )

        y1 = int(
            np.clip(
                y1,
                0,
                alto - 1,
            )
        )

        y2 = int(
            np.clip(
                y2,
                0,
                alto - 1,
            )
        )

        # ====================================================
        # FUERZA Y CONFIANZA
        # ====================================================

        fuerza_media = float(
            np.mean(
                scores_array[
                    buenos
                ]
            )
        )

        cobertura_media = float(
            np.mean(
                coberturas_array[
                    buenos
                ]
            )
        )

        fuerza_normalizada = min(
            1.0,
            fuerza_media
            /
            max(
                CONTRASTE_MIN,
                1,
            ),
        )

        penalizacion_angulo = max(
            0.0,
            1.0
            -
            abs(angulo)
            /
            max(
                ANGULO_MAX * 2,
                1e-6,
            ),
        )

        penalizacion_rectitud = max(
            0.0,
            1.0
            -
            error_rectitud
            /
            max(
                ERROR_RECTITUD_MAX * 2,
                1e-6,
            ),
        )

        confianza = (
            soporte_real
            *
            cobertura_media
            *
            fuerza_normalizada
            *
            (
                0.7
                +
                0.3
                *
                penalizacion_angulo
            )
            *
            (
                0.7
                +
                0.3
                *
                penalizacion_rectitud
            )
        )

        # ====================================================
        # PUNTOS VALIDOS
        # ====================================================

        puntos_bloques = []

        for x_punto, y_punto, bueno in zip(
            xs_array,
            ys_array,
            buenos,
        ):

            if bueno:

                puntos_bloques.append(
                    (
                        int(
                            round(
                                x_punto
                            )
                        ),
                        int(
                            round(
                                y_punto
                            )
                        ),
                    )
                )

        # ====================================================
        # GUARDAR LINEA
        # ====================================================

        lineas.append(
            {
                "y": y_central,

                "x1": x1,
                "y1": y1,

                "x2": x2,
                "y2": y2,

                "angulo": angulo,

                "error_rectitud": (
                    error_rectitud
                ),

                "soporte": (
                    soporte_real
                ),

                "cobertura": (
                    cobertura_media
                ),

                # IMPORTANTE:
                # fuerza real de la linea
                "fuerza": (
                    fuerza_media
                ),

                "confianza": float(
                    confianza
                ),

                "num_bloques": (
                    numero_bloques_real
                ),

                "bloques_validos": (
                    len(
                        puntos_bloques
                    )
                ),

                "puntos_bloques": (
                    puntos_bloques
                ),

                "region_y0": (
                    inicio_region
                ),

                "region_y1": (
                    final_region
                ),

                # Se utilizan solamente para reconstruir
                # correctamente la recta en tamaño original.
                "_a_analisis": (
                    a
                ),

                "_b_analisis": (
                    b
                ),
            }
        )

    # ========================================================
    # FILTRAR LINEAS DEBILES
    # ========================================================

    if lineas:

        fuerza_maxima = max(
            linea["fuerza"]
            for linea in lineas
        )

        fuerza_minima = (
            fuerza_maxima
            *
            FUERZA_RELATIVA_LINEA_MIN
        )

        lineas = [
            linea
            for linea in lineas
            if (
                linea["fuerza"]
                >=
                fuerza_minima
            )
        ]

    # ========================================================
    # CONVERTIR COORDENADAS AL TAMAÑO ORIGINAL
    # ========================================================

    lineas = convertir_lineas_a_original(
        lineas,
        alto_original,
        ancho_original,
        alto,
        ancho,
    )

    # ========================================================
    # ORDENAR
    # ========================================================

    lineas.sort(
        key=lambda linea: linea["y"]
    )

    return (
        lineas,
        gray_original,
    )


# ============================================================
# DIBUJAR RESULTADOS
# ============================================================

def dibujar_resultado(
    gray,
    lineas,
):

    resultado = cv2.cvtColor(
        gray,
        cv2.COLOR_GRAY2BGR,
    )

    ancho = (
        gray.shape[1]
    )

    if (
        DIBUJAR_SOLO_PRINCIPAL
        and
        lineas
    ):

        lineas_dibujar = [
            obtener_linea_principal(
                lineas
            )
        ]

    else:

        lineas_dibujar = lineas

    for numero, linea in enumerate(
        lineas_dibujar,
        start=1,
    ):

        # ----------------------------------------------------
        # PUNTOS DE BLOQUES
        # ----------------------------------------------------

        if DIBUJAR_PUNTOS_BLOQUES:

            for x, y in linea[
                "puntos_bloques"
            ]:

                cv2.circle(
                    resultado,
                    (
                        x,
                        y,
                    ),
                    3,
                    (255, 0, 0),
                    -1,
                )

        # ----------------------------------------------------
        # LINEA
        # ----------------------------------------------------

        cv2.line(
            resultado,
            (
                linea["x1"],
                linea["y1"],
            ),
            (
                linea["x2"],
                linea["y2"],
            ),
            (0, 255, 0),
            2,
        )

        # ----------------------------------------------------
        # REGION
        # ----------------------------------------------------

        cv2.rectangle(
            resultado,
            (
                0,
                linea[
                    "region_y0"
                ],
            ),
            (
                ancho - 1,
                linea[
                    "region_y1"
                ],
            ),
            (0, 0, 255),
            1,
        )

        # ----------------------------------------------------
        # TEXTO
        # ----------------------------------------------------

        texto = (
            f"L{numero} "
            f"y={linea['y']} "
            f"ang={linea['angulo']:.2f} "
            f"sup={linea['soporte']:.2f} "
            f"fuerza={linea['fuerza']:.1f} "
            f"conf={linea['confianza']:.2f}"
        )

        posicion_y = max(
            20,
            linea[
                "region_y0"
            ]
            -
            8,
        )

        cv2.putText(
            resultado,
            texto,
            (
                5,
                posicion_y,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

    return resultado


# ============================================================
# BUSCAR IMAGENES
# ============================================================

def obtener_imagenes():

    archivos = []

    for ruta in CARPETA_RAIZ.rglob(
        "*"
    ):

        if not ruta.is_file():

            continue

        if (
            ruta.suffix.lower()
            not in
            EXTENSIONES
        ):

            continue

        # No volver a analizar resultados
        if ruta.stem.lower().endswith(
            "_detectado"
        ):

            continue

        # Evitar carpeta de salida
        if esta_dentro_de(
            ruta,
            CARPETA_SALIDA,
        ):

            continue

        archivos.append(
            ruta
        )

    archivos.sort()

    return archivos


# ============================================================
# PROCESAR CARPETA
# ============================================================

def procesar_carpeta():

    if not CARPETA_RAIZ.exists():

        print()

        print(
            "ERROR: La carpeta de entrada no existe:"
        )

        print(
            CARPETA_RAIZ
        )

        print()

        return

    CARPETA_SALIDA.mkdir(
        parents=True,
        exist_ok=True,
    )

    archivos = obtener_imagenes()

    total = len(
        archivos
    )

    print()

    print(
        "================================================"
    )

    print(
        " DETECTOR DE LINEAS NEGRAS HORIZONTALES"
    )

    print(
        " DETECCION POR BLOQUES + NORMALIZACION"
    )

    print(
        "================================================"
    )

    print()

    print(
        f"Entrada: {CARPETA_RAIZ}"
    )

    print(
        f"Salida : {CARPETA_SALIDA}"
    )

    print()

    print(
        f"Imagenes encontradas: {total}"
    )

    print()

    if total == 0:

        print(
            "No se encontraron imagenes."
        )

        return

    resumen_csv = []

    lineas_csv = []

    # ========================================================
    # PROCESAR IMAGENES
    # ========================================================

    for indice, ruta in enumerate(
        archivos,
        start=1,
    ):

        relativa = ruta.relative_to(
            CARPETA_RAIZ
        )

        print(
            f"[{indice}/{total}] "
            f"{relativa}"
        )

        try:

            # =================================================
            # DETECTAR
            # =================================================

            lineas, gray = (
                detectar_lineas_horizontales(
                    ruta
                )
            )

            alto, ancho = (
                gray.shape
            )

            # Añadir coordenadas y medidas relativas 0-100
            # sobre el tamaño ORIGINAL de la imagen.
            lineas = añadir_metricas_relativas(
                lineas,
                alto,
            )

            # Dimensiones que realmente ha utilizado
            # el detector internamente.
            (
                alto_analisis,
                ancho_analisis,
                escala_analisis,
            ) = calcular_dimensiones_analisis(
                alto,
                ancho,
            )

            num_bloques = (
                calcular_numero_bloques(
                    ancho_analisis
                )
            )

            # =================================================
            # INFORMACION DE DEPURACION
            # =================================================

            print(
                f"    Original: "
                f"{ancho} x {alto}"
            )

            print(
                f"    Analisis: "
                f"{ancho_analisis} x "
                f"{alto_analisis}"
            )

            print(
                f"    Escala: "
                f"{escala_analisis:.4f}"
            )

            print(
                "    LINEAS DEVUELTAS:"
            )

            for linea in lineas:

                print(
                    f"        "
                    f"y={linea['y']} "
                    f"fuerza={linea['fuerza']:.2f} "
                    f"soporte={linea['soporte']:.2f} "
                    f"angulo={linea['angulo']:.2f} "
                    f"error={linea['error_rectitud']:.2f}"
                )

            # =================================================
            # CARPETA DESTINO
            # =================================================

            carpeta_destino = (
                CARPETA_SALIDA
                /
                relativa.parent
            )

            carpeta_destino.mkdir(
                parents=True,
                exist_ok=True,
            )

            nombre_salida = (
                ruta.stem
                +
                "_detectado.png"
            )

            ruta_salida = (
                carpeta_destino
                /
                nombre_salida
            )

            # =================================================
            # DIBUJAR Y GUARDAR
            # =================================================

            resultado = dibujar_resultado(
                gray,
                lineas,
            )

            guardar_png(
                ruta_salida,
                resultado,
            )

            # =================================================
            # LINEA PRINCIPAL
            # =================================================

            principal = (
                obtener_linea_principal(
                    lineas
                )
            )

            if principal is not None:

                principal_y = (
                    principal["y"]
                )

                principal_y_relativa = (
                    principal[
                        "y_relativa"
                    ]
                )

                principal_grosor_relativo = (
                    principal[
                        "grosor_relativo"
                    ]
                )

                confianza_principal = (
                    principal[
                        "confianza"
                    ]
                )

                angulo_principal = (
                    principal[
                        "angulo"
                    ]
                )

                soporte_principal = (
                    principal[
                        "soporte"
                    ]
                )

                fuerza_principal = (
                    principal[
                        "fuerza"
                    ]
                )

            else:

                principal_y = ""

                principal_y_relativa = ""

                principal_grosor_relativo = ""

                confianza_principal = ""

                angulo_principal = ""

                soporte_principal = ""

                fuerza_principal = ""

            todas_y = ";".join(
                str(
                    linea["y"]
                )
                for linea in lineas
            )

            todas_y_relativas = ";".join(
                f"{linea['y_relativa']:.6f}"
                for linea in lineas
            )

            grosores_relativos = ";".join(
                f"{linea['grosor_relativo']:.6f}"
                for linea in lineas
            )

            distancias_relativas = ";".join(
                f"{linea['distancia_siguiente_relativa']:.6f}"
                for linea in lineas
                if (
                    linea[
                        "distancia_siguiente_relativa"
                    ]
                    !=
                    ""
                )
            )

            # =================================================
            # CSV RESUMEN
            # =================================================

            resumen_csv.append(
                {
                    "imagen": (
                        relativa.as_posix()
                    ),

                    "ancho": (
                        ancho
                    ),

                    "alto": (
                        alto
                    ),

                    "ratio_ancho_alto": (
                        ancho / alto
                    ),

                    "ancho_analisis": (
                        ancho_analisis
                    ),

                    "alto_analisis": (
                        alto_analisis
                    ),

                    "escala_analisis": (
                        escala_analisis
                    ),

                    "num_bloques": (
                        num_bloques
                    ),

                    "num_lineas": (
                        len(lineas)
                    ),

                    "linea_principal_y": (
                        principal_y
                    ),

                    "linea_principal_y_relativa": (
                        principal_y_relativa
                    ),

                    "grosor_principal_relativo": (
                        principal_grosor_relativo
                    ),

                    "fuerza_principal": (
                        fuerza_principal
                    ),

                    "confianza_principal": (
                        confianza_principal
                    ),

                    "soporte_principal": (
                        soporte_principal
                    ),

                    "angulo_principal": (
                        angulo_principal
                    ),

                    "todas_las_y": (
                        todas_y
                    ),

                    "todas_las_y_relativas": (
                        todas_y_relativas
                    ),

                    "grosores_relativos": (
                        grosores_relativos
                    ),

                    "distancias_entre_lineas_relativas": (
                        distancias_relativas
                    ),

                    "estado": (
                        "DETECTADA"
                        if lineas
                        else
                        "NO_DETECTADA"
                    ),
                }
            )

            # =================================================
            # CSV DETALLADO
            # =================================================

            for numero_linea, linea in enumerate(
                lineas,
                start=1,
            ):

                lineas_csv.append(
                    {
                        "imagen": (
                            relativa.as_posix()
                        ),

                        "numero_linea": (
                            numero_linea
                        ),

                        "y": (
                            linea["y"]
                        ),

                        "y_relativa": (
                            linea[
                                "y_relativa"
                            ]
                        ),

                        "limite_superior_y_relativa": (
                            linea[
                                "limite_superior_y_relativa"
                            ]
                        ),

                        "limite_inferior_y_relativa": (
                            linea[
                                "limite_inferior_y_relativa"
                            ]
                        ),

                        "grosor_px": (
                            linea[
                                "grosor_px"
                            ]
                        ),

                        "grosor_relativo": (
                            linea[
                                "grosor_relativo"
                            ]
                        ),

                        "distancia_siguiente_px": (
                            linea[
                                "distancia_siguiente_px"
                            ]
                        ),

                        "distancia_siguiente_relativa": (
                            linea[
                                "distancia_siguiente_relativa"
                            ]
                        ),

                        "x1": (
                            linea["x1"]
                        ),

                        "y1": (
                            linea["y1"]
                        ),

                        "x2": (
                            linea["x2"]
                        ),

                        "y2": (
                            linea["y2"]
                        ),

                        "angulo": (
                            linea["angulo"]
                        ),

                        "error_rectitud": (
                            linea[
                                "error_rectitud"
                            ]
                        ),

                        "soporte": (
                            linea[
                                "soporte"
                            ]
                        ),

                        "cobertura": (
                            linea[
                                "cobertura"
                            ]
                        ),

                        "fuerza": (
                            linea[
                                "fuerza"
                            ]
                        ),

                        "confianza": (
                            linea[
                                "confianza"
                            ]
                        ),

                        "num_bloques": (
                            linea[
                                "num_bloques"
                            ]
                        ),

                        "bloques_validos": (
                            linea[
                                "bloques_validos"
                            ]
                        ),
                    }
                )

            # =================================================
            # TERMINAL
            # =================================================

            print(
                f"    Bloques: "
                f"{num_bloques}"
            )

            print(
                f"    Lineas: "
                f"{len(lineas)}"
            )

            if principal is not None:

                print(
                    f"    Principal: "
                    f"y={principal_y}, "
                    f"fuerza={fuerza_principal:.2f}, "
                    f"conf={confianza_principal:.3f}, "
                    f"soporte={soporte_principal:.2f}"
                )

        except Exception as e:

            print(
                f"    ERROR: {e}"
            )

            resumen_csv.append(
                {
                    "imagen": (
                        relativa.as_posix()
                    ),

                    "ancho": "",

                    "alto": "",

                    "ratio_ancho_alto": "",

                    "ancho_analisis": "",

                    "alto_analisis": "",

                    "escala_analisis": "",

                    "num_bloques": "",

                    "num_lineas": 0,

                    "linea_principal_y": "",

                    "linea_principal_y_relativa": "",

                    "grosor_principal_relativo": "",

                    "fuerza_principal": "",

                    "confianza_principal": "",

                    "soporte_principal": "",

                    "angulo_principal": "",

                    "todas_las_y": "",

                    "todas_las_y_relativas": "",

                    "grosores_relativos": "",

                    "distancias_entre_lineas_relativas": "",

                    "estado": (
                        f"ERROR: {e}"
                    ),
                }
            )

    # ========================================================
    # CSV RESUMEN
    # ========================================================

    ruta_resumen = (
        CARPETA_SALIDA
        /
        "resultados_resumen.csv"
    )

    columnas_resumen = [
        "imagen",
        "ancho",
        "alto",
        "ratio_ancho_alto",
        "ancho_analisis",
        "alto_analisis",
        "escala_analisis",
        "num_bloques",
        "num_lineas",
        "linea_principal_y",
        "linea_principal_y_relativa",
        "grosor_principal_relativo",
        "fuerza_principal",
        "confianza_principal",
        "soporte_principal",
        "angulo_principal",
        "todas_las_y",
        "todas_las_y_relativas",
        "grosores_relativos",
        "distancias_entre_lineas_relativas",
        "estado",
    ]

    # ========================================================
    # CSV DETALLADO
    # ========================================================

    ruta_lineas = (
        CARPETA_SALIDA
        /
        "resultados_lineas.csv"
    )

    columnas_lineas = [
        "imagen",
        "numero_linea",
        "y",
        "y_relativa",
        "limite_superior_y_relativa",
        "limite_inferior_y_relativa",
        "grosor_px",
        "grosor_relativo",
        "distancia_siguiente_px",
        "distancia_siguiente_relativa",
        "x1",
        "y1",
        "x2",
        "y2",
        "angulo",
        "error_rectitud",
        "soporte",
        "cobertura",
        "fuerza",
        "confianza",
        "num_bloques",
        "bloques_validos",
    ]

    guardar_csv(
        ruta_resumen,
        columnas_resumen,
        resumen_csv,
    )

    guardar_csv(
        ruta_lineas,
        columnas_lineas,
        lineas_csv,
    )

    # ========================================================
    # CSV FINAL SIMPLE
    # ========================================================

    ruta_resultados_finales = (
        CARPETA_SALIDA
        /
        "results_finales.csv"
    )

    columnas_resultados_finales = [
        "imagen",
        "posicion_lineas_relativa",
        "grosor_lineas_relativo",
        "distancia_entre_lineas_relativa",
    ]

    filas_resultados_finales = [
        {
            "imagen": fila.get("imagen", ""),
            "posicion_lineas_relativa": fila.get(
                "todas_las_y_relativas",
                "",
            ),
            "grosor_lineas_relativo": fila.get(
                "grosores_relativos",
                "",
            ),
            "distancia_entre_lineas_relativa": fila.get(
                "distancias_entre_lineas_relativas",
                "",
            ),
        }
        for fila in resumen_csv
    ]

    guardar_csv(
        ruta_resultados_finales,
        columnas_resultados_finales,
        filas_resultados_finales,
    )

    # ========================================================
    # FINAL
    # ========================================================

    print()

    print(
        "================================================"
    )

    print(
        " PROCESO TERMINADO"
    )

    print(
        "================================================"
    )

    print()

    print(
        f"Imagenes procesadas: {total}"
    )

    print()

    print(
        f"Resultados: {CARPETA_SALIDA}"
    )

    print()

    print(
        f"CSV resumen: {ruta_resumen}"
    )

    print(
        f"CSV lineas: {ruta_lineas}"
    )

    print(
        f"CSV final: {ruta_resultados_finales}"
    )

    print()


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    procesar_carpeta()