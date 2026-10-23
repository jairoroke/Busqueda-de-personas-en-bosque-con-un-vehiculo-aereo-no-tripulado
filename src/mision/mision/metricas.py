# -*- coding: utf-8 -*-
"""Métricas de una corrida (Sección 4.12.2): personas localizadas, falsas alarmas,
error de localización y error de seguimiento de la ruta.

Una candidata cuenta como detección si su confianza acumulada es al menos
`umbral_conf`; una persona está localizada si alguna detección queda a menos de
`radio` metros (5 m por omisión), con asignación uno a uno de la más cercana (y, si se pide,
que haya fusionado al menos `min_detecciones` detecciones). Las
detecciones que no se asignan a ninguna persona son falsas alarmas.
"""
import numpy as np


def evaluar_candidatas(candidatas, personas, umbral_conf=0.6, radio=5.0, min_detecciones=1):
    """candidatas: lista de (x, y, conf, n); personas: lista de dict con x, y (del JSON del mundo).
    min_detecciones: además de la confianza, exige que la candidata fusione al menos n detecciones
    (1 = el criterio original, sólo la confianza acumulada)."""
    dets = [c for c in candidatas if c[2] >= umbral_conf and c[3] >= min_detecciones]
    pares = sorted(((np.hypot(c[0] - p["x"], c[1] - p["y"]), i, j)
                    for i, c in enumerate(dets) for j, p in enumerate(personas)))
    usadas_c, usadas_p, errores, asignacion = set(), set(), [], {}
    for d, i, j in pares:
        if d > radio:
            break
        if i in usadas_c or j in usadas_p:
            continue
        usadas_c.add(i); usadas_p.add(j); errores.append(d); asignacion[j] = i
    return {
        "personas": len(personas),
        "personas_localizadas": len(usadas_p),
        "falsas_alarmas": len(dets) - len(usadas_c),
        "candidatas_totales": len(candidatas),
        "error_localizacion_m": float(np.mean(errores)) if errores else None,
        "errores_localizacion_m": [float(e) for e in errores],
        "localizadas": sorted(int(j) for j in usadas_p),
    }


def distancia_a_ruta(p, ruta):
    """Distancia de un punto 3D a la poligonal `ruta` (lista de puntos 3D)."""
    p = np.asarray(p, float); dmin = np.inf
    for a, b in zip(ruta[:-1], ruta[1:]):
        a = np.asarray(a, float); b = np.asarray(b, float); ab = b - a
        t = 0.0 if not ab.any() else float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
        dmin = min(dmin, float(np.linalg.norm(p - (a + t * ab))))
    return dmin
