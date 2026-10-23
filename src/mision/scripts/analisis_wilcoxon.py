#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analisis_wilcoxon.py -- Análisis estadístico de la hipótesis:
diferencias apareadas por semilla entre la estrategia de re-observación y la
fija, prueba de rangos con signo de Wilcoxon, intervalo de confianza de la
mediana por remuestreo y corrección de Holm por nivel de oclusión.

Uso:
  python3 analisis_wilcoxon.py --resultados ~/resultados    (archivos JSON de las corridas)
  python3 analisis_wilcoxon.py --demo                       (datos sintéticos para probar el script)
  python3 analisis_wilcoxon.py --resultados ~/resultados --controlador red
"""
import argparse, glob, json, os
import numpy as np
from scipy import stats


def cargar(carpeta):
    filas = []
    for f in glob.glob(os.path.join(carpeta, "*.json")):
        with open(f) as fh:
            r = json.load(fh)
        if "personas_localizadas" in r:
            filas.append(r)
    return filas


def aparear(filas, oclusion, controlador="cascada", piloto=None):
    """Devuelve pares (reobs, fija) por semilla para una oclusión dada."""
    por_clave = {}
    for r in filas:
        if r["oclusion"] != oclusion or r["controlador"] != controlador or (piloto and r.get("piloto", "px4") != piloto):
            continue
        clave = (r["semilla"], r["iluminacion"], r["viento"])
        por_clave.setdefault(clave, {})[r["estrategia"]] = r
    pares = [(v["reobs"], v["fija"]) for v in por_clave.values() if "reobs" in v and "fija" in v]
    return pares


def ic_mediana(d, n_boot=10000, alfa=0.05, rng=np.random.default_rng(0)):
    med = np.array([np.median(rng.choice(d, len(d), replace=True)) for _ in range(n_boot)])
    return np.percentile(med, 100*alfa/2), np.percentile(med, 100*(1-alfa/2))


def prueba(pares, variable, alternativa):
    a = np.array([p[0][variable] for p in pares], float)
    b = np.array([p[1][variable] for p in pares], float)
    d = a - b
    if len(d) == 0:
        return dict(n=0, mediana=float("nan"), ic=(float("nan"), float("nan")), W=float("nan"), p=float("nan"))
    if np.all(d == 0):
        return dict(n=len(d), mediana=0.0, ic=(0.0, 0.0), W=0.0, p=1.0)
    W, p = stats.wilcoxon(a, b, alternative=alternativa, zero_method="wilcox")
    return dict(n=len(d), mediana=float(np.median(d)), ic=ic_mediana(d), W=float(W), p=float(p))


def holm(ps):
    """Valores p ajustados por el método de Holm (los NaN se dejan fuera)."""
    ps = np.asarray(ps, float); ajust = np.full(len(ps), np.nan)
    validos = np.where(np.isfinite(ps))[0]
    orden = validos[np.argsort(ps[validos])]; m = len(validos)
    prev = 0.0
    for k, i in enumerate(orden):
        prev = max(prev, (m - k) * ps[i]); ajust[i] = min(prev, 1.0)
    return ajust


def demo():
    """Datos sintéticos con el efecto que postula la hipótesis en oclusión media."""
    rng = np.random.default_rng(1); filas = []
    for s in range(1, 31):
        for il in ["alta", "baja"]:
            for vi in ["no", "si"]:
                for oc, efecto in [("baja", 0.05), ("media", 0.5), ("alta", 0.15)]:
                    base = rng.poisson(1.2)
                    for est in ["fija", "reobs"]:
                        pl = base + (rng.binomial(1, efecto) if est == "reobs" else 0)
                        filas.append(dict(semilla=s, iluminacion=il, viento=vi, oclusion=oc, estrategia=est,
                                          controlador="cascada", personas_localizadas=int(pl),
                                          rmse_seguimiento=float(rng.normal(0.35, 0.05) + (0.01 if est == "reobs" else 0)),
                                          falsas_alarmas=int(rng.poisson(0.4 if est == "fija" else 0.25)),
                                          duracion_s=float(240 + (30 if est == "reobs" else 0) + rng.normal(0, 5))))
    return filas


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--resultados", default=os.path.expanduser("~/resultados"))
    ap.add_argument("--controlador", default="cascada")
    ap.add_argument("--piloto", default=None, help="px4 o ardupilot (por omisión, todas las corridas)")
    ap.add_argument("--demo", action="store_true")
    a = ap.parse_args()
    filas = demo() if a.demo else cargar(a.resultados)
    for r in filas:                                           # personas por minuto
        r["personas_por_min"] = 60.0 * r["personas_localizadas"] / r["duracion_s"]
    print(f"{len(filas)} corridas; controlador {a.controlador}")
    print(f"{'oclusión':9s} {'variable':22s} {'n':>4s} {'mediana':>8s} {'IC 95 %':>16s} {'W':>8s} {'p':>8s}")
    ps, etiquetas = [], []
    for oc in ["baja", "media", "alta"]:
        pares = aparear(filas, oc, a.controlador, a.piloto)
        for var, alt in [("personas_localizadas", "greater"), ("personas_por_min", "greater"),
                         ("rmse_seguimiento", "two-sided"), ("falsas_alarmas", "two-sided")]:
            r = prueba(pares, var, alt)
            ps.append(r["p"]); etiquetas.append((oc, var))
            print(f"{oc:9s} {var:22s} {r['n']:4d} {r['mediana']:8.3f} "
                  f"[{r['ic'][0]:6.3f}, {r['ic'][1]:6.3f}] {r['W']:8.1f} {r['p']:8.4f}")
    print("\nValores p ajustados por Holm (todas las pruebas):")
    for (oc, var), pa in zip(etiquetas, holm(np.array(ps))):
        print(f"  {oc:6s} {var:22s} p_ajustado = {pa:.4f}")
