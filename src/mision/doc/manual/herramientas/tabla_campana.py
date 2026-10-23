#!/usr/bin/env python3
"""Escribe capitulos/resultados_campana.tex con las tablas de la campaña (~/resultados)."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.expanduser("~/ros2_ws/src/mision/scripts"))
from analisis_wilcoxon import cargar, aparear, prueba, holm

RES = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/resultados")
SAL = os.path.join(os.path.dirname(__file__), "..", "capitulos", "resultados_campana.tex")
filas = cargar(RES)
for r in filas:
    r["personas_por_min"] = 60.0 * r["personas_localizadas"] / r["duracion_s"]
VARS = [("personas_localizadas", "greater", "personas"), ("personas_por_min", "greater", "personas/min"),
        ("rmse_seguimiento", "two-sided", "RMSE (m)"), ("falsas_alarmas", "two-sided", "falsas alarmas")]
L = []
semillas = sorted({r["semilla"] for r in filas})
L.append(f"% generado por herramientas/tabla_campana.py con {len(filas)} corridas\n")
L.append(f"Corridas analizadas: {len(filas)} (semillas {semillas[0]}--{semillas[-1]}).\n" if filas else "")

# medias por oclusión, estrategia y controlador
L.append("\\begin{table}[H]\\centering\\small\n\\begin{tabular}{lllrrrrrrr}\n\\toprule\n"
         "\\textbf{Control} & \\textbf{Oclusión} & \\textbf{Estrategia} & $n$ & \\textbf{Encontradas} & "
         "\\textbf{FA} & \\textbf{FA$_{10}$} & \\textbf{Avisos} & \\textbf{Duración (s)} & \\textbf{RMSE (m)} \\\\ \\midrule\n")
for ctl in ["cascada", "red"]:
    for oc in ["baja", "media", "alta"]:
        for est in ["fija", "reobs"]:
            g = [r for r in filas if r["controlador"] == ctl and r["oclusion"] == oc and r["estrategia"] == est]
            if not g:
                continue
            tot = sum(r["personas_localizadas"] for r in g); pers = sum(int(r["personas"]) for r in g)
            enc = f"{tot}/{pers}" if pers else f"{tot}"
            L.append(f"{ctl} & {oc} & {est} & {len(g)} & {enc} & {np.mean([r['falsas_alarmas'] for r in g]):.1f} & {np.mean([r['por_min_detecciones']['10']['falsas_alarmas'] for r in g]):.1f} & {np.mean([len(r.get('avisos', [])) for r in g]):.1f} & "
                     f"{np.mean([r['duracion_s'] for r in g]):.0f} & {np.mean([r['rmse_seguimiento'] for r in g]):.3f} \\\\\n")
L.append("\\bottomrule\n\\end{tabular}\n\\caption{Promedios por condición. FA: falsas alarmas con una detección; FA$_{10}$: con al menos 10; avisos GPS por vuelo.}"
         "\n\\label{tab:campana-medias}\n\\end{table}\n")

# Wilcoxon + Holm
filas_w, ps = [], []
for oc in ["baja", "media", "alta"]:
    pares = aparear(filas, oc, "cascada")
    for var, alt, nom in VARS:
        r = prueba(pares, var, alt); filas_w.append((oc, nom, r)); ps.append(r["p"])
aj = holm(ps)
L.append("\\begin{table}[H]\\centering\\small\n\\begin{tabular}{llrrcrr}\n\\toprule\n"
         "\\textbf{Oclusión} & \\textbf{Variable} & $n$ & \\textbf{Mediana $d$} & \\textbf{IC 95~\\%} & $p$ & "
         "$p_{Holm}$ \\\\ \\midrule\n")
f = lambda x, k=3: "--" if not np.isfinite(x) else f"{x:.{k}f}"
for (oc, nom, r), pa in zip(filas_w, aj):
    L.append(f"{oc} & {nom} & {r['n']} & {f(r['mediana'])} & [{f(r['ic'][0])}, {f(r['ic'][1])}] & "
             f"{f(r['p'])} & {f(pa)} \\\\\n")
L.append("\\bottomrule\n\\end{tabular}\n\\caption{Re-observación menos barrido fijo, apareado por semilla, "
         "iluminación y viento (controlador en cascada).}\n\\label{tab:campana-wilcoxon}\n\\end{table}\n")
open(SAL, "w").write("".join(L))
print(f"{len(filas)} corridas -> {os.path.abspath(SAL)}")
