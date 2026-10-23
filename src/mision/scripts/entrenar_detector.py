#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
entrenar_detector.py -- Entrena el detector de personas de una etapa (familia
YOLO, biblioteca ultralytics) a partir de pesos preentrenados en COCO y lo
evalúa globalmente y por estrato (nivel de oclusión, iluminación y altura), Sección 4.10.

Uso (con el Python de ~/.venvs/yolo):
  python3 entrenar_detector.py entrenar --datos ~/datasets/personas_bosque/datos.yaml --epocas 80
  python3 entrenar_detector.py evaluar --pesos ../yolo/personas_yolo26s.pt --datos ~/datasets/personas_bosque/datos.yaml \\
          --estratos ~/datasets/personas_bosque/estratos.csv
El archivo de estratos puede ser el JSON del diseño original ({"img_0001.jpg": "oclusion_media|30m", ...}) o el
estratos.csv que escribe capturar_dataset.py (imagen,partición,oclusión,iluminación,altura,personas).

Diferencias con el Código 8.9 del diseño original: el modelo base por omisión es YOLO26s (la versión
vigente de Ultralytics; la n es más rápida pero pierde personas pequeñas), el entrenamiento usa la
GPU, evalúa sobre la partición val (el dataset del simulador no tiene test aparte) y copia el mejor
modelo a yolo/ del paquete, que es el que usa el launch.
Requiere: ultralytics, torch
"""
import argparse, csv, json, os, shutil, tempfile, time
from ultralytics import YOLO

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def entrenar(datos, epocas, modelo_base="yolo26s.pt", tam=1280, lote=8, nombre=None):
    datos = os.path.abspath(os.path.expanduser(datos))
    proyecto = os.path.join(os.path.dirname(datos), "entrenamientos")
    nombre = nombre or "personas_" + os.path.splitext(os.path.basename(modelo_base))[0]
    modelo = YOLO(modelo_base)                                   # pesos preentrenados en COCO
    modelo.train(data=datos, epochs=epocas, imgsz=tam, batch=lote, name=nombre, project=proyecto, exist_ok=True,
                 patience=20, optimizer="AdamW", lr0=1e-3, cos_lr=True, close_mosaic=10, device=0, workers=8,
                 hsv_h=0.015, hsv_s=0.6, hsv_v=0.4, degrees=10.0, scale=0.5, fliplr=0.5, flipud=0.5, mosaic=1.0,
                 seed=0, deterministic=True)
    mejor = os.path.join(proyecto, nombre, "weights", "best.pt")
    destino = os.path.join(RAIZ, "yolo", nombre + ".pt")
    os.makedirs(os.path.dirname(destino), exist_ok=True)
    shutil.copy(mejor, destino)
    print(f"Modelo copiado a {destino}")
    return destino


def evaluar_global(pesos, datos, tam=1280, particion="val"):
    modelo = YOLO(pesos)
    r = modelo.val(data=datos, imgsz=tam, split=particion, conf=0.001, iou=0.6, plots=False, verbose=False)
    carpeta = os.path.join(os.path.dirname(os.path.abspath(datos)), "images", particion)
    imgs = sorted(os.listdir(carpeta))[:200]
    t0 = time.perf_counter()
    for im in imgs:
        modelo.predict(os.path.join(carpeta, im), imgsz=tam, verbose=False)
    return {"AP50": float(r.box.map50), "AP50_95": float(r.box.map), "P": float(r.box.mp), "R": float(r.box.mr),
            "ms_por_imagen": 1000 * (time.perf_counter() - t0) / max(len(imgs), 1)}


def leer_estratos(ruta, particion="val"):
    if ruta.endswith(".json"):
        with open(ruta) as fh:
            return json.load(fh)
    estratos = {}
    with open(ruta) as fh:
        for fila in csv.reader(fh):
            img, parte, oclusion, iluminacion, h = fila[:5]
            if parte != particion:
                continue
            banda = "<20m" if float(h) < 20 else ("20-32m" if float(h) < 32 else ">32m")
            estratos[img] = f"oclusion_{oclusion}|sol_{iluminacion}|{banda}"
    return estratos


def evaluar_por_estrato(pesos, datos, estratos_ruta, tam=1280, particion="val", separar=True):
    """Crea un conjunto temporal por estrato (enlaces simbólicos) y evalúa cada uno."""
    estratos = leer_estratos(estratos_ruta, particion)
    raiz = os.path.dirname(os.path.abspath(datos)); resultados = {}
    # además de las combinaciones, cada factor por separado
    grupos = {}
    for img, e in estratos.items():
        grupos.setdefault(e, []).append(img)
        if separar:
            for parte in e.split("|"):
                grupos.setdefault(parte, []).append(img)
    for estrato in sorted(grupos):
        tmp = tempfile.mkdtemp(prefix="estrato_")
        for sub in ("images", "labels"):
            os.makedirs(os.path.join(tmp, sub, "test"))
        for img in grupos[estrato]:
            base = os.path.splitext(img)[0]
            os.symlink(os.path.join(raiz, "images", particion, img), os.path.join(tmp, "images/test", img))
            lab = os.path.join(raiz, "labels", particion, base + ".txt")
            if os.path.exists(lab):
                os.symlink(lab, os.path.join(tmp, "labels/test", base + ".txt"))
        yaml = os.path.join(tmp, "datos.yaml")
        with open(yaml, "w") as fh:
            fh.write(f"path: {tmp}\ntrain: images/test\nval: images/test\ntest: images/test\nnames:\n  0: persona\n")
        r = YOLO(pesos).val(data=yaml, imgsz=tam, split="test", conf=0.001, iou=0.6, plots=False, verbose=False)
        resultados[estrato] = {"n": len(grupos[estrato]), "AP50": float(r.box.map50), "AP50_95": float(r.box.map),
                               "P": float(r.box.mp), "R": float(r.box.mr)}
        shutil.rmtree(tmp)
    return resultados


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="modo", required=True)
    e = sub.add_parser("entrenar"); e.add_argument("--datos", required=True); e.add_argument("--epocas", type=int, default=80)
    e.add_argument("--base", default="yolo26s.pt"); e.add_argument("--tam", type=int, default=1280)
    e.add_argument("--lote", type=int, default=8)
    v = sub.add_parser("evaluar"); v.add_argument("--pesos", required=True); v.add_argument("--datos", required=True)
    v.add_argument("--estratos", default=None); v.add_argument("--tam", type=int, default=1280)
    v.add_argument("--particion", default="val")
    a = ap.parse_args()
    if a.modo == "entrenar":
        destino = entrenar(a.datos, a.epocas, a.base, a.tam, a.lote)
        print(json.dumps(evaluar_global(destino, os.path.expanduser(a.datos), a.tam), indent=2))
    else:
        print(json.dumps(evaluar_global(a.pesos, a.datos, a.tam, a.particion), indent=2))
        if a.estratos:
            print(json.dumps(evaluar_por_estrato(a.pesos, a.datos, a.estratos, a.tam, a.particion), indent=2))
