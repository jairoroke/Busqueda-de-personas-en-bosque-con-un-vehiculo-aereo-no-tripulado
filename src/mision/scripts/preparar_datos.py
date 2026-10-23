#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
preparar_datos.py -- Prepara los conjuntos de datos de personas en imágenes aéreas
para el detector de una etapa (Sección 4.9): convierte las anotaciones (XML de
Pascal VOC o CSV) al formato de YOLO, divide las imágenes grandes en mosaicos
con traslape, reparte las particiones por imagen original y escribe el archivo
YAML del conjunto.

Uso:
  python3 preparar_datos.py --imagenes HERIDAL/imagenes --anotaciones HERIDAL/labels \\
      --formato voc --salida datos/heridal --mosaico 1280 --traslape 0.2
Formato CSV esperado (una caja por fila): imagen,xmin,ymin,xmax,ymax,clase
"""
import argparse, csv, glob, os, random, xml.etree.ElementTree as ET
from PIL import Image


def leer_voc(ruta):
    cajas = []
    for obj in ET.parse(ruta).getroot().findall("object"):
        b = obj.find("bndbox")
        cajas.append(tuple(float(b.find(k).text) for k in ("xmin", "ymin", "xmax", "ymax")))
    return cajas


def leer_csv(ruta):
    cajas = {}
    with open(ruta) as fh:
        for fila in csv.DictReader(fh):
            cajas.setdefault(fila["imagen"], []).append(tuple(float(fila[k]) for k in ("xmin", "ymin", "xmax", "ymax")))
    return cajas


def mosaicos(ancho, alto, tam, traslape):
    """Esquinas (x0, y0) de los mosaicos que cubren la imagen con el traslape pedido."""
    paso = int(tam * (1 - traslape))
    xs = list(range(0, max(ancho - tam, 0) + 1, paso)); ys = list(range(0, max(alto - tam, 0) + 1, paso))
    if xs[-1] + tam < ancho: xs.append(ancho - tam)
    if ys[-1] + tam < alto: ys.append(alto - tam)
    return [(x, y) for y in ys for x in xs]


def recortar(img, cajas, tam, traslape, base, carpeta_img, carpeta_lab, min_frac=0.5):
    """Divide la imagen en mosaicos y escribe cada uno con sus etiquetas YOLO (clase cx cy w h normalizados)."""
    W, H = img.size; n = 0
    for (x0, y0) in mosaicos(W, H, tam, traslape):
        etiquetas = []
        for (xmin, ymin, xmax, ymax) in cajas:
            ix0, iy0 = max(xmin, x0), max(ymin, y0); ix1, iy1 = min(xmax, x0+tam), min(ymax, y0+tam)
            if ix1 <= ix0 or iy1 <= iy0:
                continue
            frac = (ix1-ix0)*(iy1-iy0) / max((xmax-xmin)*(ymax-ymin), 1e-6)
            if frac < min_frac:
                continue                                      # caja cortada: se conserva si queda al menos la mitad
            cx, cy = ((ix0+ix1)/2 - x0)/tam, ((iy0+iy1)/2 - y0)/tam
            etiquetas.append(f"0 {cx:.6f} {cy:.6f} {(ix1-ix0)/tam:.6f} {(iy1-iy0)/tam:.6f}")
        nombre = f"{base}_{x0}_{y0}"
        img.crop((x0, y0, x0+tam, y0+tam)).save(os.path.join(carpeta_img, nombre + ".jpg"), quality=95)
        with open(os.path.join(carpeta_lab, nombre + ".txt"), "w") as fh:
            fh.write("\n".join(etiquetas))
        n += 1
    return n


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--imagenes", required=True); ap.add_argument("--anotaciones", required=True)
    ap.add_argument("--formato", choices=["voc", "csv"], default="voc")
    ap.add_argument("--salida", required=True); ap.add_argument("--mosaico", type=int, default=1280)
    ap.add_argument("--traslape", type=float, default=0.2); ap.add_argument("--semilla", type=int, default=0)
    ap.add_argument("--fracciones", default="0.7,0.15,0.15")
    a = ap.parse_args()
    imgs = sorted(glob.glob(os.path.join(a.imagenes, "*.[jJpP][pPnN]*")))
    random.Random(a.semilla).shuffle(imgs)                    # partición por imagen original, no por mosaico
    f_tr, f_va, _ = map(float, a.fracciones.split(","))
    n = len(imgs); cortes = {"train": imgs[:int(f_tr*n)], "val": imgs[int(f_tr*n):int((f_tr+f_va)*n)],
                             "test": imgs[int((f_tr+f_va)*n):]}
    csv_cajas = leer_csv(a.anotaciones) if a.formato == "csv" else None
    total = 0
    for parte, lista in cortes.items():
        ci, cl = os.path.join(a.salida, "images", parte), os.path.join(a.salida, "labels", parte)
        os.makedirs(ci, exist_ok=True); os.makedirs(cl, exist_ok=True)
        for ruta in lista:
            base = os.path.splitext(os.path.basename(ruta))[0]
            if a.formato == "voc":
                xml = os.path.join(a.anotaciones, base + ".xml")
                cajas = leer_voc(xml) if os.path.exists(xml) else []
            else:
                cajas = csv_cajas.get(os.path.basename(ruta), [])
            with Image.open(ruta) as img:
                total += recortar(img.convert("RGB"), cajas, a.mosaico, a.traslape, base, ci, cl)
    with open(os.path.join(a.salida, "datos.yaml"), "w") as fh:
        fh.write(f"path: {os.path.abspath(a.salida)}\ntrain: images/train\nval: images/val\ntest: images/test\n"
                 f"names:\n  0: persona\n")
    print(f"{n} imágenes originales -> {total} mosaicos; YAML en {os.path.join(a.salida, 'datos.yaml')}")
