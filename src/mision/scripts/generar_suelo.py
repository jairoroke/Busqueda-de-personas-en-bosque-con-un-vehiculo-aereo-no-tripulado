#!/usr/bin/env python3
"""Genera el modelo 'suelo': un plano de 400 x 400 m con textura de pasto y tierra.

Sobre un verde liso una persona se detecta trivialmente; el pasto con manchas de
tierra, hojarasca y variaciones de tono se parece más a las imágenes aéreas de
bosque. La textura (1024x1024, ruido de varias escalas con semilla fija) se
repite cada 8 m sobre una malla OBJ con coordenadas UV mayores que 1, porque
Gazebo estira la textura de un <plane> sobre todo el plano.

Uso: python3 scripts/generar_suelo.py
"""
import os

import numpy as np
from PIL import Image

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LADO, REPETICION, N = 400.0, 8.0, 1024


def ruido(rng, n, escala):
    """Ruido suave periódico: rejilla aleatoria interpolada (para que el mosaico no tenga costuras)."""
    k = max(2, n // escala)
    g = rng.random((k, k))
    x = np.arange(n) * k / n
    i0 = np.floor(x).astype(int); t = x - i0; i1 = (i0 + 1) % k
    t = t * t * (3 - 2 * t)
    a = g[i0][:, i0] * (1 - t)[None, :] + g[i0][:, i1] * t[None, :]
    b = g[i1][:, i0] * (1 - t)[None, :] + g[i1][:, i1] * t[None, :]
    return a * (1 - t)[:, None] + b * t[:, None]


def textura(semilla=3):
    rng = np.random.default_rng(semilla)
    n = sum(w * ruido(rng, N, e) for w, e in ((0.45, 256), (0.3, 64), (0.15, 16), (0.1, 4)))
    tierra = (0.5 * ruido(rng, N, 128) + 0.3 * ruido(rng, N, 32) + 0.2 * ruido(rng, N, 8)) > 0.64   # manchas de tierra
    hoja = ruido(rng, N, 8) > 0.8                          # hojarasca
    pasto = np.stack([0.20 + 0.18 * n, 0.33 + 0.22 * n, 0.12 + 0.10 * n], -1)
    suelo = np.stack([0.36 + 0.12 * n, 0.29 + 0.10 * n, 0.19 + 0.07 * n], -1)
    seca = np.stack([0.45 + 0.10 * n, 0.38 + 0.08 * n, 0.20 + 0.05 * n], -1)
    img = np.where(tierra[..., None], suelo, pasto)
    img = np.where(hoja[..., None], 0.5 * img + 0.5 * seca, img)
    img *= 0.85 + 0.3 * rng.random((N, N, 1)) ** 2         # grano fino
    return Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))


def main():
    d = os.path.join(RAIZ, 'models', 'suelo')
    os.makedirs(os.path.join(d, 'meshes'), exist_ok=True)
    os.makedirs(os.path.join(d, 'materials', 'textures'), exist_ok=True)
    textura().save(os.path.join(d, 'materials', 'textures', 'pasto.png'))
    h, u = LADO / 2, LADO / REPETICION
    with open(os.path.join(d, 'meshes', 'suelo.obj'), 'w') as f:
        f.write('mtllib suelo.mtl\no suelo\n')
        f.write(f'v {-h} {-h} 0\nv {h} {-h} 0\nv {h} {h} 0\nv {-h} {h} 0\n')
        f.write(f'vt 0 0\nvt {u} 0\nvt {u} {u}\nvt 0 {u}\nvn 0 0 1\nusemtl pasto\n')
        f.write('f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n')
    with open(os.path.join(d, 'meshes', 'suelo.mtl'), 'w') as f:
        f.write('newmtl pasto\nKa 1 1 1\nKd 1 1 1\nKs 0.02 0.02 0.02\nmap_Kd ../materials/textures/pasto.png\n')
    with open(os.path.join(d, 'model.sdf'), 'w') as f:
        f.write(f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="suelo">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>{LADO:.0f} {LADO:.0f}</size></plane></geometry>
      </collision>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>meshes/suelo.obj</uri></mesh></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.02 0.02 0.02 1</specular>
          <pbr><metal><albedo_map>materials/textures/pasto.png</albedo_map><roughness>1</roughness>
            <metalness>0</metalness></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")
    with open(os.path.join(d, 'model.config'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<model>\n  <name>suelo</name>\n  <version>1.0</version>\n'
                '  <sdf version="1.9">model.sdf</sdf>\n  <description>Suelo de pasto y tierra '
                '(generado por scripts/generar_suelo.py)</description>\n</model>\n')
    print(f'{d}: {LADO:.0f} m, textura {N}x{N} repetida cada {REPETICION} m')


if __name__ == '__main__':
    main()
