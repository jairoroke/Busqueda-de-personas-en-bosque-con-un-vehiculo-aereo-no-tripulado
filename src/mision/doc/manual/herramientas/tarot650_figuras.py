#!/usr/bin/env python3
"""Figuras del Tarot 650 para el libro (capítulo 2).

  figuras/tarot650_motor.png   empuje de un motor, f = kf w^2 hasta w_max
  figuras/tarot650_render.png  el vehículo dibujado con las mallas de su modelo de Gazebo

Los datos salen de models/tarot650_busqueda/model.sdf (los mismos de la tabla
VEHICULOS['tarot650'] de codigo/nn_imitacion.py). Solo usa numpy y matplotlib
(lee los STL binarios a mano), sin interfaz gráfica.

Uso: python3 herramientas/tarot650_figuras.py   (desde doc/manual)
"""
import os
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

AQUI = os.path.dirname(os.path.abspath(__file__))
MANUAL = os.path.dirname(AQUI)
FIGURAS = os.path.join(MANUAL, "figuras")
MODELO = os.path.join(MANUAL, "..", "..", "models", "tarot650_busqueda")
SDF = open(os.path.join(MODELO, "model.sdf")).read()


def valor(etiqueta):
    return float(re.search(rf"<{etiqueta}>([^<]+)</{etiqueta}>", SDF).group(1))


KF = valor("motorConstant")            # 4.24e-5 N/(rad/s)^2
WMAX = valor("maxRotVelocity")         # 680 rad/s
M_CUERPO = valor("mass")               # primer <mass>: base_link, 2.12 kg
M_TOTAL = M_CUERPO + 4*0.02 + 0.05     # + cuatro rotores de 20 g + cámara de 50 g = 2.25 kg
G = 9.81


def motor():
    w = np.linspace(0, WMAX, 400)
    f = KF*w**2
    fh = M_TOTAL*G/4
    wh = np.sqrt(fh/KF)
    fig, ax = plt.subplots(figsize=(4.8, 3.0), dpi=200)
    ax.plot(w, f, lw=1.6, color="C0")
    ax.axvline(wh, color="tab:red", ls="--", lw=1.2)
    ax.axhline(fh, color="tab:red", ls=":", lw=1.2)
    ax.text(wh + 15, fh + 8.5, f"vuelo estacionario\n$\\omega_h$ = {wh:.0f} rad/s, $f_h$ = {fh:.2f} N",
            color="tab:red", fontsize=8)
    ax.set_title(f"Motor del Tarot 650: $f = k_f\\omega^2$ (máx. {KF*WMAX**2:.1f} N)", fontsize=10)
    ax.set_xlabel(r"$\omega$ [rad/s]", fontsize=8)
    ax.set_ylabel("empuje por motor [N]", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGURAS, "tarot650_motor.png"))
    plt.close(fig)
    print(f"tarot650_motor.png: w_h = {wh:.1f} rad/s, f_h = {fh:.3f} N, f_max = {KF*WMAX**2:.2f} N")


def leer_stl(ruta):
    b = open(ruta, "rb").read()
    n = int(np.frombuffer(b[80:84], "<u4")[0])
    d = np.frombuffer(b[84:84 + 50*n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return d["v"].astype(float)


def mallas():
    """Triángulos en el marco del cuerpo y su color difuso, según model.sdf."""
    piezas = []
    for nombre, color in re.findall(r'<visual name="(\w+)">\s*<geometry><mesh><uri>model://tarot650_busqueda/'
                                    r'meshes/\w+\.stl</uri></mesh></geometry>\s*<material><ambient>[^<]*</ambient>'
                                    r'<diffuse>([^<]+)</diffuse>', SDF):
        if nombre == "helice":
            continue
        piezas.append((leer_stl(os.path.join(MODELO, "meshes", nombre + ".stl")),
                       [float(c) for c in color.split()[:3]]))
    for pose, malla in re.findall(r'<link name="rotor_\d">\s*<pose>([^<]+)</pose>.*?meshes/(helice_\d)\.stl',
                                  SDF, flags=re.S):
        t = np.array([float(x) for x in pose.split()[:3]])
        piezas.append((leer_stl(os.path.join(MODELO, "meshes", malla + ".stl")) + t, [0.9, 0.35, 0.05]))
    return piezas


def sombrear(tri, color, luz=np.array([0.4, -0.5, 0.75])):
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    k = 0.45 + 0.55*np.abs(n @ (luz/np.linalg.norm(luz)))
    c = np.clip(np.array(color)[None, :]*k[:, None] + 0.12*k[:, None], 0, 1)
    return c


def caja(centro, lado):
    c = np.array(centro)
    h = lado/2
    v = np.array([[x, y, z] for x in (-h, h) for y in (-h, h) for z in (-h, h)]) + c
    caras = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [v[list(f)] for f in caras]


def render():
    z_cam = -0.24
    piezas = mallas()
    fig = plt.figure(figsize=(7.2, 3.9), dpi=200)

    ax = fig.add_subplot(1, 2, 1, projection="3d")
    for tri, color in piezas:
        ax.add_collection3d(Poly3DCollection(tri, facecolors=sombrear(tri, color), edgecolors="none",
                                             linewidths=0))
    ax.add_collection3d(Poly3DCollection(caja((0, 0, z_cam), 0.04), facecolors="#1f3fa8", edgecolors="#10205a",
                                         linewidths=0.3))
    ax.plot([0, 0], [0, 0], [-0.07, z_cam + 0.02], color="#1f3fa8", lw=0.8)
    ax.set_xlim(-0.45, 0.45); ax.set_ylim(-0.45, 0.45); ax.set_zlim(-0.30, 0.10)
    ax.set_box_aspect((1, 1, 0.4/0.9), zoom=0.95)
    ax.view_init(elev=24, azim=-58)
    ax.set_xlabel("x [m]", fontsize=7, labelpad=-2); ax.set_ylabel("y [m]", fontsize=7, labelpad=-2)
    ax.set_zlabel("z [m]", fontsize=7, labelpad=0)
    ax.tick_params(labelsize=5.5, pad=-1)
    ax.set_title("Vista isométrica", fontsize=9)

    ax2 = fig.add_subplot(1, 2, 2)
    todos = [(t, sombrear(t, c)) for t, c in piezas]
    tri = np.concatenate([t for t, _ in todos])
    col = np.concatenate([c for _, c in todos])
    orden = np.argsort(tri[:, :, 2].mean(axis=1))          # de abajo hacia arriba
    ax2.add_collection(PolyCollection(tri[orden][:, :, :2], facecolors=col[orden], edgecolors="none"))
    ax2.add_patch(plt.Rectangle((-0.02, -0.02), 0.04, 0.04, color="#1f3fa8", alpha=0.35, zorder=0))
    ax2.set_xlim(-0.45, 0.45); ax2.set_ylim(-0.45, 0.45); ax2.set_aspect("equal")
    ax2.set_xlabel("x [m] (adelante)", fontsize=7); ax2.set_ylabel("y [m]", fontsize=7)
    ax2.tick_params(labelsize=6); ax2.grid(alpha=0.3)
    ax2.set_title("Vista superior", fontsize=9)

    fig.text(0.5, 0.015, "Mallas del CAD del Tarot Ironman 650 (models/tarot650_busqueda). En azul, la cámara: "
             "24 cm bajo el centro, mirando hacia abajo.", ha="center", fontsize=6.5)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.16, wspace=0.32)
    fig.savefig(os.path.join(FIGURAS, "tarot650_render.png"))
    plt.close(fig)
    print("tarot650_render.png")


if __name__ == "__main__":
    motor()
    render()
