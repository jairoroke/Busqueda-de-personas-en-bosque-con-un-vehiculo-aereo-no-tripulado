# -*- coding: utf-8 -*-
"""Geometría de la localización rayo-terreno (Sección 4.11, ecs. 4.3-4.7).

Mismas funciones que codigo/sim_localizacion.py, compartidas por el localizador
y las pruebas. Marco inercial ENU y cuerpo FLU; ángulos eta = (psi, theta, phi)
con R = Rz(psi) Ry(theta) Rx(phi).
"""
import numpy as np

# Orientación de la cámara respecto al cuerpo (ec. 4.3): columnas = ejes de la
# cámara en el cuerpo (derecha de la imagen = -A_y, abajo = -A_x, eje óptico = -A_z).
R_BC = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1.0]])
# En x500_busqueda la cámara está 25 cm debajo del origen de base_link, bajo los
# patines (scripts/generar_vehiculo.py, Z_CAMARA). En el x500_mono_cam_down de PX4
# está 10 cm ARRIBA (y ve el propio vehículo); el diseño original supone 10 cm abajo.
T_BC = np.array([0.0, 0.0, -0.25])


def R_euler(psi, th, ph):
    c, s = np.cos, np.sin
    Rz = np.array([[c(psi), -s(psi), 0], [s(psi), c(psi), 0], [0, 0, 1]])
    Ry = np.array([[c(th), 0, s(th)], [0, 1, 0], [-s(th), 0, c(th)]])
    Rx = np.array([[1, 0, 0], [0, c(ph), -s(ph)], [0, s(ph), c(ph)]])
    return Rz @ Ry @ Rx


def euler_de_R(R):
    """(psi, theta, phi) de R = Rz(psi) Ry(theta) Rx(phi)."""
    theta = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    phi = np.arctan2(R[2, 1], R[2, 2])
    psi = np.arctan2(R[1, 0], R[0, 0])
    return np.array([psi, theta, phi])


def proyectar(X, xi, eta, K, t_bc=T_BC):
    """Píxel (u, v) de un punto X del marco inercial (ec. 3.26); None si queda detrás."""
    R = R_euler(*eta)
    Xc = (R @ R_BC).T @ (np.asarray(X, float) - (xi + R @ t_bc))
    if Xc[2] <= 1e-6:
        return None
    u = K @ Xc
    return u[:2] / u[2]


def localizar(uv, xi, eta, K_inv, z_t, t_bc=T_BC):
    """Intersección rayo-terreno, ecs. (4.5)-(4.6); None si el rayo no baja al terreno."""
    R = R_euler(*eta)
    pc = xi + R @ t_bc
    r = R @ R_BC @ K_inv @ np.array([uv[0], uv[1], 1.0])
    if r[2] >= -1e-6:
        return None
    lam = (z_t - pc[2]) / r[2]
    return pc + lam * r


def covarianza(uv, xi, eta, K_inv, z_t, Sp, Se, Su, eps=1e-6, t_bc=T_BC):
    """Ec. (4.7) con jacobianos numéricos (diferencias centradas); también los tres sumandos."""
    def h(v):
        x = localizar(v[6:8], v[0:3], v[3:6], K_inv, z_t, t_bc)
        return x[:2] if x is not None else np.full(2, np.nan)
    v0 = np.concatenate([xi, eta, uv]); J = np.zeros((2, 8))
    for k in range(8):
        dv = np.zeros(8); dv[k] = eps
        J[:, k] = (h(v0 + dv) - h(v0 - dv)) / (2*eps)
    Jp, Je, Ju = J[:, 0:3], J[:, 3:6], J[:, 6:8]
    terminos = [Jp @ Sp @ Jp.T, Je @ Se @ Je.T, Ju @ Su @ Ju.T]
    return sum(terminos), terminos
