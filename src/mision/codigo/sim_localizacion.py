#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_localizacion.py -- Localización de una detección en el terreno por
intersección rayo-terreno (Sección 4.11) y análisis del error:
(a) propagación de covarianza a primer orden (ec. 4.7 del diseño), y
(b) simulación de Monte Carlo que la valida.

Separa las tres fuentes del error de localización (pregunta P2):
posición del vehículo, orientación del vehículo y detección en la imagen.

Genera imagenes/sim_localizacion.pdf y escribe en pantalla el cuadro que se
reporta en la Sección 5.6.

Uso: python3 sim_localizacion.py
Requiere: numpy, matplotlib
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rng = np.random.default_rng(2027)

# ----------------------------------------------------------------- cámara
W, H = 1280, 960                     # píxeles
HFOV = 1.74                          # rad (modelo mono_cam de PX4)
f = (W/2)/np.tan(HFOV/2)             # ≈ 540 px
K = np.array([[f, 0, W/2], [0, f, H/2], [0, 0, 1.0]])
K_inv = np.linalg.inv(K)
# Orientación de la cámara respecto al cuerpo (ec. 4.3): C_x=-A_y, C_y=-A_x, C_z=-A_z
R_bc = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1.0]])
t_bc = np.array([0, 0, -0.10])       # cámara 10 cm bajo el centro de masa


def R_euler(psi, th, ph):
    """R = Rz(psi) Ry(theta) Rx(phi) (ec. 3.5 del diseño)."""
    c, s = np.cos, np.sin
    Rz = np.array([[c(psi), -s(psi), 0], [s(psi), c(psi), 0], [0, 0, 1]])
    Ry = np.array([[c(th), 0, s(th)], [0, 1, 0], [-s(th), 0, c(th)]])
    Rx = np.array([[1, 0, 0], [0, c(ph), -s(ph)], [0, s(ph), c(ph)]])
    return Rz @ Ry @ Rx


def proyectar(X, xi, eta):
    """Proyección estenopeica de un punto X del marco inercial (ec. 3.26)."""
    R = R_euler(*eta)
    pc = xi + R @ t_bc
    Xc = (R @ R_bc).T @ (X - pc)
    u = K @ Xc
    return u[:2]/u[2]


def localizar(uv, xi, eta, z_t=0.0):
    """Intersección rayo-terreno (ecs. 4.5-4.6)."""
    R = R_euler(*eta)
    pc = xi + R @ t_bc
    r = R @ R_bc @ K_inv @ np.array([uv[0], uv[1], 1.0])
    lam = (z_t - pc[2])/r[2]
    return pc + lam*r


def jacobianos(uv, xi, eta, eps=1e-6):
    """Jacobianos numéricos de la localización respecto a (xi, eta, uv)."""
    def h(v):
        return localizar(v[6:8], v[0:3], v[3:6])[:2]
    v0 = np.concatenate([xi, eta, uv])
    J = np.zeros((2, 8))
    for k in range(8):
        dv = np.zeros(8); dv[k] = eps
        J[:, k] = (h(v0+dv) - h(v0-dv))/(2*eps)
    return J[:, 0:3], J[:, 3:6], J[:, 6:8]        # J_p, J_eta, J_u


def covarianza(uv, xi, eta, sig_p, sig_eta, sig_u):
    """Ec. (4.7): Sigma = J_p S_p J_p' + J_eta S_eta J_eta' + J_u S_u J_u'."""
    Jp, Je, Ju = jacobianos(uv, xi, eta)
    Sp = np.diag([sig_p[0]**2, sig_p[0]**2, sig_p[1]**2])
    Se = (sig_eta**2)*np.eye(3)
    Su = (sig_u**2)*np.eye(2)
    terms = [Jp @ Sp @ Jp.T, Je @ Se @ Je.T, Ju @ Su @ Ju.T]
    return sum(terms), terms


def montecarlo(h, sig_p, sig_eta, sig_u, n=4000):
    """Error RMS horizontal de la localización por Monte Carlo."""
    err2 = np.zeros(n)
    for i in range(n):
        # pose verdadera: altura h, inclinación pequeña aleatoria, guiñada cualquiera
        xi = np.array([0.0, 0.0, h])
        eta = np.array([rng.uniform(-np.pi, np.pi), rng.normal(0, 0.05), rng.normal(0, 0.05)])
        # persona verdadera dentro de la huella
        X = np.array([rng.uniform(-0.35*h, 0.35*h), rng.uniform(-0.3*h, 0.3*h), 0.0])
        uv = proyectar(X, xi, eta)
        # versiones estimadas (con error) de la pose y de la detección
        xi_e = xi + np.array([rng.normal(0, sig_p[0]), rng.normal(0, sig_p[0]),
                              rng.normal(0, sig_p[1])])
        eta_e = eta + rng.normal(0, sig_eta, 3)
        uv_e = uv + rng.normal(0, sig_u, 2)
        xh = localizar(uv_e, xi_e, eta_e)
        err2[i] = np.sum((xh[:2] - X[:2])**2)
    return np.sqrt(err2.mean()), np.percentile(np.sqrt(err2), 90)


if __name__ == "__main__":
    alturas = [20.0, 30.0, 40.0]
    # (sigma_p horizontal, sigma_p vertical) [m], sigma_eta [rad], sigma_u [px]
    nominal = ((0.5, 1.0), np.deg2rad(0.5), 5.0)
    casos = {
        "estimación nominal ($\\sigma_p$=0.5 m, $\\sigma_\\eta$=0.5°, $\\sigma_u$=5 px)": nominal,
        "posición degradada ($\\sigma_p$=2 m)": ((2.0, 3.0), np.deg2rad(0.5), 5.0),
        "orientación degradada ($\\sigma_\\eta$=2°)": ((0.5, 1.0), np.deg2rad(2.0), 5.0),
        "detección degradada ($\\sigma_u$=20 px)": ((0.5, 1.0), np.deg2rad(0.5), 20.0),
    }
    print(f"f = {f:.1f} px")
    print(f"{'caso':62s} {'h':>4s} {'RMSE MC':>8s} {'p90 MC':>7s} {'RMSE lin':>9s} "
          f"{'pos':>6s} {'ori':>6s} {'det':>6s}")
    resultados = {}
    for nombre, (sp, se, su) in casos.items():
        resultados[nombre] = []
        for h in alturas:
            rmse_mc, p90 = montecarlo(h, sp, se, su)
            # covarianza linealizada: inclinación cero y un punto de la imagen 20 % fuera del centro
            xi = np.array([0.0, 0.0, h]); eta = np.zeros(3)
            uv = np.array([W/2 + 0.2*W, H/2 + 0.2*H])     # punto típico, fuera del centro
            S, terms = covarianza(uv, xi, eta, sp, se, su)
            rmse_lin = np.sqrt(np.trace(S))
            contrib = [np.sqrt(np.trace(T)) for T in terms]
            resultados[nombre].append((h, rmse_mc, p90, rmse_lin, contrib))
            print(f"{nombre:62s} {h:4.0f} {rmse_mc:8.2f} {p90:7.2f} {rmse_lin:9.2f} "
                  f"{contrib[0]:6.2f} {contrib[1]:6.2f} {contrib[2]:6.2f}")

    # ---------------------------------------------------------------- figura
    plt.rcParams.update({"font.size": 9, "font.family": "serif"})
    fig, ax = plt.subplots(1, 2, figsize=(6.3, 2.9))
    for nombre, res in resultados.items():
        hs = [r[0] for r in res]
        mc = [r[1] for r in res]
        lin = [r[3] for r in res]
        l, = ax[0].plot(hs, lin, "-", lw=1.2, label=nombre)
        ax[0].plot(hs, mc, "o", color=l.get_color(), ms=4)
    ax[0].set_xlabel("altura de vuelo (m)"); ax[0].set_ylabel("RMSE horizontal (m)")
    ax[0].set_title("líneas: covarianza (4.7); puntos: Monte Carlo", fontsize=8)
    ax[0].set_ylim(0, 4.8); ax[0].legend(fontsize=6, loc="upper left"); ax[0].grid(alpha=0.3)
    # contribuciones en el caso nominal
    nominal = list(resultados.values())[0]
    hs = [r[0] for r in nominal]
    c = np.array([r[4] for r in nominal])**2            # varianzas
    base = np.zeros(len(hs))
    fuentes = ["posición del vehículo", "orientación del vehículo", "detección en la imagen"]
    for k, lab in enumerate(fuentes):
        ax[1].bar(hs, c[:, k], bottom=base, width=5, label=lab)
        base += c[:, k]
    ax[1].set_xlabel("altura de vuelo (m)"); ax[1].set_ylabel("varianza horizontal (m$^2$)")
    ax[1].set_title("descomposición por fuente (caso nominal)", fontsize=8)
    ax[1].legend(fontsize=6); ax[1].grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig("../imagenes/sim_localizacion.pdf")
    print("Figura guardada en ../imagenes/sim_localizacion.pdf")
