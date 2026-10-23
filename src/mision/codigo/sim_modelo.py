#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_modelo.py -- Simulación numérica de referencia del modelo dinámico del
cuadrirrotor (Sanchez, Garcia-Carrillo, Rondon y Garcia, 2010, ecs. 7.9-7.14)
con los parámetros del vehículo x500 de PX4/Gazebo (Anexo A) y el control
base de vuelo estacionario de la Etapa 1.

Genera la figura imagenes/sim_modelo.pdf y el cuadro de valores que se
reportan en la tabla de validación del modelo.

Uso: python3 sim_modelo.py
Requiere: numpy, scipy, matplotlib
"""
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# --------------------------------------------------------------------------
# Parámetros del vehículo (modelo x500 de PX4-gazebo-models, Anexo A)
# --------------------------------------------------------------------------
m = 2.0            # kg, masa total
g = 9.81           # m/s^2
Ixx = 0.02166667   # kg m^2
Iyy = 0.02166667   # kg m^2
Izz = 0.04         # kg m^2
d = 0.174          # m, desplazamiento de cada rotor en x y en y (configuración en X)
kf = 8.54858e-6    # N s^2/rad^2, constante de empuje f_i = kf * w_i^2
km = 0.016         # m, constante de momento tau_i = km * f_i
wmax = 1000.0      # rad/s, velocidad angular máxima de cada rotor

# Posición (x_i, y_i) y sentido de giro s_i (+1 antihorario, -1 horario) de los
# cuatro rotores, en el orden de PX4 (rotor_0 ... rotor_3)
rot_xy = np.array([[d, -d], [-d, d], [d, d], [-d, -d]])
rot_s = np.array([1.0, 1.0, -1.0, -1.0])

# Matriz de asignación [u, M_phi, M_theta, M_psi]^T = A [f_0, f_1, f_2, f_3]^T
A = np.vstack([np.ones(4),
               rot_xy[:, 1],        # M_phi   =  sum f_i * y_i
               -rot_xy[:, 0],       # M_theta = -sum f_i * x_i
               -rot_s * km])        # M_psi   = -sum s_i * km * f_i
A_inv = np.linalg.inv(A)

# --------------------------------------------------------------------------
# Ganancias del control base (PD) -- Etapa 1
# --------------------------------------------------------------------------
kp_z, kd_z = 2.0, 2.5          # altitud
kp_xy, kd_xy = 1.0, 1.6        # posición horizontal (aceleración deseada)
kp_att, kd_att = 40.0, 10.0    # ángulos de alabeo y cabeceo
kp_psi, kd_psi = 10.0, 5.0     # guiñada
tilt_max = np.deg2rad(20.0)    # límite de inclinación deseada


# Referencias: despegue con rampa de 1 m/s hasta z=5 m; en t=8 s escalón a
# (x,y)=(2,-1) m con guiñada nula
def referencia(t):
    z_d = min(1.0*t, 5.0)
    if t < 8.0:
        return np.array([0.0, 0.0, z_d]), 0.0
    return np.array([2.0, -1.0, z_d]), 0.0


def control(t, s):
    x, y, z, dx, dy, dz, psi, th, ph, dpsi, dth, dph = s
    p_d, psi_d = referencia(t)
    # altitud -> empuje total
    az = kp_z*(p_d[2]-z) - kd_z*dz
    u = m*(g + az)/max(np.cos(th)*np.cos(ph), 0.5)
    u = np.clip(u, 0.0, 4*kf*wmax**2)
    # posición horizontal -> ángulos deseados (inversión para ángulos pequeños)
    ax = kp_xy*(p_d[0]-x) - kd_xy*dx
    ay = kp_xy*(p_d[1]-y) - kd_xy*dy
    th_d = np.clip(m*(ax*np.cos(psi) + ay*np.sin(psi))/max(u, 1e-3), -tilt_max, tilt_max)
    ph_d = np.clip(m*(ax*np.sin(psi) - ay*np.cos(psi))/max(u, 1e-3), -tilt_max, tilt_max)
    # orientación -> aceleraciones angulares (tau_psi, tau_theta, tau_phi)
    tau_psi = kp_psi*(psi_d-psi) - kd_psi*dpsi
    tau_th = kp_att*(th_d-th) - kd_att*dth
    tau_ph = kp_att*(ph_d-ph) - kd_att*dph
    return u, tau_psi, tau_th, tau_ph, th_d, ph_d


def modelo(t, s):
    x, y, z, dx, dy, dz, psi, th, ph, dpsi, dth, dph = s
    u, tau_psi, tau_th, tau_ph, _, _ = control(t, s)
    # Ecuaciones (7.9)-(7.14)
    ddx = u*(np.sin(ph)*np.sin(psi) + np.cos(ph)*np.cos(psi)*np.sin(th))/m
    ddy = u*(np.cos(ph)*np.sin(th)*np.sin(psi) - np.cos(psi)*np.sin(ph))/m
    ddz = u*np.cos(th)*np.cos(ph)/m - g
    return [dx, dy, dz, ddx, ddy, ddz, dpsi, dth, dph, tau_psi, tau_th, tau_ph]


if __name__ == "__main__":
    s0 = np.zeros(12)
    T = 20.0
    t_eval = np.linspace(0.0, T, 4001)
    sol = solve_ivp(modelo, (0.0, T), s0, t_eval=t_eval, method="RK45",
                    rtol=1e-8, atol=1e-10)
    t, S = sol.t, sol.y

    # Señales de control y velocidades de los rotores a posteriori
    U = np.array([control(ti, S[:, i]) for i, ti in enumerate(t)])
    u, tau_psi, tau_th, tau_ph = U[:, 0], U[:, 1], U[:, 2], U[:, 3]
    M = np.vstack([u, Ixx*tau_ph, Iyy*tau_th, Izz*tau_psi])   # momentos físicos
    F = A_inv @ M                                             # empuje por rotor
    W = np.sqrt(np.clip(F, 0.0, None)/kf)                     # rad/s

    # Valores que se reportan en la Sección 5.1
    w_hover = np.sqrt(m*g/(4*kf))

    def t_asent(sig, ref, tol, t0):
        err = np.abs(sig - ref)
        idx = np.where((t >= t0) & (err > tol))[0]
        return (t[idx[-1]] - t0) if len(idx) else 0.0
    print(f"Velocidad de rotor en vuelo estacionario: {w_hover:.1f} rad/s "
          f"({w_hover*60/(2*np.pi):.0f} rpm), {100*w_hover/wmax:.1f} % de w_max")
    print(f"Empuje máximo total: {4*kf*wmax**2:.2f} N; relación empuje/peso: "
          f"{4*kf*wmax**2/(m*g):.2f}")
    print(f"Tiempo de asentamiento (2 %) en z tras la rampa (t=5 s): "
          f"{t_asent(S[2], 5.0, 0.1, 5.0):.2f} s")
    print(f"Sobrepaso en z: {100*(S[2].max()-5.0)/5.0:.2f} %")
    print(f"Tiempo de asentamiento (2 %) en x tras el escalón: "
          f"{t_asent(S[0], 2.0, 0.04, 8.0):.2f} s")
    print(f"Inclinación máxima: theta {np.rad2deg(np.abs(S[7]).max()):.1f} deg, "
          f"phi {np.rad2deg(np.abs(S[8]).max()):.1f} deg")
    print(f"Velocidad máxima de rotor: {W.max():.1f} rad/s ({100*W.max()/wmax:.1f} % de w_max)")
    print(f"Error final: |x-2|={abs(S[0, -1]-2):.4f} m, |y+1|={abs(S[1, -1]+1):.4f} m, "
          f"|z-5|={abs(S[2, -1]-5):.4f} m")

    # ---------------------------------------------------------------- figura
    plt.rcParams.update({"font.size": 9, "font.family": "serif"})
    fig, ax = plt.subplots(2, 2, figsize=(6.3, 4.6), sharex=True)
    ax[0, 0].plot(t, S[0], label="$x$"); ax[0, 0].plot(t, S[1], label="$y$")
    ax[0, 0].plot(t, S[2], label="$z$")
    ax[0, 0].plot(t, [referencia(ti)[0][0] for ti in t], "k:", lw=0.8)
    ax[0, 0].plot(t, [referencia(ti)[0][1] for ti in t], "k:", lw=0.8)
    ax[0, 0].plot(t, [referencia(ti)[0][2] for ti in t], "k:", lw=0.8, label="referencia")
    ax[0, 0].set_ylabel("posición (m)"); ax[0, 0].legend(loc="center right", fontsize=7)
    ax[0, 1].plot(t, np.rad2deg(S[8]), label=r"$\phi$")
    ax[0, 1].plot(t, np.rad2deg(S[7]), label=r"$\theta$")
    ax[0, 1].plot(t, np.rad2deg(S[6]), label=r"$\psi$")
    ax[0, 1].set_ylabel("ángulos (grados)"); ax[0, 1].legend(fontsize=7)
    ax[1, 0].plot(t, u, label="$u$"); ax[1, 0].axhline(m*g, color="k", ls=":", lw=0.8, label="$mg$")
    ax[1, 0].set_ylabel("empuje total (N)"); ax[1, 0].set_xlabel("tiempo (s)")
    ax[1, 0].legend(fontsize=7)
    for i in range(4):
        ax[1, 1].plot(t, W[i], lw=0.9, label=f"rotor {i}")
    ax[1, 1].axhline(wmax, color="k", ls="--", lw=0.8, label=r"$\omega_{\max}$")
    ax[1, 1].axhline(w_hover, color="k", ls=":", lw=0.8, label=r"$\omega_h$")
    ax[1, 1].set_ylabel("velocidad de rotor (rad/s)"); ax[1, 1].set_xlabel("tiempo (s)")
    ax[1, 1].legend(fontsize=6, ncol=2)
    for a in ax.ravel():
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig("../imagenes/sim_modelo.pdf")
    print("Figura guardada en ../imagenes/sim_modelo.pdf")
