#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nn_imitacion.py -- Primera fase del entrenamiento del controlador neuronal
(Sección 4.8): una red 15-64-32-4 con activación ReLU, con la interfaz del
módulo mc_nn_control de PX4 (15 entradas, 4 salidas en [-1,1]), aprende por
imitación (clonación de comportamiento + DAgger) el control en cascada de la
Sección 4.7 en un simulador en Python del modelo de la Sección 4.2 completado
con la matriz de asignación, la dinámica de los motores y la saturación.

Todo se implementa con numpy (incluida la retropropagación) para que el
script no dependa de bibliotecas de aprendizaje profundo; el mismo flujo se
reproduce en PyTorch para la fase de refuerzo (diseño original, Sección 5).

Genera imagenes/nn_imitacion.pdf, guarda los pesos en pesos_nn.npz y escribe
el cuadro de la Sección 5.3.

Uso: python3 nn_imitacion.py [--vehiculo tarot650|x500] [--masa M --salida pesos.npz]
     Para el tarot650_busqueda de Gazebo (el vehículo que se usa):
       python3 nn_imitacion.py --vehiculo tarot650 --masa 2.25 --com 0.00067 0.00067 --adelanto \
           --salida pesos_nn_tarot650_busqueda.npz
     (2.12 kg del cuerpo + 4 rotores de 20 g + cámara de 50 g; la cámara corre el centro
     de masa 0.67 mm en x y en y). Masa, inercias, kf y retardos de los motores del Tarot
     son supuestos (tabla VEHICULOS de cuadrirotor_sim/scripts/generar_modelo.py).

     Con --vehiculo x500 y sin más opciones reproduce el Cuadro 11 (m = 2.0 kg, la masa de base_link). El
     vehículo completo de Gazebo pesa 2.114 kg (cuerpo + 4 rotores de 16 g +
     cámara de 50 g); como la red imita una cascada PD sin acción integral, la
     diferencia de masa deja un error estacionario en z de unos 0.27 m en SITL.

     Para el x500_busqueda de Gazebo:
       python3 nn_imitacion.py --masa 2.114 --com 0.00071 0.00071 --adelanto \
           --salida pesos_nn_x500_busqueda.npz
     --com: centro de masa corrido (x, y) [m]. La cámara (50 g) tiene su centro de
       masa a 3 cm del eje; corre el del vehículo 0.7 mm y, sin integral, la red
       se queda inclinada ~1 grado (~0.2 m de error horizontal en SITL). Con --com
       el simulador y el experto usan la geometría real y la red aprende a
       compensarlo con los motores.
     --adelanto: la mitad de los episodios siguen una referencia que se mueve a
       velocidad constante (hasta 4 m/s) y el objetivo es p_ref + (kd/kp) v_ref,
       la consigna adelantada que manda mision_node con controlador:=red. Con esa
       consigna la cascada PD equivale a una con prealimentación de velocidad y
       sigue la referencia sin atraso; la red tiene que haber visto esos errores
       (hasta ~7 m) para reproducirla.
Requiere: numpy, matplotlib
"""
import argparse
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rng = np.random.default_rng(314200394)

ap = argparse.ArgumentParser()
ap.add_argument("--vehiculo", choices=("tarot650", "x500"), default="x500",
                help="física del vehículo (ver VEHICULOS)")
ap.add_argument("--masa", type=float, default=None, help="masa nominal del vehículo [kg]")
ap.add_argument("--salida", default="pesos_nn.npz")
ap.add_argument("--figura", default="../imagenes/nn_imitacion.pdf")
ap.add_argument("--com", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"),
                help="centro de masa corrido respecto al centro de los rotores [m]")
ap.add_argument("--adelanto", action="store_true",
                help="entrenar también con referencias en movimiento y consigna adelantada")
ap.add_argument("--viento", action="store_true",
                help="entrenar con el arrastre y el viento de Gazebo (WindEffects): F = k m_cuerpo (w - v), k = 0.15, w hasta 6 m/s")
ARGS, _ = ap.parse_known_args()

# ============================================================ vehículo
# Rotores en el orden de PX4 (rotor_0 adelante-derecha ccw, rotor_1 atrás-izquierda ccw,
# rotor_2 adelante-izquierda cw, rotor_3 atrás-derecha cw), marco FLU: (dx, dy) del brazo.
VEHICULOS = {
    # x500 de PX4-gazebo-models (Anexo A); m = 2.0 kg es la masa de base_link.
    # m_viento: masa de los eslabones con <enable_wind> (WindEffects empuja con k m (w - v)).
    "x500": dict(m=2.0, m_viento=2.0, I=(0.02166667, 0.02166667, 0.04), dxy=(0.174, 0.174),
                 kf=8.54858e-6, km=0.016, wmax=1000.0, T_up=0.0125, T_dn=0.025),
    # Tarot 650 de cuadrirotor_sim (inercia total con los rotores, generar_modelo.py).
    "tarot650": dict(m=2.25, m_viento=2.12, I=(0.03412, 0.03609, 0.06596), dxy=(0.2198, 0.212),
                     kf=4.24e-5, km=0.016, wmax=680.0, T_up=0.03, T_dn=0.05),
}
VEH = VEHICULOS[ARGS.vehiculo]
K_VIENTO = 0.15                                       # = FACTOR_VIENTO de scripts/generar_mundo.py
m, g = (ARGS.masa if ARGS.masa is not None else VEH["m"]), 9.81
Ixx, Iyy, Izz = VEH["I"]
kf, km, wmax = VEH["kf"], VEH["km"], VEH["wmax"]
T_up, T_dn = VEH["T_up"], VEH["T_dn"]
fmax = kf*wmax**2                                     # empuje máximo por rotor (x500 8.55 N, Tarot 19.6 N)
dx_, dy_ = VEH["dxy"]
rot_xy = np.array([[dx_, -dy_], [-dx_, dy_], [dx_, dy_], [-dx_, -dy_]])
rot_s = np.array([1.0, 1.0, -1.0, -1.0])
COM = np.array(ARGS.com, float)


def asignacion(com):
    """Matriz de asignación (ec. 4.1) con los pares tomados respecto al centro de masa."""
    return np.vstack([np.ones(4), rot_xy[:, 1] - com[1], -(rot_xy[:, 0] - com[0]), -rot_s*km])


A = asignacion(COM)
A_inv = np.linalg.inv(A)
I_vec = np.array([Ixx, Iyy, Izz])                     # para (M_phi, M_theta, M_psi)

DT = 1/400.0                                          # periodo del lazo (400 Hz)


def R_euler(psi, th, ph):
    c, s = np.cos, np.sin
    return np.array([
        [c(psi)*c(th), c(psi)*s(th)*s(ph)-s(psi)*c(ph), c(psi)*s(th)*c(ph)+s(psi)*s(ph)],
        [s(psi)*c(th), s(psi)*s(th)*s(ph)+c(psi)*c(ph), s(psi)*s(th)*c(ph)-c(psi)*s(ph)],
        [-s(th), c(th)*s(ph), c(th)*c(ph)]])


class SimX500:
    """Modelo (3.12)-(3.17) + asignación + motores de primer orden + saturación."""
    def __init__(self, rand=False):
        self.reset(rand)

    def reset(self, rand=False, xi0=None, eta0=None):
        # aleatorización de dominio (masa, inercia, k_f, constantes de tiempo)
        self.m = m * (rng.uniform(0.8, 1.2) if rand else 1.0)
        self.I = I_vec * (rng.uniform(0.8, 1.2) if rand else 1.0)
        self.kf = kf * (rng.uniform(0.85, 1.15) if rand else 1.0)
        self.Tu = T_up * (rng.uniform(0.7, 1.3) if rand else 1.0)
        self.Td = T_dn * (rng.uniform(0.7, 1.3) if rand else 1.0)
        self.wind = rng.normal(0, 1.5, 3)*np.array([1, 1, 0.2]) if rand else np.zeros(3)
        # viento de Gazebo: velocidad del aire w (horizontal) y arrastre k m_viento (w - v) en los tres ejes
        # Con --viento el arrastre está siempre (también en calma, como en los mundos de Gazebo); en 60 %
        # de los episodios aleatorios hay además viento de hasta 6 m/s. Sin --viento no se sortea nada.
        self.con_aire = bool(ARGS.viento)
        self.w_aire = np.zeros(3)
        if self.con_aire and rand and rng.random() < 0.6:
            ang = rng.uniform(0, 2*np.pi); mag = rng.uniform(0.0, 6.0)
            self.w_aire = mag*np.array([np.cos(ang), np.sin(ang), 0.0])
        # centro de masa: el nominal (--com) más una pequeña incertidumbre
        self.com = COM + (rng.normal(0, 0.0002, 2) if rand and COM.any() else 0.0)
        self.A = asignacion(self.com)
        self.xi = np.zeros(3) if xi0 is None else np.array(xi0, float)
        self.v = np.zeros(3)
        self.eta = np.zeros(3) if eta0 is None else np.array(eta0, float)   # (psi, theta, phi)
        self.deta = np.zeros(3)
        if self.com.any():                                                   # motores en equilibrio
            f_eq = np.linalg.solve(self.A, [self.m*g, 0, 0, 0])
            self.w = np.sqrt(np.maximum(f_eq, 0.0)/self.kf)
        else:
            self.w = np.full(4, np.sqrt(self.m*g/(4*self.kf)))
        return self.obs(np.zeros(3))

    def omega_body(self):
        psi, th, ph = self.eta; dpsi, dth, dph = self.deta
        return np.array([dph - dpsi*np.sin(th),
                         dth*np.cos(ph) + dpsi*np.cos(th)*np.sin(ph),
                         dpsi*np.cos(th)*np.cos(ph) - dth*np.sin(ph)])

    def obs(self, goal):
        """Las 15 entradas del módulo mc_nn_control (convención ENU-FLU)."""
        R = R_euler(*self.eta)
        return np.concatenate([goal - self.xi, R[0, :], R[1, :], self.v, self.omega_body()])

    def step(self, a, nsub=2):
        a = np.clip(a, -1.0, 1.0)
        f_d = 0.5*(a + 1.0)*fmax                                # acción -> empuje deseado por rotor
        w_d = np.sqrt(np.maximum(f_d, 0.0)/self.kf)
        h = DT/nsub
        for _ in range(nsub):
            T = np.where(w_d > self.w, self.Tu, self.Td)
            self.w += h/T*(w_d - self.w)
            self.w = np.clip(self.w, 0.0, wmax)
            fvec = self.kf*self.w**2
            u, Mphi, Mth, Mpsi = self.A @ fvec
            tau = np.array([Mpsi, Mth, Mphi])/np.array([self.I[2], self.I[1], self.I[0]])
            psi, th, ph = self.eta
            acc = np.array([u*(np.sin(ph)*np.sin(psi) + np.cos(ph)*np.cos(psi)*np.sin(th)),
                            u*(np.cos(ph)*np.sin(th)*np.sin(psi) - np.cos(psi)*np.sin(ph)),
                            u*np.cos(th)*np.cos(ph) - self.m*g]) / self.m + self.wind/self.m
            if self.con_aire:
                acc += K_VIENTO*VEH["m_viento"]*(self.w_aire - self.v)/self.m
            self.v += h*acc; self.xi += h*self.v
            self.deta += h*tau; self.eta += h*self.deta
        return self.xi, self.eta


# ============================================================ experto (cascada PD)
kp_z, kd_z, kp_xy, kd_xy, kp_att, kd_att, kp_psi, kd_psi = 2.0, 2.5, 1.0, 1.6, 40.0, 10.0, 10.0, 5.0
tilt_max = np.deg2rad(20.0)


def experto(sim, goal, psi_d=0.0):
    """Cascada PD de la Sección 4.7 expresada en acciones de los motores (nominal m, I)."""
    x, y, z = sim.xi; dx, dy, dz = sim.v; psi, th, ph = sim.eta; dpsi, dth, dph = sim.deta
    az = kp_z*(goal[2]-z) - kd_z*dz
    u = m*(g + az)/max(np.cos(th)*np.cos(ph), 0.5)
    u = np.clip(u, 0.0, 4*fmax)
    ax = kp_xy*(goal[0]-x) - kd_xy*dx
    ay = kp_xy*(goal[1]-y) - kd_xy*dy
    th_d = np.clip(m*(ax*np.cos(psi) + ay*np.sin(psi))/max(u, 1e-3), -tilt_max, tilt_max)
    ph_d = np.clip(m*(ax*np.sin(psi) - ay*np.cos(psi))/max(u, 1e-3), -tilt_max, tilt_max)
    tau_psi = kp_psi*(psi_d-psi) - kd_psi*dpsi
    tau_th = kp_att*(th_d-th) - kd_att*dth
    tau_ph = kp_att*(ph_d-ph) - kd_att*dph
    M = np.array([u, Ixx*tau_ph, Iyy*tau_th, Izz*tau_psi])
    fvec = A_inv @ M
    return np.clip(2.0*fvec/fmax - 1.0, -1.0, 1.0)


# ============================================================ red neuronal (numpy)
SCALE = np.concatenate([np.full(3, 1/5.0), np.ones(6), np.full(3, 1/5.0), np.full(3, 1/5.0)])


class MLP:
    """Red totalmente conectada 15-64-32-4, ReLU, salida lineal; Adam; retropropagación manual."""
    def __init__(self, sizes=(15, 64, 32, 4)):
        self.sizes = sizes
        self.W = [rng.normal(0, np.sqrt(2/sizes[i]), (sizes[i], sizes[i+1]))
                  for i in range(len(sizes)-1)]
        self.b = [np.zeros(sizes[i+1]) for i in range(len(sizes)-1)]
        self.mW = [np.zeros_like(w) for w in self.W]; self.vW = [np.zeros_like(w) for w in self.W]
        self.mb = [np.zeros_like(b) for b in self.b]; self.vb = [np.zeros_like(b) for b in self.b]
        self.t = 0

    def n_params(self):
        return sum(w.size for w in self.W) + sum(b.size for b in self.b)

    def forward(self, X, cache=False):
        H = [X]; Z = []
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            z = H[-1] @ W + b; Z.append(z)
            H.append(np.maximum(z, 0.0) if i < len(self.W)-1 else z)
        return (H, Z) if cache else H[-1]

    def __call__(self, obs):
        return np.clip(self.forward((obs*SCALE)[None, :])[0], -1.0, 1.0)

    def train_step(self, X, Y, lr, b1=0.9, b2=0.999, eps=1e-8):
        H, Z = self.forward(X, cache=True)
        n = X.shape[0]
        dZ = 2.0*(H[-1] - Y)/n                         # gradiente del error cuadrático medio
        gW = [None]*len(self.W); gb = [None]*len(self.b)
        for i in reversed(range(len(self.W))):
            gW[i] = H[i].T @ dZ; gb[i] = dZ.sum(axis=0)
            if i > 0:
                dZ = (dZ @ self.W[i].T) * (Z[i-1] > 0)
        self.t += 1
        for i in range(len(self.W)):
            for P, G, M_, V_ in ((self.W[i], gW[i], self.mW[i], self.vW[i]),
                                 (self.b[i], gb[i], self.mb[i], self.vb[i])):
                M_ *= b1; M_ += (1-b1)*G
                V_ *= b2; V_ += (1-b2)*G*G
                mhat = M_/(1-b1**self.t); vhat = V_/(1-b2**self.t)
                P -= lr*mhat/(np.sqrt(vhat)+eps)
        return np.mean((H[-1]-Y)**2)

    def fit(self, X, Y, epochs=15, batch=256, lr=1e-3):
        n = X.shape[0]
        for ep in range(epochs):
            idx = rng.permutation(n); loss = 0.0; nb = 0
            lr_ep = lr*(0.5**(ep//5))
            for k in range(0, n, batch):
                j = idx[k:k+batch]
                loss += self.train_step(X[j], Y[j], lr_ep); nb += 1
            if ep % 5 == 4 or ep == epochs-1:
                print(f"  época {ep+1:2d}  pérdida {loss/nb:.5f}")


# ============================================================ episodios
ADELANTO_XY, ADELANTO_Z = kd_xy/kp_xy, kd_z/kp_z     # consigna adelantada: p_ref + c v_ref


def episodio_movil(politica, T=6.0, rand=True, ruido=0.0, etiquetar=True, v_max=4.0):
    """Referencia que se mueve a velocidad constante; el objetivo es la consigna adelantada.

    Devuelve observaciones, etiquetas, RMSE respecto a la referencia en la segunda mitad
    (régimen) y la trayectoria (posición, referencia)."""
    sim = SimX500(rand=rand)
    ang = rng.uniform(0, 2*np.pi); vel = rng.uniform(0.5, v_max)
    v_ref = np.array([vel*np.cos(ang), vel*np.sin(ang), rng.uniform(-0.5, 0.5)])
    p0 = np.zeros(3)
    xi0 = p0 + rng.uniform(-1.0, 1.0, 3)*np.array([1, 1, 0.3])
    sim.reset(rand=rand, xi0=xi0, eta0=rng.normal(0, 0.05, 3))
    sim.v = v_ref*rng.uniform(0.0, 1.0)                 # arranca quieto, en marcha o en medio
    lead = np.array([ADELANTO_XY, ADELANTO_XY, ADELANTO_Z])
    n = int(T/DT); X = []; Y = []; err = np.zeros(n); traj = np.zeros((n, 6))
    for k in range(n):
        p_ref = p0 + v_ref*k*DT
        goal = p_ref + lead*v_ref
        o = sim.obs(goal)
        a_exp = experto(sim, goal)
        a = politica(o) if politica is not None else a_exp
        if ruido > 0:
            a = a + rng.normal(0, ruido, 4)
        if etiquetar:
            X.append(o*SCALE); Y.append(a_exp)
        sim.step(a)
        err[k] = np.linalg.norm(sim.xi - (p0 + v_ref*(k+1)*DT))
        traj[k, :3] = sim.xi; traj[k, 3:] = p0 + v_ref*(k+1)*DT
        if np.abs(sim.eta[1:]).max() > np.deg2rad(60) or err[k] > 10.0:
            return np.array(X), np.array(Y), np.inf, traj[:k+1]
    return np.array(X), np.array(Y), np.sqrt(np.mean(err[n//2:]**2)), traj


def episodio(politica, T=4.0, rand=True, ruido=0.0, etiquetar=True, registro=False):
    """Ejecuta una política; devuelve observaciones, etiquetas del experto y métricas."""
    if ARGS.adelanto and rng.random() < 0.5:
        return episodio_movil(politica, rand=rand, ruido=ruido, etiquetar=etiquetar)
    sim = SimX500(rand=rand)
    xi0 = rng.uniform(-2.0, 2.0, 3)*np.array([1, 1, 0.5]); eta0 = rng.normal(0, 0.1, 3)
    goal = np.array([0.0, 0.0, 0.0])
    sim.reset(rand=rand, xi0=xi0, eta0=eta0)
    n = int(T/DT); X = []; Y = []; err = np.zeros(n); traj = np.zeros((n, 6))
    for k in range(n):
        if k == n//2:                                  # escalón de consigna a mitad del episodio
            goal = rng.uniform(-1.5, 1.5, 3)*np.array([1, 1, 0.5])
        o = sim.obs(goal)
        a_exp = experto(sim, goal)
        a = politica(o) if politica is not None else a_exp
        if ruido > 0:
            a = a + rng.normal(0, ruido, 4)
        if etiquetar:
            X.append(o*SCALE); Y.append(a_exp)
        sim.step(a)
        err[k] = np.linalg.norm(sim.xi - goal)
        traj[k, :3] = sim.xi; traj[k, 3:] = sim.eta
        if np.abs(sim.eta[1:]).max() > np.deg2rad(60) or err[k] > 8.0:
            return np.array(X), np.array(Y), np.inf, traj[:k+1]       # episodio fallido
    return np.array(X), np.array(Y), np.sqrt(np.mean(err**2)), traj


def episodio_escalon(politica, rand=True):
    """Episodio de escalones sin sorteo de referencia móvil (para evaluar igual que el Cuadro 11)."""
    guardado = ARGS.adelanto; ARGS.adelanto = False
    try:
        return episodio(politica, rand=rand, etiquetar=False)
    finally:
        ARGS.adelanto = guardado


def sesgo_estacionario(politica, T=8.0):
    """Error horizontal y vertical de régimen al mantener un punto fijo (modelo nominal)."""
    sim = SimX500(rand=False); sim.reset(xi0=[0.0, 0.0, 0.0])
    goal = np.zeros(3)
    for _ in range(int(T/DT)):
        o = sim.obs(goal)
        sim.step(politica(o) if politica is not None else experto(sim, goal))
    return np.hypot(sim.xi[0], sim.xi[1]), sim.xi[2]


def evaluar(politica, n=40, rand=True, movil=False):
    """RMSE medio y episodios fallidos: escalones (como el Cuadro 11) o referencias en movimiento."""
    rmse = []; fallos = 0
    for _ in range(n):
        if movil:
            _, _, r, _ = episodio_movil(politica, rand=rand, etiquetar=False)
        else:
            _, _, r, _ = episodio_escalon(politica, rand=rand)
        if np.isinf(r): fallos += 1
        else: rmse.append(r)
    return (np.mean(rmse) if rmse else np.inf), fallos


if __name__ == "__main__":
    t0 = time.time()
    red = MLP()
    print(f"Red 15-64-32-4: {red.n_params()} parámetros")
    # --- iteración 0: clonación de comportamiento con ruido de exploración
    print("Clonación de comportamiento (experto con ruido de exploración)...")
    Xs, Ys = [], []
    for e in range(120):
        X, Y, _, _ = episodio(None, ruido=0.08)
        Xs.append(X); Ys.append(Y)
    Xd = np.vstack(Xs); Yd = np.vstack(Ys)
    print(f"  {Xd.shape[0]} muestras")
    red.fit(Xd, Yd, epochs=15)
    rmse, fallos = evaluar(red)
    print(f"  evaluación: RMSE {rmse:.3f} m, episodios fallidos {fallos}/40")
    # --- iteraciones de DAgger: estados visitados por la red, etiquetas del experto
    for it in range(1, 4):
        print(f"DAgger, iteración {it}...")
        for e in range(60):
            X, Y, _, _ = episodio(red, ruido=0.02)
            if len(X): Xs.append(X); Ys.append(Y)
        Xd = np.vstack(Xs); Yd = np.vstack(Ys)
        print(f"  {Xd.shape[0]} muestras acumuladas")
        red.fit(Xd, Yd, epochs=15)
        rmse, fallos = evaluar(red)
        print(f"  evaluación: RMSE {rmse:.3f} m, episodios fallidos {fallos}/40")
    rmse_exp, fallos_exp = evaluar(None)
    print(f"Experto (cascada PD): RMSE {rmse_exp:.3f} m, episodios fallidos {fallos_exp}/40")
    rmse_nom, _ = evaluar(red, rand=False); rmse_exp_nom, _ = evaluar(None, rand=False)
    print(f"Sin aleatorización: red {rmse_nom:.3f} m, experto {rmse_exp_nom:.3f} m")
    if ARGS.adelanto:
        rm, fm = evaluar(red, movil=True); rme, fme = evaluar(None, movil=True)
        print(f"Referencia en movimiento (hasta 4 m/s, régimen): red {rm:.3f} m ({fm} fallos), "
              f"experto {rme:.3f} m ({fme} fallos)")
    sh, sz = sesgo_estacionario(red); she, sze = sesgo_estacionario(None)
    print(f"Error de régimen en punto fijo: red {100*sh:.1f} cm horizontal, {100*sz:.1f} cm vertical; "
          f"experto {100*she:.1f} cm, {100*sze:.1f} cm")
    np.savez(ARGS.salida, **{f"W{i}": w for i, w in enumerate(red.W)},
             **{f"b{i}": b for i, b in enumerate(red.b)}, scale=SCALE)
    print(f"Pesos guardados en {ARGS.salida}; tiempo total: {time.time()-t0:.0f} s")

    # ---------------------------------------------------------------- figura
    rng2 = np.random.default_rng(7)

    def demo(politica):
        sim = SimX500(rand=False); sim.reset(xi0=[1.5, -1.0, 0.5], eta0=[0.3, 0.0, 0.0])
        goal = np.zeros(3); n = int(6.0/DT); tr = np.zeros((n, 7)); acts = np.zeros((n, 4))
        for k in range(n):
            if k == n//2: goal = np.array([1.0, 1.0, -0.5])
            o = sim.obs(goal)
            a = politica(o) if politica is not None else experto(sim, goal)
            acts[k] = np.clip(a, -1, 1); sim.step(a)
            tr[k, :3] = sim.xi; tr[k, 3:6] = sim.eta; tr[k, 6] = np.linalg.norm(sim.xi-goal)
        return tr, acts
    tr_e, ac_e = demo(None); tr_n, ac_n = demo(red)
    t = np.arange(tr_e.shape[0])*DT
    plt.rcParams.update({"font.size": 9, "font.family": "serif"})
    fig, ax = plt.subplots(2, 2, figsize=(6.3, 4.4), sharex=True)
    for i, lab in enumerate(["$x$", "$y$", "$z$"]):
        ax[0, 0].plot(t, tr_e[:, i], "--", lw=1.0, color=f"C{i}")
        ax[0, 0].plot(t, tr_n[:, i], "-", lw=1.0, color=f"C{i}", label=lab)
    ax[0, 0].set_ylabel("posición (m)"); ax[0, 0].legend(fontsize=7)
    ax[0, 0].set_title("líneas: red; discontinuas: cascada", fontsize=8)
    for i, lab in enumerate([r"$\psi$", r"$\theta$", r"$\phi$"]):
        ax[0, 1].plot(t, np.rad2deg(tr_e[:, 3+i]), "--", lw=1.0, color=f"C{i}")
        ax[0, 1].plot(t, np.rad2deg(tr_n[:, 3+i]), "-", lw=1.0, color=f"C{i}", label=lab)
    ax[0, 1].set_ylabel("ángulos (grados)"); ax[0, 1].legend(fontsize=7)
    ax[1, 0].plot(t, tr_e[:, 6], "--", label="cascada PD")
    ax[1, 0].plot(t, tr_n[:, 6], "-", label="red 15-64-32-4")
    ax[1, 0].set_ylabel("error de posición (m)"); ax[1, 0].set_xlabel("tiempo (s)")
    ax[1, 0].legend(fontsize=7)
    for i in range(4):
        ax[1, 1].plot(t, ac_n[:, i], lw=0.8, label=f"motor {i}")
    ax[1, 1].set_ylabel("acción de la red $a_i$"); ax[1, 1].set_xlabel("tiempo (s)")
    ax[1, 1].legend(fontsize=6, ncol=2)
    for a in ax.ravel(): a.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(ARGS.figura)
    print(f"Figura guardada en {ARGS.figura}")
