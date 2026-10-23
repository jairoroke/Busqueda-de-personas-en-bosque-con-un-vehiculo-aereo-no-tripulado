#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ppo_refinamiento.py -- Segunda fase del entrenamiento del controlador neuronal
(Sección 4.8.3): refina por aprendizaje por refuerzo (PPO, Schulman et al.,
2017) la política 15-64-32-4 obtenida por imitación (pesos_nn.npz), en el
mismo simulador de Python con aleatorización de dominio, usando la recompensa
de la ec. (4.4) del diseño.

Uso: python3 ppo_refinamiento.py --pesos pesos_nn.npz --iteraciones 100 --salida pesos_nn_ppo.npz
     Entrenamiento completo recomendado (Sección 6.5 del diseño original), para el x500 de Gazebo:
     python3 ppo_refinamiento.py --pesos pesos_nn_x500_busqueda.npz --masa 2.114 --iteraciones 300 \
         --horizonte 1024 --log-std -3 --salida pesos_nn_x500_busqueda_ppo.npz
     Versión mejorada (--mejorado), la que se usa para el x500_busqueda:
     python3 ppo_refinamiento.py --pesos pesos_nn_x500_busqueda.npz --masa 2.114 --com 0.00071 0.00071 \
         --adelanto --mejorado --iteraciones 200 --salida pesos_nn_x500_busqueda_ppo.npz
Requiere: numpy, torch, y el archivo nn_imitacion.py en la misma carpeta (simulador y experto)

Por qué divergía la versión original (300 iteraciones: a la par de la imitación hasta la ~100 y
luego 3.2 m de RMSE y 4 fallos de 40):
  1. Con la ec. (4.4) cada paso cuesta -(|e_p|^2 + ...) + 1: con más de ~1 m de error la
     recompensa por paso es negativa (hasta -25 a 5 m) y estrellarse cuesta solo -10 una vez.
     Terminar pronto "conviene", y PPO acaba aprendiéndolo.
  2. Los episodios cortados a los 5 s se trataban como terminales (el crítico creía que ahí
     se acababa el mundo).
  3. El crítico empezaba al azar y el mismo optimizador movía actor y crítico con el mismo
     paso: las primeras ventajas, puro ruido, desarmaban la política de imitación.
--mejorado corrige las tres cosas:
  - recompensa acotada y positiva: r = exp(-|e|^2 / sigma^2) - penalizaciones pequeñas, en
    [-0.2, 1]; estrellarse cuesta -100 (más que cualquier racha de pasos);
  - los cortes por tiempo hacen bootstrap con el valor del estado siguiente;
  - 15 iteraciones de calentamiento del crítico antes de tocar al actor, tasas de
    aprendizaje separadas (actor 3e-5, crítico 1e-3), parada temprana por KL (0.01) y un
    término que penaliza alejarse del experto (clonación como regularizador);
  - se evalúa cada 10 iteraciones y se guarda sólo la mejor política (si ninguna mejora a la
    imitación, se queda la imitación).
Con --adelanto la mitad de los episodios usan la referencia móvil de nn_imitacion.py.
"""
import argparse, time
import numpy as np
import torch, torch.nn as nn
import nn_imitacion as NI
from nn_imitacion import SimX500, DT, SCALE, evaluar, R_euler, experto

torch.set_num_threads(4)
dev = torch.device("cpu")

# ------------------------------------------------------------------ recompensa (ec. 4.4)
W_P, W_R, W_V, W_OMEGA, W_A, W_DELTA, R0 = 1.0, 1.0, 0.05, 0.02, 0.05, 0.5, 1.0


def recompensa(sim, goal, a, a_prev):
    e_p = goal - sim.xi
    R33 = np.cos(sim.eta[1]) * np.cos(sim.eta[2])
    r = -(W_P*e_p @ e_p + W_R*(1.0 - R33) + W_V*sim.v @ sim.v + W_OMEGA*sim.omega_body() @ sim.omega_body()
          + W_A*a @ a + W_DELTA*(a - a_prev) @ (a - a_prev)) + R0
    terminado = np.abs(sim.eta[1:]).max() > np.deg2rad(60) or np.linalg.norm(e_p) > 5.0
    return (r - 10.0, True) if terminado else (r, False)


SIGMA_P, W_ATT_M, W_OMEGA_M, W_DELTA_M, CHOQUE_M = 1.0, 0.1, 0.005, 0.05, -100.0


FORMA = "exp"     # "exp": exp(-|e|^2/sigma^2); "lineal": 1 - |e|/2 (alineada con el RMSE de la evaluación)


def recompensa_acotada(sim, p_ref, a, a_prev):
    """Recompensa de --mejorado: positiva mientras vuele, máxima en la referencia."""
    e_p = p_ref - sim.xi
    R33 = np.cos(sim.eta[1]) * np.cos(sim.eta[2])
    w = sim.omega_body()
    base = np.exp(-(e_p @ e_p)/SIGMA_P**2) if FORMA == "exp" else max(1.0 - 0.5*np.linalg.norm(e_p), -1.0)
    r = (base - W_ATT_M*(1.0 - R33) - W_OMEGA_M*min(w @ w, 20.0)
         - W_DELTA_M*(a - a_prev) @ (a - a_prev))
    terminado = np.abs(sim.eta[1:]).max() > np.deg2rad(60) or np.linalg.norm(e_p) > 8.0
    return (CHOQUE_M, True) if terminado else (r, False)


# ------------------------------------------------------------------ redes
class Actor(nn.Module):
    def __init__(self, pesos, log_std0=-2.0):
        super().__init__()
        self.red = nn.Sequential(nn.Linear(15, 64), nn.ReLU(), nn.Linear(64, 32), nn.ReLU(), nn.Linear(32, 4))
        z = np.load(pesos)
        with torch.no_grad():                           # inicializa con la política de imitación
            for k, capa in enumerate([self.red[0], self.red[2], self.red[4]]):
                capa.weight.copy_(torch.tensor(z[f"W{k}"].T, dtype=torch.float32))
                capa.bias.copy_(torch.tensor(z[f"b{k}"], dtype=torch.float32))
        self.log_std = nn.Parameter(torch.full((4,), log_std0))

    def distribucion(self, obs):
        mu = self.red(obs)
        return torch.distributions.Normal(mu, self.log_std.exp())

    def exportar(self, ruta):
        W = [c.weight.detach().numpy().T.copy() for c in (self.red[0], self.red[2], self.red[4])]
        b = [c.bias.detach().numpy().copy() for c in (self.red[0], self.red[2], self.red[4])]
        np.savez(ruta, **{f"W{i}": w for i, w in enumerate(W)}, **{f"b{i}": v for i, v in enumerate(b)}, scale=SCALE)


class Critico(nn.Module):
    def __init__(self):
        super().__init__()
        self.red = nn.Sequential(nn.Linear(15, 64), nn.ReLU(), nn.Linear(64, 64), nn.ReLU(), nn.Linear(64, 1))

    def forward(self, obs):
        return self.red(obs).squeeze(-1)


# ------------------------------------------------------------------ entornos vectorizados
class Entornos:
    def __init__(self, n, rng, mejorado=False, adelanto=False):
        self.mejorado, self.adelanto = mejorado, adelanto
        self.n, self.rng = n, rng
        self.v_ref = np.zeros((n, 3)); self.p0 = np.zeros((n, 3)); self.movil = np.zeros(n, bool)
        self.sims = [SimX500(rand=True) for _ in range(n)]
        self.goals = [np.zeros(3) for _ in range(n)]
        self.a_prev = np.zeros((n, 4)); self.t = np.zeros(n, int)
        for i in range(n):
            self.reiniciar(i)

    def reiniciar(self, i):
        xi0 = self.rng.uniform(-2.0, 2.0, 3) * np.array([1, 1, 0.5]); eta0 = self.rng.normal(0, 0.1, 3)
        self.sims[i].reset(rand=True, xi0=xi0, eta0=eta0)
        self.goals[i] = np.zeros(3); self.a_prev[i] = 0.0; self.t[i] = 0
        self.movil[i] = self.adelanto and self.rng.random() < 0.5
        if self.movil[i]:                                               # referencia móvil (nn_imitacion)
            ang = self.rng.uniform(0, 2*np.pi); vel = self.rng.uniform(0.5, 4.0)
            self.v_ref[i] = [vel*np.cos(ang), vel*np.sin(ang), self.rng.uniform(-0.5, 0.5)]
            self.p0[i] = self.sims[i].xi - self.rng.uniform(-1.0, 1.0, 3)*np.array([1, 1, 0.3])
            self.sims[i].v = self.v_ref[i]*self.rng.uniform(0.0, 1.0)

    def referencia(self, i):
        """(p_ref, objetivo que ve la red) del entorno i en su paso actual."""
        if not self.movil[i]:
            return self.goals[i], self.goals[i]
        p_ref = self.p0[i] + self.v_ref[i]*self.t[i]*DT
        lead = np.array([NI.ADELANTO_XY, NI.ADELANTO_XY, NI.ADELANTO_Z])
        return p_ref, p_ref + lead*self.v_ref[i]

    def obs(self):
        return np.array([s.obs(self.referencia(i)[1]) * SCALE for i, s in enumerate(self.sims)], np.float32)

    def acciones_experto(self):
        return np.array([experto(s, self.referencia(i)[1]) for i, s in enumerate(self.sims)], np.float32)

    def paso(self, acciones):
        """Devuelve recompensa, fin (terminal o corte), corte por tiempo y observación previa al reinicio."""
        rec = np.zeros(self.n, np.float32); fin = np.zeros(self.n, bool); corte = np.zeros(self.n, bool)
        obs_fin = np.zeros((self.n, 15), np.float32)
        for i, (s, a) in enumerate(zip(self.sims, acciones)):
            a = np.clip(a, -1.0, 1.0)
            s.step(a); self.t[i] += 1
            if self.t[i] == 800 and not self.movil[i]:                  # escalón de consigna a los 2 s
                self.goals[i] = self.rng.uniform(-1.5, 1.5, 3) * np.array([1, 1, 0.5])
            if self.mejorado:
                rec[i], fin[i] = recompensa_acotada(s, self.referencia(i)[0], a, self.a_prev[i])
            else:
                rec[i], fin[i] = recompensa(s, self.goals[i], a, self.a_prev[i])
            self.a_prev[i] = a
            if not fin[i] and self.t[i] >= 2000:                        # episodios de 5 s
                corte[i] = True
                obs_fin[i] = s.obs(self.referencia(i)[1]) * SCALE
            if fin[i] or corte[i]:
                fin[i] = True; self.reiniciar(i)
        return rec, fin, corte, obs_fin


# ------------------------------------------------------------------ PPO
def entrenar(pesos, iteraciones, n_env=32, horizonte=512, gamma=0.99, lam=0.95, clip=0.2, epocas=4,
             lote=2048, lr=3e-4, salida="pesos_nn_ppo.npz", log_std0=-2.0):
    rng = np.random.default_rng(0); torch.manual_seed(0)
    actor, critico = Actor(pesos, log_std0).to(dev), Critico().to(dev)
    opt = torch.optim.Adam(list(actor.parameters()) + list(critico.parameters()), lr=lr)
    envs = Entornos(n_env, rng)
    obs = torch.tensor(envs.obs())
    for it in range(iteraciones):
        t0 = time.time()
        B_obs, B_act, B_logp, B_rec, B_fin, B_val = [], [], [], [], [], []
        for _ in range(horizonte):                                      # ---- recolección de trayectorias
            with torch.no_grad():
                dist = actor.distribucion(obs); act = dist.sample()
                logp = dist.log_prob(act).sum(-1); val = critico(obs)
            rec, fin, _, _ = envs.paso(act.numpy())
            B_obs.append(obs); B_act.append(act); B_logp.append(logp); B_val.append(val)
            B_rec.append(torch.tensor(rec)); B_fin.append(torch.tensor(fin, dtype=torch.float32))
            obs = torch.tensor(envs.obs())
        with torch.no_grad():
            val_sig = critico(obs)
        # ---- ventajas con GAE
        T = horizonte; adv = torch.zeros(T, n_env); ultimo = torch.zeros(n_env)
        for t in reversed(range(T)):
            no_fin = 1.0 - B_fin[t]
            v_sig = val_sig if t == T-1 else B_val[t+1]
            delta = B_rec[t] + gamma*v_sig*no_fin - B_val[t]
            ultimo = delta + gamma*lam*no_fin*ultimo; adv[t] = ultimo
        ret = adv + torch.stack(B_val)
        O = torch.cat(B_obs); A = torch.cat(B_act); LP = torch.cat(B_logp)
        ADV = adv.reshape(-1); RET = ret.reshape(-1)
        ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-8)
        # ---- optimización del objetivo recortado
        N = O.shape[0]
        for _ in range(epocas):
            for idx in torch.randperm(N).split(lote):
                dist = actor.distribucion(O[idx]); logp = dist.log_prob(A[idx]).sum(-1)
                razon = (logp - LP[idx]).exp()
                perdida_pi = -torch.min(razon*ADV[idx], razon.clamp(1-clip, 1+clip)*ADV[idx]).mean()
                perdida_v = 0.5*(critico(O[idx]) - RET[idx]).pow(2).mean()
                opt.zero_grad(); (perdida_pi + perdida_v).backward()
                nn.utils.clip_grad_norm_(list(actor.parameters()) + list(critico.parameters()), 0.5); opt.step()
        rec_media = torch.stack(B_rec).mean().item()
        print(f"iteración {it+1:3d}  recompensa media/paso {rec_media:7.3f}  std acción {actor.log_std.exp().mean():.3f}  "
              f"{time.time()-t0:.0f} s", flush=True)
        if (it+1) % 10 == 0:
            actor.exportar(salida)
            politica = lambda o, actor=actor: np.clip(actor.red(torch.tensor((o*SCALE)[None], dtype=torch.float32)).detach().numpy()[0], -1, 1)
            rmse, fallos = evaluar(politica)
            print(f"  evaluación (40 episodios aleatorizados): RMSE {rmse:.3f} m, fallos {fallos}/40", flush=True)
    actor.exportar(salida)
    return actor


def politica_de(actor):
    return lambda o: np.clip(actor.red(torch.tensor((o*SCALE)[None], dtype=torch.float32)).detach().numpy()[0], -1, 1)


def puntaje(politica, adelanto):
    """Evaluación con semilla fija: RMSE de escalones aleatorizados (+ seguimiento) y fallos."""
    NI.rng = np.random.default_rng(1234)
    r, f = evaluar(politica)
    if adelanto:
        rm, fm = evaluar(politica, movil=True); r, f = 0.5*(r + rm), f + fm
    return r, f


def entrenar_mejorado(pesos, iteraciones, n_env=32, horizonte=512, gamma=0.99, lam=0.95, clip=0.1, epocas=4,
                      lote=2048, lr_actor=3e-5, lr_critico=1e-3, salida="pesos_nn_ppo.npz", log_std0=-3.0,
                      calentamiento=15, kl_max=0.01, w_bc=1.0, adelanto=False):
    rng = np.random.default_rng(0); torch.manual_seed(0)
    actor, critico = Actor(pesos, log_std0).to(dev), Critico().to(dev)
    opt_a = torch.optim.Adam(actor.parameters(), lr=lr_actor)
    opt_c = torch.optim.Adam(critico.parameters(), lr=lr_critico)
    envs = Entornos(n_env, rng, mejorado=True, adelanto=adelanto)
    mejor, mejor_f = puntaje(politica_de(actor), adelanto)
    print(f"imitación (punto de partida): puntaje {mejor:.3f} m, fallos {mejor_f}", flush=True)
    actor.exportar(salida)
    obs = torch.tensor(envs.obs())
    for it in range(iteraciones):
        t0 = time.time()
        B_obs, B_act, B_logp, B_rec, B_fin, B_val, B_exp = [], [], [], [], [], [], []
        for _ in range(horizonte):
            with torch.no_grad():
                dist = actor.distribucion(obs); act = dist.sample()
                logp = dist.log_prob(act).sum(-1); val = critico(obs)
            B_exp.append(torch.tensor(envs.acciones_experto()))
            rec, fin, corte, obs_fin = envs.paso(act.numpy())
            rec = torch.tensor(rec)
            if corte.any():                                             # corte por tiempo: bootstrap
                with torch.no_grad():
                    v_fin = critico(torch.tensor(obs_fin))
                rec = rec + gamma*v_fin*torch.tensor(corte, dtype=torch.float32)
            B_obs.append(obs); B_act.append(act); B_logp.append(logp); B_val.append(val)
            B_rec.append(rec); B_fin.append(torch.tensor(fin, dtype=torch.float32))
            obs = torch.tensor(envs.obs())
        with torch.no_grad():
            val_sig = critico(obs)
        T = horizonte; adv = torch.zeros(T, n_env); ultimo = torch.zeros(n_env)
        for t in reversed(range(T)):
            no_fin = 1.0 - B_fin[t]
            v_sig = val_sig if t == T-1 else B_val[t+1]
            delta = B_rec[t] + gamma*v_sig*no_fin - B_val[t]
            ultimo = delta + gamma*lam*no_fin*ultimo; adv[t] = ultimo
        ret = adv + torch.stack(B_val)
        O = torch.cat(B_obs); A = torch.cat(B_act); LP = torch.cat(B_logp); AE = torch.cat(B_exp)
        ADV = adv.reshape(-1); RET = ret.reshape(-1)
        ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-8)
        N = O.shape[0]; kl = 0.0; actualiza_actor = it >= calentamiento
        for _ in range(epocas):
            for idx in torch.randperm(N).split(lote):
                perdida_v = 0.5*(critico(O[idx]) - RET[idx]).pow(2).mean()
                opt_c.zero_grad(); perdida_v.backward(); nn.utils.clip_grad_norm_(critico.parameters(), 0.5); opt_c.step()
                if not actualiza_actor:
                    continue
                dist = actor.distribucion(O[idx]); logp = dist.log_prob(A[idx]).sum(-1)
                razon = (logp - LP[idx]).exp()
                perdida_pi = -torch.min(razon*ADV[idx], razon.clamp(1-clip, 1+clip)*ADV[idx]).mean()
                perdida_bc = (dist.mean - AE[idx]).pow(2).mean()        # no alejarse del experto
                opt_a.zero_grad(); (perdida_pi + w_bc*perdida_bc).backward()
                nn.utils.clip_grad_norm_(actor.parameters(), 0.5); opt_a.step()
                with torch.no_grad():
                    kl = (LP[idx] - actor.distribucion(O[idx]).log_prob(A[idx]).sum(-1)).mean().item()
            if actualiza_actor and kl > kl_max:                         # parada temprana por KL
                break
        rec_media = torch.stack(B_rec).mean().item()
        print(f"iteración {it+1:3d}  recompensa media/paso {rec_media:6.3f}  KL {kl:7.4f}  "
              f"{'crítico' if not actualiza_actor else 'actor+crítico'}  {time.time()-t0:.0f} s", flush=True)
        if actualiza_actor and (it+1) % 10 == 0:
            r, f = puntaje(politica_de(actor), adelanto)
            nota = ""
            if f <= mejor_f and r < mejor:
                mejor, mejor_f = r, f; actor.exportar(salida); nota = "  <- mejor, guardada"
            print(f"  evaluación: puntaje {r:.3f} m, fallos {f}{nota}", flush=True)
    print(f"Mejor política: puntaje {mejor:.3f} m, fallos {mejor_f} (guardada en {salida})")
    return actor


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--pesos", default="pesos_nn.npz")
    ap.add_argument("--iteraciones", type=int, default=100)
    ap.add_argument("--salida", default="pesos_nn_ppo.npz")
    ap.add_argument("--horizonte", type=int, default=512)
    ap.add_argument("--log-std", type=float, default=-2.0)
    ap.add_argument("--masa", type=float, default=2.0, help="masa del simulador (la lee nn_imitacion.py)")
    ap.add_argument("--com", type=float, nargs=2, default=(0.0, 0.0), help="centro de masa (lo lee nn_imitacion.py)")
    ap.add_argument("--adelanto", action="store_true", help="incluir referencias móviles (lo lee nn_imitacion.py)")
    ap.add_argument("--mejorado", action="store_true", help="recompensa acotada, bootstrap, calentamiento, KL, mejor política")
    ap.add_argument("--forma", choices=["exp", "lineal"], default="exp", help="forma de la recompensa de --mejorado")
    ap.add_argument("--w-bc", type=float, default=1.0, help="peso del término de clonación en --mejorado")
    a = ap.parse_args()
    if a.mejorado:
        FORMA = a.forma
        entrenar_mejorado(a.pesos, a.iteraciones, horizonte=a.horizonte, salida=a.salida,
                          log_std0=a.log_std if a.log_std != -2.0 else -3.0, adelanto=a.adelanto, w_bc=a.w_bc)
    else:
        entrenar(a.pesos, a.iteraciones, horizonte=a.horizonte, salida=a.salida, log_std0=a.log_std)
    print(f"Pesos refinados guardados en {a.salida}; exportar con exportar_tflite.py --pesos {a.salida}")
