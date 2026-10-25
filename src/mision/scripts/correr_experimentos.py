#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
correr_experimentos.py -- Ejecuta el plan experimental factorial de la Sección 4.12:
para cada combinación de factores y cada semilla lanza la simulación integrada
(ros2 launch mision simulacion.launch.py: mundo, Gazebo, PX4 SITL, agente
uXRCE-DDS, puente, detector, localizador y misión), espera el archivo de
resultados de la misión y cierra todo.

Uso:
  python3 correr_experimentos.py --salida ~/resultados --semillas 30
  python3 correr_experimentos.py --dry-run                        (sólo imprime el plan)
  python3 correr_experimentos.py --semillas 2 --solo oclusion=media,controlador=cascada
  python3 correr_experimentos.py --piloto ardupilot --semillas 30
  python3 correr_experimentos.py --vehiculo x500 --salida ~/resultados_x500     (el x500 de las primeras corridas)

Diferencias con el Código 8.6 del diseño original: cada corrida es un solo `ros2 launch`
(el launch genera el mundo y ordena el arranque), corre en su propia partición de
Gazebo y dominio de ROS 2 (para que no se mezcle con otra simulación abierta), y el
tiempo de cada corrida queda en el registro.
"""
import argparse, itertools, json, os, signal, subprocess, time

FACTORES = {"oclusion": ["baja", "media", "alta"], "iluminacion": ["alta", "baja"],
            "viento": ["no", "si"], "estrategia": ["fija", "reobs"]}


def plan(semillas):
    combos = list(itertools.product(*FACTORES.values()))
    for s in range(1, semillas + 1):
        for c in combos:
            yield dict(zip(FACTORES.keys(), c), semilla=s, controlador="cascada")
    # factor controlador: sólo oclusión media, sol alto, con y sin viento
    for s in range(1, semillas + 1):
        for viento in FACTORES["viento"]:
            for est in FACTORES["estrategia"]:
                yield dict(oclusion="media", iluminacion="alta", viento=viento, estrategia=est, semilla=s,
                           controlador="red")


def nombre_de(c):
    return f"{c['piloto']}_{c['oclusion']}_{c['iluminacion']}_{c['viento']}_{c['estrategia']}_{c['controlador']}_{c['semilla']}"


def correr(c, salida, timeout=900, dominio=42, extra=()):
    archivo = os.path.join(salida, nombre_de(c) + ".json")
    if os.path.exists(archivo):
        return "ya existe"
    env = dict(os.environ, GZ_PARTITION=f"experimentos_{os.getpid()}", ROS_DOMAIN_ID=str(dominio))
    cmd = ["ros2", "launch", "mision", "simulacion.launch.py", "gui:=false", f"salida:={archivo}"] + \
          [f"{k}:={c[k]}" for k in ("vehiculo", "piloto", "oclusion", "iluminacion", "viento", "estrategia", "controlador", "semilla")] + list(extra)
    t0 = time.time()
    with open(os.path.join(salida, "logs", nombre_de(c) + ".log"), "w") as log:
        p = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pass
        finally:
            for sig, espera in ((signal.SIGINT, 20), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
                if p.poll() is not None:
                    break
                try:
                    os.killpg(p.pid, sig); p.wait(timeout=espera)
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    pass
    if os.path.exists(archivo):
        with open(archivo) as fh:
            r = json.load(fh)
        r.update(c); r["tiempo_pared_s"] = time.time() - t0
        with open(archivo, "w") as fh:
            json.dump(r, fh)
        return f"ok ({r.get('personas_localizadas')}/{r.get('personas')} personas, {r['tiempo_pared_s']:.0f} s)"
    return "sin resultado (tiempo agotado)"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--salida", default=os.path.expanduser("~/resultados"))
    ap.add_argument("--semillas", type=int, default=30)
    ap.add_argument("--solo", default="", help="filtro k=v,k=v (por ejemplo oclusion=media,controlador=red)")
    ap.add_argument("--timeout", type=int, default=900)
    ap.add_argument("--dominio", type=int, default=42, help="ROS_DOMAIN_ID de las corridas")
    ap.add_argument("--piloto", choices=["px4", "ardupilot"], default="px4")
    ap.add_argument("--vehiculo", choices=["tarot650", "x500"], default="tarot650")
    ap.add_argument("--dry-run", action="store_true")
    a, extra = ap.parse_known_args()                          # lo demás (k:=v) pasa al launch
    filtro = dict(x.split("=") for x in a.solo.split(",") if x)
    corridas = [dict(c, piloto=a.piloto, vehiculo=a.vehiculo) for c in plan(a.semillas)]
    corridas = [c for c in corridas if all(str(c[k]) == v for k, v in filtro.items())]
    print(f"{len(corridas)} corridas en el plan")
    if a.dry_run:
        for c in corridas[:8]:
            print(c)
        print("...")
    else:
        os.makedirs(os.path.join(a.salida, "logs"), exist_ok=True)
        for k, c in enumerate(corridas):
            print(f"[{k+1}/{len(corridas)}] {c} -> {correr(c, a.salida, a.timeout, a.dominio, extra)}", flush=True)
