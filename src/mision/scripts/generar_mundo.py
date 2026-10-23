#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generar_mundo.py -- Genera un mundo forestal de Gazebo Harmonic (archivo SDF)
para PX4 SITL con un nivel de oclusión, una condición de iluminación, viento
opcional y personas en posiciones aleatorias registradas (Sección 4.4.2).

Uso:
  python3 generar_mundo.py --oclusion media --iluminacion alta --viento no \\
                           --semilla 1 --salida ~/mundos

Produce: bosque_<oclusion>_<iluminacion>_<viento>_<semilla>.sdf y
         personas_<...>.json (posiciones y poses verdaderas de las personas)

Diferencias con el Código 8.1 del diseño original:
  - Usa los modelos locales del paquete (model://roble, model://pino,
    model://persona_*, model://suelo), así funciona sin conexión; Gazebo los
    encuentra con GZ_SIM_RESOURCE_PATH=<paquete>/models (lo pone el launch).
  - El nivel de oclusión es la fracción del suelo cubierta por copas (15, 45 y
    75 %), medida sobre una rejilla de 0.5 m mientras se colocan los árboles.
    Las densidades fijas del diseño original (60, 160 y 280 árboles/ha) suponían copas
    de 6 m; los robles de Fuel miden 10 m de diámetro y los pinos 3.2 m, así que
    esas densidades daban otra cobertura. La densidad resultante va en el JSON.
  - Además de "de pie" y "acostada" hay personas "sentadas" (Rescue Randy) y
    tres modelos distintos de persona de pie.
  - El mundo declara sus sistemas (física, sensores, etc.), así que no depende
    del server.config de PX4, que también carga GstCameraSystem (ver README).
"""
import argparse, json, os
import numpy as np

# Copas como cilindros: radio, altura inferior y superior [m] (medidas de las mallas)
COPA = {"roble": (5.0, 1.5, 6.5), "pino": (1.6, 0.8, 5.1)}
COBERTURA = {"baja": 0.15, "media": 0.45, "alta": 0.75}
# Dirección del sol (alta: mediodía; baja: sombras largas) e intensidad
SOL = {"alta": ("0.001 0.3 -0.95", "0.9 0.9 0.9 1", "0.4 0.4 0.4 1"),
       "baja": ("0.6 0.6 -0.35", "0.75 0.65 0.55 1", "0.25 0.25 0.28 1")}
PERSONAS_DE_PIE = ["persona_de_pie", "persona_caminando", "persona_mujer"]
# Altura del punto que se usa como "posición de la persona" (centro visible desde arriba)
POSES = ("de_pie", "acostada", "sentada")

CABECERA = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{nombre}">
    <physics type="ode">
      <max_step_size>{paso}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>{frecuencia}</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>{ambiente}</ambient>
      <background>0.7 0.75 0.8 1</background>
      <shadows>true</shadows>
    </scene>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
{viento}
    <include>
      <uri>model://suelo</uri>
      <name>suelo</name>
    </include>
    <light name="sol" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>{sol}</direction>
      <diffuse>{difusa}</diffuse>
      <specular>0.27 0.27 0.27 1</specular>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>19.4826</latitude_deg>
      <longitude_deg>-99.2447</longitude_deg>
      <elevation>2250</elevation>
    </spherical_coordinates>
"""

VIENTO = """    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <horizontal>
        <magnitude>
          <time_for_rise>5</time_for_rise>
          <sin><amplitude_percent>0.25</amplitude_percent><period>20</period></sin>
          <noise type="gaussian"><mean>0</mean><stddev>0.5</stddev></noise>
        </magnitude>
        <direction>
          <time_for_rise>30</time_for_rise>
          <sin><amplitude>15</amplitude><period>60</period></sin>
          <noise type="gaussian"><mean>0</mean><stddev>3</stddev></noise>
        </direction>
      </horizontal>
      <vertical>
        <noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise>
      </vertical>
    </plugin>
    <wind><linear_velocity>{vx} {vy} 0</linear_velocity></wind>"""

INCLUDE = """    <include>
      <uri>model://{modelo}</uri>
      <name>{nombre}</name>
      <pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
    </include>
"""


def cobertura(arboles, lado, paso=0.5):
    """Fracción del cuadrado de lado `lado` cubierta por al menos una copa (rejilla de `paso` m)."""
    g = np.arange(-lado/2 + paso/2, lado/2, paso)
    X, Y = np.meshgrid(g, g)
    cubierto = np.zeros(X.shape, bool)
    for e, x, y in arboles:
        r = COPA[e][0]
        cubierto |= (X - x)**2 + (Y - y)**2 < r*r
    return float(cubierto.mean())


def generar(oclusion, iluminacion, viento, semilla, lado=120.0, n_personas=None, libre=8.0, paso=0.004,
            vehiculo=None):
    """Con PX4 el vehículo lo agrega PX4 al arrancar (paso de 4 ms, como sus mundos); con ArduPilot
    (paso de 1 ms, el que espera el SITL por JSON) se incluye en el mundo con `vehiculo`."""
    rng = np.random.default_rng(semilla)
    nombre = f"bosque_{oclusion}_{iluminacion}_{viento}_{semilla}"
    sol, difusa, ambiente = SOL[iluminacion]
    partes = [CABECERA.format(nombre=nombre, sol=sol, difusa=difusa, ambiente=ambiente, paso=paso,
                              frecuencia=round(1.0 / paso),
                              viento=VIENTO.format(vx=4.0, vy=0.0) if viento == "si" else "")]
    # árboles: hasta alcanzar la cobertura de copas del nivel de oclusión; el
    # bosque se extiende 15 m más allá del área de búsqueda para que el borde
    # no sea un claro. Se deja libre el punto de despegue (origen).
    objetivo = COBERTURA[oclusion]
    arboles = []
    margen = lado + 30.0
    cob = 0.0
    while cob < objetivo:
        for _ in range(10):
            e = "roble" if rng.uniform() < 0.5 else "pino"
            p = rng.uniform(-margen/2, margen/2, 2)
            if np.linalg.norm(p) > libre + COPA[e][0]:
                arboles.append((e, float(p[0]), float(p[1])))
        cob = cobertura(arboles, lado)
    for i, (e, x, y) in enumerate(arboles):
        partes.append(INCLUDE.format(modelo=e, nombre=f"arbol_{i}", x=x, y=y, z=0,
                                     roll=0, pitch=0, yaw=rng.uniform(0, 2*np.pi)))
    # personas: 1 a 3, de pie, acostadas o sentadas, a más de 1 m de los troncos
    n_personas = n_personas or int(rng.integers(1, 4))
    personas = []
    while len(personas) < n_personas:
        x, y = rng.uniform(-lado/2 + 10, lado/2 - 10, 2)
        if min((np.hypot(x - ax, y - ay) for _, ax, ay in arboles), default=9.0) < 1.0:
            continue
        pose = POSES[int(rng.integers(0, 3))]
        yaw = rng.uniform(0, 2*np.pi)
        if pose == "sentada":
            modelo, roll, z = "persona_sentada", 0.0, 0.0
        else:
            modelo = PERSONAS_DE_PIE[int(rng.integers(0, 3))]
            roll, z = (0.0, 0.0) if pose == "de_pie" else (np.pi/2, 0.15)
        j = len(personas)
        partes.append(INCLUDE.format(modelo=modelo, nombre=f"persona_{j}", x=x, y=y, z=z,
                                     roll=roll, pitch=0, yaw=yaw))
        # ¿está bajo una copa? (vista desde arriba)
        bajo = [i for i, (e, ax, ay) in enumerate(arboles) if np.hypot(x - ax, y - ay) < COPA[e][0]]
        personas.append({"id": j, "x": float(x), "y": float(y), "pose": pose, "modelo": modelo,
                         "yaw": float(yaw), "bajo_copa": bool(bajo)})
    if vehiculo:
        partes.append(INCLUDE.format(modelo=vehiculo, nombre="x500_busqueda_0", x=0, y=0, z=0.24, roll=0, pitch=0, yaw=0))
    partes.append("  </world>\n</sdf>\n")
    verdad = {"mundo": nombre, "oclusion": oclusion, "iluminacion": iluminacion, "viento": viento,
              "semilla": semilla, "lado_m": lado, "n_arboles": len(arboles),
              "arboles_por_ha": round(len(arboles) / (margen**2 / 1e4), 1),
              "cobertura_copas": round(cob, 3), "personas": personas,
              "arboles": [{"especie": e, "x": x, "y": y} for e, x, y in arboles]}
    return nombre, "".join(partes), verdad


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--oclusion", choices=COBERTURA.keys(), default="media")
    ap.add_argument("--iluminacion", choices=SOL.keys(), default="alta")
    ap.add_argument("--viento", choices=["si", "no"], default="no")
    ap.add_argument("--semilla", type=int, default=1)
    ap.add_argument("--lado", type=float, default=120.0)
    ap.add_argument("--personas", type=int, default=None, help="número de personas (por omisión 1 a 3)")
    ap.add_argument("--paso", type=float, default=0.004, help="paso de la física [s] (0.001 con ArduPilot)")
    ap.add_argument("--vehiculo", default=None, help="modelo a incluir en el origen (x500_busqueda_ardupilot)")
    ap.add_argument("--salida", default=".")
    a = ap.parse_args()
    nombre, sdf, verdad = generar(a.oclusion, a.iluminacion, a.viento, a.semilla, a.lado, a.personas,
                                 paso=a.paso, vehiculo=a.vehiculo)
    os.makedirs(a.salida, exist_ok=True)
    with open(os.path.join(a.salida, nombre + ".sdf"), "w") as fh:
        fh.write(sdf)
    with open(os.path.join(a.salida, "personas_" + nombre + ".json"), "w") as fh:
        json.dump(verdad, fh, indent=2)
    bajo = sum(p["bajo_copa"] for p in verdad["personas"])
    print(f"Mundo {nombre}: {verdad['n_arboles']} árboles ({verdad['arboles_por_ha']}/ha), cobertura "
          f"{100*verdad['cobertura_copas']:.0f} %, {len(verdad['personas'])} personas ({bajo} bajo copa)")
