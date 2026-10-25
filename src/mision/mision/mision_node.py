#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mision_node.py -- Nodo de misión en modo offboard de PX4 (Sección 4.11.5 y
Algoritmo 1): despega, recorre el área en franjas paralelas y, en la estrategia
de re-observación, desvía el recorrido hacia las candidatas dudosas.

Suscripciones
  /fmu/out/vehicle_odometry  px4_msgs/VehicleOdometry
  /fmu/out/vehicle_status_v1 px4_msgs/VehicleStatus
  /fmu/out/register_ext_component_reply  (identificador del modo Neural Control)
  /personas/candidatas       vision_msgs/Detection3DArray (del localizador)
Publicaciones
  /fmu/in/offboard_control_mode  px4_msgs/OffboardControlMode (20 Hz)
  /fmu/in/trajectory_setpoint    px4_msgs/TrajectorySetpoint (20 Hz, NED)
  /fmu/in/vehicle_command        px4_msgs/VehicleCommand (armar, modos, aterrizar)

Al terminar escribe un archivo JSON con el registro y las métricas de la corrida.

Ejecución:
  ros2 run mision mision_node --ros-args -p estrategia:=reobs -p controlador:=cascada \\
      -p lado:=120.0 -p altura:=30.0 -p velocidad:=3.0 -p verdad:=<personas_...json> -p salida:=/tmp/corrida_1.json

Diferencias con el Código 8.5 del diseño original:
  - controlador:=red cambia al modo Neural Control (mc_nn_control) al llegar a la
    altura de trabajo y regresa a offboard antes de aterrizar (sustituye al
    temporizador fijo de modo_neural.py, que podía activar la red a media subida).
  - La consigna avanza a lo más `adelanto_max` metros por delante del vehículo, y el punto
    de ruta cambia cuando el vehículo está a menos de 1.5 m y la consigna ya llegó a él.
  - Con controlador:=red la consigna que se manda va adelantada (`prealimentacion`):
    p_ref + c v_ref, con c = kd/kp de la cascada que imita la red (1.6 s en x, y;
    1.25 s en z). Una cascada PD con esa consigna equivale a una con prealimentación
    de velocidad y sigue la referencia sin atraso; sin ella la red se atrasaba
    kd/kp * v = 4.8 m a 3 m/s y, con el límite de adelanto, avanzaba a ~1.6 m/s.
    `tau_adelanto` (s) filtra la velocidad de referencia del adelanto; por omisión 0
    (sin filtro): filtrarla hace que en las esquinas el adelanto siga apuntando en la
    dirección vieja y el vehículo se pase de la esquina (con 1 s el RMSE de la ruta
    subió de 0.23 a 0.37 m). Sin filtro equivale exactamente a una PD con
    prealimentación.
    Requiere la red entrenada con --adelanto (codigo/nn_imitacion.py). El registro
    guarda la consigna sin adelanto.
  - Si recibe el JSON de personas verdaderas (`verdad`), calcula al final las métricas
    (mision/metricas.py): personas localizadas, falsas alarmas, error de localización
    y RMSE de seguimiento de la ruta. La verdad no se usa para decidir nada en vuelo.
  - Aviso con GPS: cuando una candidata se confirma (confianza acumulada >= s2 y al
    menos `n_aviso` detecciones fusionadas, a más de 5 m de las ya avisadas) publica
    una sola vez (sólo en el barrido o la re-observación y dentro del área de búsqueda
    más 5 m) sus coordenadas en /personas/gps (sensor_msgs/NavSatFix, frame_id
    persona_<k>) y el texto en /personas/alerta (los dos con QoS transient local),
    lo escribe en la terminal y en `registro_personas` (CSV). Con `archivo_mapa`
    escribe ahí (JSON) la ruta y las personas avisadas, que dibuja el módulo de
    MAVProxy mision.mavproxy_trayectoria. El origen del mundo está en mision/geo.py.
  - Evasión de obstáculos (mision/evasion.py, `evasion`): los puntos del LiDAR
    (/dron/lidar/points) pasan al marco del mundo con la pose del piloto y, si el
    tramo recto hacia el siguiente punto pasa a menos de `radio_evasion` de un
    obstáculo a la altura del vehículo, la consigna toma el rumbo libre más cercano
    (histograma polar) o sube. A 30 m sobre el bosque casi nunca actúa.
"""
import json, math, os, time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from px4_msgs.msg import (OffboardControlMode, RegisterExtComponentReply, TrajectorySetpoint, VehicleCommand,
                          VehicleOdometry, VehicleStatus)
from vision_msgs.msg import Detection3DArray

from mision.evasion import Evasion, SueloPlano
from mision.geo import ORIGEN, enu_a_geodesica, texto_dms
from mision.metricas import distancia_a_ruta, evaluar_candidatas
from mision.px4_util import QOS_PX4, comando, enu_a_ned, ned_a_enu, yaw_enu_a_ned, R_enu_flu


class Mision(Node):
    def __init__(self):
        super().__init__("mision")
        for nombre, valor in [("estrategia", "fija"), ("controlador", "cascada"), ("lado", 120.0), ("altura", 30.0),
                              ("velocidad", 3.0), ("adelanto_max", 3.0), ("huella", 70.0), ("s1", 0.3), ("s2", 0.6),
                              ("max_reobs", 2), ("altura_reobs", 20.0), ("espera_reobs", 3.0), ("verdad", ""),
                              ("radio_acierto", 5.0), ("min_detecciones", 1), ("salida", "/tmp/corrida.json"),
                              ("tiempo_max", 900.0), ("prealimentacion", -1.0), ("n_aviso", 10),
                              ("registro_personas", os.path.expanduser("~/personas_localizadas.csv")),
                              ("archivo_mapa", ""), ("origen_gps", list(ORIGEN)),
                              ("evasion", True), ("radio_evasion", 3.0), ("alcance_evasion", 20.0),
                              ("tau_adelanto", 0.0)]:
            self.declare_parameter(nombre, valor)
        g = lambda n: self.get_parameter(n).value
        self.estrategia, self.controlador, self.lado, self.h = g("estrategia"), g("controlador"), g("lado"), g("altura")
        self.v, self.adelanto, self.s1, self.s2 = g("velocidad"), g("adelanto_max"), g("s1"), g("s2")
        self.max_reobs, self.h_reobs, self.espera_reobs = g("max_reobs"), g("altura_reobs"), g("espera_reobs")
        self.verdad, self.radio, self.salida, self.t_max = g("verdad"), g("radio_acierto"), g("salida"), g("tiempo_max")
        # adelanto de la consigna en segundos (x, y, z); -1 = automático: kd/kp de la red si controlador:=red
        c = g("prealimentacion")
        if c < 0:
            c = 1.0 if self.controlador == "red" else 0.0
        self.lead = c*np.array([1.6, 1.6, 1.25])
        self.v_ref = np.zeros(3)
        self.v_lead = np.zeros(3)                             # v_ref filtrada (la que adelanta la consigna)
        self.alfa_lead = 0.05/(g("tau_adelanto") + 0.05)
        self.create_subscription(Detection3DArray, "/personas/candidatas", self.cb_cand, 10)
        self.pos = None                                       # posición actual en ENU
        self.nav = None; self.armado = False; self.mode_id = None
        self.waypoints = self.franjas()                       # lista de puntos ENU (x, y, z)
        self.indice = 1                                       # el 0 es el despegue
        self.consigna = np.array([0.0, 0.0, self.h])
        self.reobs = {}                                       # id candidata -> veces re-observada
        self.dudosas = []                                     # candidatas pendientes (x, y, id)
        self.desvio = None                                    # [punto, t_fin] de la re-observación en curso
        self.candidatas = []                                  # última lista (x, y, conf, n)
        self.fase = "despegue"; self.t_fase = time.time()
        self.contador = 0
        self.t0 = time.time()
        self.registro = {"estrategia": self.estrategia, "controlador": self.controlador, "altura": self.h,
                         "lado": self.lado, "reobservaciones": 0, "trayectoria": [], "consignas": [],
                         "ruta": [p.tolist() for p in self.waypoints], "eventos": []}
        self.avisadas = []                                    # personas confirmadas y avisadas (x, y)
        self.R = np.eye(3)                                    # orientación ENU-FLU estimada por el piloto
        self.evasion = Evasion(SueloPlano(), radio=g("radio_evasion"), alcance=g("alcance_evasion"))
        self.desviado = False
        if g("evasion"):
            self.create_subscription(PointCloud2, "/dron/lidar/points", self.cb_lidar, 5)
        latched = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_gps = self.create_publisher(NavSatFix, "/personas/gps", latched)
        self.pub_alerta = self.create_publisher(String, "/personas/alerta", latched)
        self.registro["avisos"] = []
        self.mapa = {"ruta": [list(self.geo(p)[:2]) for p in self.waypoints], "personas": []}
        self.escribir_mapa()
        self.conectar()
        self.timer = self.create_timer(0.05, self.ciclo)      # 20 Hz
        self.get_logger().info(f"misión {self.estrategia} con controlador {self.controlador}: "
                               f"{len(self.waypoints)} puntos de ruta a {self.h} m")

    # ------------------------------------------------------------ recorrido
    def franjas(self):
        """Barrido en franjas paralelas al eje x con separación = 60 % de la huella."""
        sep = 0.6 * self.get_parameter("huella").value * self.h / 30.0
        ys = np.arange(-self.lado/2 + sep/2, self.lado/2, sep)
        pts = [np.array([0.0, 0.0, self.h])]
        for k, y in enumerate(ys):
            xs = (-self.lado/2, self.lado/2) if k % 2 == 0 else (self.lado/2, -self.lado/2)
            pts.append(np.array([xs[0], y, self.h])); pts.append(np.array([xs[1], y, self.h]))
        pts.append(np.array([0.0, 0.0, self.h]))
        return pts

    # ------------------------------------------------------------ PX4 (mision_ardupilot.py sobrescribe esta parte)
    def conectar(self):
        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.cb_odom, QOS_PX4)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.cb_status, QOS_PX4)
        self.create_subscription(RegisterExtComponentReply, "/fmu/out/register_ext_component_reply", self.cb_reply, QOS_PX4)

    def iniciar(self):
        """Tras 1 s de consignas: modo offboard y armar."""
        comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def reintentar_armado(self, dt):
        if not self.armado and dt > 10.0 and self.contador % 40 == 0:
            self.iniciar()

    def activar_red(self):
        mid = self.mode_id if self.mode_id is not None else VehicleStatus.NAVIGATION_STATE_EXTERNAL1
        comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, float(mid))
        self.evento(f"Neural Control solicitado (nav_state {mid})")

    def devolver_control(self):
        comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def ordenar_aterrizaje(self):
        comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def nueva_posicion(self, p_enu):
        self.pos = p_enu
        if self.contador % 4 == 0 and self.fase != "fin":     # 25 Hz en el registro
            # los desvíos de re-observación (ida, espera y regreso hasta 2 m de la ruta) no cuentan en el seguimiento
            if self.desvio is not None:
                self.regresando = True
            elif getattr(self, "regresando", False) and distancia_a_ruta(self.pos, self.waypoints) < 2.0:
                self.regresando = False
            fase = "reobs" if self.desvio is not None or getattr(self, "regresando", False) else self.fase
            self.registro["trayectoria"].append([time.time()-self.t0, *self.pos.tolist(), fase])

    # ------------------------------------------------------------ callbacks
    def cb_odom(self, msg):
        self.R = R_enu_flu(msg.q)
        self.nueva_posicion(ned_a_enu(msg.position))

    def cb_lidar(self, msg):
        if self.pos is None:
            return
        q = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True).astype(float)
        q = q[np.all(np.isfinite(q), axis=1)]
        q = q[np.linalg.norm(q, axis=1) < self.evasion.alcance + 10.0]
        self.evasion.agregar(self.pos + (q + [0.0, 0.0, 0.1]) @ self.R.T, self.pos)

    def cb_status(self, msg):
        self.nav = msg.nav_state; self.armado = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def cb_reply(self, msg):
        nombre = bytes(msg.name).split(b"\x00")[0].decode(errors="ignore")
        if msg.success and nombre.startswith("Neural"):
            self.mode_id = int(msg.mode_id)

    def cb_cand(self, msg):
        self.candidatas = []
        for i, d in enumerate(msg.detections):
            r = d.results[0]
            x, y, conf = r.pose.pose.position.x, r.pose.pose.position.y, r.hypothesis.score
            n = int(d.id) if d.id.isdigit() else 1
            self.candidatas.append((x, y, conf, n))
            if conf >= self.s2 and n >= self.get_parameter("n_aviso").value and self.fase == "barrido" \
                    and max(abs(x), abs(y)) <= self.lado/2 + 5.0 \
                    and all(np.hypot(x - ax, y - ay) > 5.0 for ax, ay in self.avisadas):
                self.avisar(x, y, conf, n)
            if self.estrategia != "reobs":
                continue
            if self.s1 <= conf < self.s2 and self.reobs.get(i, 0) < self.max_reobs \
                    and all(abs(x-dx) + abs(y-dy) > 1.0 for dx, dy, _ in self.dudosas) \
                    and all(j != i for _, _, j in self.dudosas):
                self.dudosas.append((x, y, i))

    # ------------------------------------------------------------ aviso con GPS
    def geo(self, p_enu):
        return enu_a_geodesica(float(p_enu[0]), float(p_enu[1]), float(p_enu[2]) if len(p_enu) > 2 else 0.0,
                               self.get_parameter("origen_gps").value)

    def escribir_mapa(self):
        ruta = self.get_parameter("archivo_mapa").value
        if not ruta:
            return
        try:
            tmp = ruta + ".tmp"
            with open(tmp, "w") as fh:
                json.dump(self.mapa, fh)
            os.replace(tmp, ruta)
        except OSError as e:
            self.get_logger().error(f"No se pudo escribir {ruta}: {e}")

    def avisar(self, x, y, conf, n):
        """Publica y registra las coordenadas GPS de una persona confirmada (una sola vez)."""
        self.avisadas.append((x, y))
        k = len(self.avisadas)
        lat, lon, alt = self.geo((x, y, 0.0))
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = f"persona_{k}"
        m.status.status = NavSatStatus.STATUS_FIX; m.status.service = NavSatStatus.SERVICE_GPS
        m.latitude, m.longitude, m.altitude = lat, lon, alt
        m.position_covariance = [4.0, 0, 0, 0, 4.0, 0, 0, 0, 16.0]   # ~2 m (error de localización típico < 1 m)
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_gps.publish(m)
        hora = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        texto = (f"PERSONA {k} LOCALIZADA {hora}\n"
                 f"  GPS: {lat:.6f}, {lon:.6f}  ({texto_dms(lat, lon)})\n"
                 f"  Mapa: https://www.google.com/maps?q={lat:.6f},{lon:.6f}\n"
                 f"  Local: ({x:.1f}, {y:.1f}) m   confianza {conf:.2f}, {n} detecciones")
        self.pub_alerta.publish(String(data=texto))
        self.get_logger().warn("\n" + texto)
        self.registro["avisos"].append({"persona": k, "t": round(time.time()-self.t0, 2), "x": float(x), "y": float(y),
                                        "lat": lat, "lon": lon, "confianza": float(conf), "detecciones": int(n)})
        self.mapa["personas"].append([lat, lon, k])
        self.escribir_mapa()
        ruta = self.get_parameter("registro_personas").value
        if ruta:
            try:
                nuevo = not os.path.exists(ruta)
                with open(ruta, "a") as fh:
                    if nuevo:
                        fh.write("fecha,persona,latitud,longitud,x_m,y_m,confianza,detecciones\n")
                    fh.write(f"{hora},{k},{lat:.7f},{lon:.7f},{x:.2f},{y:.2f},{conf:.3f},{n}\n")
            except OSError as e:
                self.get_logger().error(f"No se pudo escribir {ruta}: {e}")

    # ------------------------------------------------------------ consignas
    def enviar_consigna(self, p_enu, yaw_enu):
        mode = OffboardControlMode(); mode.position = True
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint(); sp.position = enu_a_ned(p_enu); sp.yaw = yaw_enu_a_ned(yaw_enu)
        sp.velocity = [math.nan]*3; sp.acceleration = [math.nan]*3
        sp.timestamp = mode.timestamp
        self.pub_sp.publish(sp)

    def publicar(self, p_enu, yaw_enu=0.0):
        p_env = p_enu
        self.v_lead += self.alfa_lead*(self.v_ref - self.v_lead)
        if self.lead.any() and self.fase not in ("despegue", "aterrizaje", "fin"):
            p_env = p_enu + self.lead*self.v_lead              # consigna adelantada (prealimentación)
        self.enviar_consigna(p_env, yaw_enu)
        if self.contador % 5 == 0:
            self.registro["consignas"].append([time.time()-self.t0, *map(float, p_enu)])

    def evento(self, texto):
        self.get_logger().info(texto)
        self.registro["eventos"].append([round(time.time()-self.t0, 2), texto])

    def cambiar(self, fase):
        self.evento(f"{self.fase} -> {fase}")
        self.fase = fase; self.t_fase = time.time()

    def avanzar_hacia(self, objetivo):
        """Mueve la consigna hacia el objetivo a la velocidad v, sin alejarse más de adelanto_max del vehículo."""
        if self.get_parameter("evasion").value:
            objetivo = self.evasion.corregir(self.consigna, objetivo)
            if self.evasion.activa and not self.desviado:
                self.evento(f"evasión: obstáculo en el camino cerca de ({self.pos[0]:.1f}, {self.pos[1]:.1f}), se desvía")
            self.desviado = self.evasion.activa
        delta = objetivo - self.consigna
        dist = np.linalg.norm(delta)
        paso = self.v * 0.05
        candidata = objetivo if dist < paso else self.consigna + paso*delta/dist
        if np.linalg.norm(candidata - self.pos) <= self.adelanto or np.linalg.norm(candidata - self.pos) < \
                np.linalg.norm(self.consigna - self.pos):
            self.v_ref = (candidata - self.consigna)/0.05       # velocidad de la referencia (ciclo de 20 Hz)
            self.consigna = candidata

    # ------------------------------------------------------------ ciclo principal
    def ciclo(self):
        self.contador += 1
        self.v_ref = np.zeros(3)                              # sólo avanzar_hacia la cambia en este ciclo
        if self.contador == 20:
            self.iniciar()
        if self.pos is None or self.fase == "fin":
            self.publicar(self.consigna)
            return
        dt = time.time() - self.t_fase
        if time.time() - self.t0 > self.t_max and self.fase not in ("aterrizaje", "fin"):
            self.evento("tiempo máximo agotado"); self.registro["tiempo_agotado"] = True
            self.aterrizar(); return

        if self.fase == "despegue":
            self.consigna = np.array([0.0, 0.0, self.h])
            if self.contador > 20:
                self.reintentar_armado(dt)
            if np.linalg.norm(self.pos - self.consigna) < 1.0:
                if self.controlador == "red":
                    self.activar_red()
                self.cambiar("barrido")
                self.t_barrido = time.time()
            self.publicar(self.consigna); return

        if self.fase == "aterrizaje":
            self.publicar(self.consigna)
            if (not self.armado and dt > 3.0) or dt > 60.0:
                self.terminar()
            return

        # re-observación en curso: mantener espera_reobs s sobre la candidata y regresar
        if self.desvio is not None:
            punto, t_fin = self.desvio
            self.avanzar_hacia(punto)
            if np.linalg.norm(self.pos - punto) < 1.5:
                if t_fin == 0.0:
                    self.desvio[1] = time.time() + self.espera_reobs
                elif time.time() > t_fin:
                    self.desvio = None
                    self.evento("re-observación terminada")
            self.publicar(self.consigna); return

        if self.indice >= len(self.waypoints):
            self.aterrizar(); return
        objetivo = self.waypoints[self.indice]
        self.avanzar_hacia(objetivo)
        # cambiar de punto cuando el vehículo está cerca y la consigna ya llegó: si la consigna
        # no ha llegado (con la red y la prealimentación el vehículo va a la par de ella), se iría
        # en diagonal al siguiente punto, recortaría la esquina y correría todo el tramo siguiente
        if np.linalg.norm(self.pos - objetivo) < 1.5 and np.linalg.norm(self.consigna - objetivo) < 0.1:
            self.indice += 1
            # al terminar una franja, atender la candidata dudosa más cercana (Algoritmo 1)
            if self.estrategia == "reobs" and self.indice % 2 == 1 and self.dudosas:
                x, y, i = min(self.dudosas, key=lambda c: np.hypot(c[0]-self.pos[0], c[1]-self.pos[1]))
                self.dudosas.remove((x, y, i)); self.reobs[i] = self.reobs.get(i, 0) + 1
                self.registro["reobservaciones"] += 1
                lateral = 0.3 * self.h_reobs                  # desplazamiento lateral: otro punto de vista
                self.desvio = [np.array([x + lateral, y, self.h_reobs]), 0.0]
                self.evento(f"re-observación de la candidata {i} en ({x:.1f}, {y:.1f})")
        self.publicar(self.consigna)

    def aterrizar(self):
        if self.controlador == "red":                        # devolver el control al piloto antes de aterrizar
            self.devolver_control()
        self.registro["duracion_s"] = time.time() - self.t0
        self.registro["duracion_barrido_s"] = time.time() - getattr(self, "t_barrido", self.t0)
        self.ordenar_aterrizaje()
        self.cambiar("aterrizaje")

    def terminar(self):
        self.fase = "fin"
        r = self.registro
        r["candidatas_finales"] = [list(map(float, c)) for c in self.candidatas]
        # error de seguimiento: distancia de la trayectoria a la ruta durante el barrido (sin desvíos)
        tr = [t for t in r["trayectoria"] if t[4] == "barrido"]
        if tr:
            d = np.array([distancia_a_ruta(t[1:4], self.waypoints) for t in tr[::5]])
            r["rmse_seguimiento"] = float(np.sqrt(np.mean(d**2)))
            r["error_max_seguimiento"] = float(d.max())
        if self.verdad:
            with open(self.verdad) as fh:
                verdad = json.load(fh)
            n_min = self.get_parameter("min_detecciones").value
            r.update(evaluar_candidatas(self.candidatas, verdad["personas"], self.s2, self.radio, n_min))
            r["min_detecciones"] = n_min
            # sensibilidad al criterio de confirmación (número de detecciones fusionadas)
            r["por_min_detecciones"] = {
                str(n): {k: v for k, v in evaluar_candidatas(self.candidatas, verdad["personas"], self.s2, self.radio, n).items()
                         if k in ("personas_localizadas", "falsas_alarmas", "error_localizacion_m")}
                for n in (1, 3, 5, 10)}
            r["mundo"] = verdad["mundo"]
            for k in ("oclusion", "iluminacion", "viento", "semilla", "cobertura_copas"):
                r[k] = verdad.get(k)
            self.evento(f"{r['personas_localizadas']}/{r['personas']} personas localizadas, "
                        f"{r['falsas_alarmas']} falsas alarmas, error {r['error_localizacion_m']}")
        with open(self.salida, "w") as fh:
            json.dump(r, fh)
        self.get_logger().info(f"misión terminada en {r.get('duracion_s', 0):.0f} s; "
                               f"RMSE de seguimiento {r.get('rmse_seguimiento', float('nan')):.2f} m; registro en {self.salida}")
        self.timer.cancel()
        raise SystemExit


def main():
    rclpy.init(); nodo = Mision()
    try:
        rclpy.spin(nodo)
    except (SystemExit, KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
