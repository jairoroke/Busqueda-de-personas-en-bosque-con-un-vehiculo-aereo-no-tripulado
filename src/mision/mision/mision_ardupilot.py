#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mision_ardupilot.py -- La misma misión de mision_node.py (barrido en franjas,
re-observación, cambio de controlador, métricas) volada por ArduCopter SITL en
modo GUIDED por MAVLink (pymavlink), en lugar de PX4 en modo offboard.

  - Consignas: SET_POSITION_TARGET_LOCAL_NED (sólo posición y guiñada).
  - Despegue: GUIDED, armar y MAV_CMD_NAV_TAKEOFF a la altura de trabajo.
  - controlador:=red: publica "red" en el tópico de Gazebo /x500/controlador; el
    sistema SelectorMotores deja pasar a los motores los comandos de
    controlador_red en lugar de los de ArduPilot. ArduPilot sigue estimando el
    estado y recibe las mismas consignas (así sus errores, y sus integradores, se
    quedan chicos), y recupera los motores para aterrizar.
  - Estimación: publica en /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry,
    NED/FRD como PX4) la posición y la orientación del EKF de ArduPilot
    (LOCAL_POSITION_NED y ATTITUDE_QUATERNION a 50 Hz), así el localizador es el
    mismo con los dos pilotos. La marca de tiempo es time_boot_ms de ArduPilot,
    que en SITL con lockstep avanza con el reloj de Gazebo. ArduPilot no envía las
    varianzas del EKF en esos mensajes; se usan valores típicos fijos (parámetros
    sigma_pos y sigma_ori).
  - Publica la consigna en /x500/consigna (geometry_msgs/PointStamped, ENU) para
    controlador_red.

Ejecución (con el Python de ~/venv-ardupilot, que tiene pymavlink y ve ROS 2):
  ~/venv-ardupilot/bin/python3 -m mision.mision_ardupilot --ros-args -p conexion:=tcp:127.0.0.1:5760 \\
      -p estrategia:=reobs -p controlador:=cascada -p verdad:=<personas_...json> -p salida:=/tmp/corrida.json
"""
import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleOdometry
from pymavlink import mavutil

from mision.mision_node import Mision
from mision.px4_util import QOS_PX4, R_enu_flu

# SET_POSITION_TARGET_LOCAL_NED: usar posición y guiñada; ignorar velocidad, aceleración y tasa de guiñada.
MASCARA = 0b0000_1001_1111_1000
SYS_STATUS, ATTITUDE_QUATERNION, LOCAL_POSITION_NED = 1, 31, 32


class MisionArduPilot(Mision):
    def conectar(self):
        for nombre, valor in [("conexion", "tcp:127.0.0.1:5760"), ("sigma_pos", 0.3), ("sigma_ori_deg", 0.5),
                              ("tasa_estado_hz", 50.0)]:
            self.declare_parameter(nombre, valor)
        g = lambda n: self.get_parameter(n).value
        self.sigma_pos, self.sigma_ori = g("sigma_pos"), math.radians(g("sigma_ori_deg"))
        self.tasa = g("tasa_estado_hz")
        self.mav = mavutil.mavlink_connection(g("conexion"), source_system=250)
        self.pub_odom = self.create_publisher(VehicleOdometry, "/fmu/out/vehicle_odometry", QOS_PX4)
        self.pub_consigna = self.create_publisher(PointStamped, "/x500/consigna", 10)
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node as GzNode
        self.StringMsg = StringMsg
        self.gz = GzNode()
        self.pub_modo = self.gz.advertise("/x500/controlador", StringMsg)
        self.estado_ap = "conectando"; self.t_ap = time.time()
        self.prearmado = False; self.despegando = False
        self.ned = None; self.vel_ned = np.zeros(3)
        self.create_timer(0.005, self.leer_mavlink)
        self.get_logger().info(f"esperando a ArduPilot en {g('conexion')}")

    # ------------------------------------------------------------ MAVLink
    def pedir_intervalo(self, mensaje, hz):
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mensaje, 1e6 / hz, 0, 0, 0, 0, 0)

    def leer_mavlink(self):
        while True:
            m = self.mav.recv_match(blocking=False)
            if m is None:
                break
            tipo = m.get_type()
            if tipo == "HEARTBEAT" and m.get_srcSystem() == self.mav.target_system:
                self.armado = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif tipo == "SYS_STATUS":
                self.prearmado = bool(m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)
            elif tipo == "LOCAL_POSITION_NED":
                self.ned = np.array([m.x, m.y, m.z]); self.vel_ned = np.array([m.vx, m.vy, m.vz])
                self.nueva_posicion(np.array([m.y, m.x, -m.z]))
            elif tipo == "ATTITUDE_QUATERNION" and self.ned is not None:
                self.publicar_odometria(m)
            elif tipo == "STATUSTEXT":
                self.get_logger().info(f"ArduPilot: {m.text}")
        if self.estado_ap == "conectando" and self.mav.target_system != 0:
            for msg_id in (ATTITUDE_QUATERNION, LOCAL_POSITION_NED):
                self.pedir_intervalo(msg_id, self.tasa)
            self.pedir_intervalo(SYS_STATUS, 2.0)
            self.estado_ap = "preparando"; self.t_ap = time.time()
            self.evento("conectado a ArduPilot; esperando el EKF")

    def publicar_odometria(self, m):
        o = VehicleOdometry()
        o.timestamp = o.timestamp_sample = int(m.time_boot_ms) * 1000
        o.pose_frame = VehicleOdometry.POSE_FRAME_NED
        o.position = [float(v) for v in self.ned]
        o.q = [float(m.q1), float(m.q2), float(m.q3), float(m.q4)]
        self.R = R_enu_flu(o.q)                               # para pasar el LiDAR al marco del mundo
        o.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        o.velocity = [float(v) for v in self.vel_ned]
        o.angular_velocity = [float(m.rollspeed), float(m.pitchspeed), float(m.yawspeed)]
        o.position_variance = [self.sigma_pos**2] * 3
        o.orientation_variance = [self.sigma_ori**2] * 3
        o.velocity_variance = [0.1**2] * 3
        self.pub_odom.publish(o)

    # ------------------------------------------------------------ lo que cambia respecto a PX4
    def iniciar(self):
        pass                                                  # el arranque lo lleva reintentar_armado

    def reintentar_armado(self, dt):
        """GUIDED, armar y despegar, cuando el EKF y las verificaciones previas están listos."""
        espera = time.time() - self.t_ap
        if self.estado_ap == "preparando" and self.prearmado and self.ned is not None:
            self.mav.set_mode("GUIDED")
            self.estado_ap = "armando"; self.t_ap = time.time(); self.evento("GUIDED; armando")
        elif self.estado_ap == "armando":
            if self.armado:
                self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                               mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.h)
                self.estado_ap = "despegando"; self.t_ap = time.time(); self.evento(f"armado; despegando a {self.h} m")
            elif espera > 2.0:
                self.mav.set_mode("GUIDED"); self.mav.arducopter_arm(); self.t_ap = time.time()

    def enviar_consigna(self, p_enu, yaw_enu):
        c = PointStamped(); c.header.stamp = self.get_clock().now().to_msg(); c.header.frame_id = "map"
        c.point.x, c.point.y, c.point.z = map(float, p_enu)
        self.pub_consigna.publish(c)
        if self.fase == "despegue" or self.mav.target_system == 0:
            return                                            # el despegue lo hace MAV_CMD_NAV_TAKEOFF
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASCARA,
            float(p_enu[1]), float(p_enu[0]), float(-p_enu[2]), 0, 0, 0, 0, 0, 0, math.pi/2 - yaw_enu, 0)

    def modo_motores(self, modo):
        m = self.StringMsg(); m.data = modo
        for _ in range(3):
            self.pub_modo.publish(m)

    def activar_red(self):
        self.modo_motores("red")
        self.evento("motores para la red neuronal (SelectorMotores)")

    def devolver_control(self):
        self.modo_motores("ardupilot")
        self.evento("motores para ArduPilot")

    def ordenar_aterrizaje(self):
        self.mav.set_mode("LAND")


def main():
    rclpy.init(); nodo = MisionArduPilot()
    try:
        rclpy.spin(nodo)
    except (SystemExit, KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
