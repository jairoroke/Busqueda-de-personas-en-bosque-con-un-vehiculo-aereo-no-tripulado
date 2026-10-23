#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
controlador_red.py -- La red 15-64-32-4 de codigo/nn_imitacion.py como
controlador del x500 cuando lo pilota ArduPilot SITL (ArduPilot no tiene un
módulo equivalente a mc_nn_control de PX4).

Cada mensaje de odometría de Gazebo (/x500/odometria, 400 Hz, la frecuencia de
entrenamiento) arma las mismas 15 entradas que el módulo de PX4 (convención
ENU-FLU: error de posición, dos primeras filas de R, velocidad inercial y
velocidad angular del cuerpo), evalúa la red en numpy y publica las velocidades de
rotor w_i = w_max sqrt((a_i + 1)/2), las del simulador del entrenamiento, en
/x500/red/motor_speed (gz.msgs.Actuators). SelectorMotores sólo las deja pasar
cuando la misión pide "red"; si la consigna deja de llegar, no publica y el
selector regresa a ArduPilot.

Diferencia con PX4: aquí la red usa el estado verdadero de Gazebo, como los
controladores de ROS 2 de cuadrirotor_sim; en PX4 usa el estimado del EKF. La
localización de personas usa en los dos casos el estimado del piloto.

Ejecución: ros2 run mision controlador_red --ros-args -p pesos:=<paquete>/codigo/pesos_nn_x500_busqueda.npz
"""
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node as GzNode

from mision.px4_util import R_de_cuaternion


class ControladorRed(Node):
    def __init__(self):
        super().__init__("controlador_red")
        for nombre, valor in [("pesos", ""), ("w_max", 1000.0), ("odometria", "/x500/odometria"),
                              ("salida", "/x500/red/motor_speed"), ("consigna_max_s", 0.5)]:
            self.declare_parameter(nombre, valor)
        g = lambda n: self.get_parameter(n).value
        z = np.load(g("pesos"))
        self.W = [z[f"W{i}"] for i in range(3)]; self.b = [z[f"b{i}"] for i in range(3)]
        self.W[0] = z["scale"][:, None] * self.W[0]           # la normalización, absorbida como en exportar_tflite.py
        self.w_max, self.edad_max = g("w_max"), g("consigna_max_s")
        self.lock = threading.Lock()
        self.consigna = None; self.t_consigna = 0.0
        self.n = 0; self.t_inferencia = 0.0
        self.create_subscription(PointStamped, "/x500/consigna", self.cb_consigna, 10)
        self.gz = GzNode()
        self.pub = self.gz.advertise(g("salida"), Actuators)
        self.gz.subscribe(Odometry, g("odometria"), self.cb_odom)
        self.create_timer(10.0, self.informe)
        self.get_logger().info(f"red 15-64-32-4 de {g('pesos')}; w = {self.w_max:.0f} sqrt((a+1)/2) rad/s")

    def cb_consigna(self, msg):
        with self.lock:
            self.consigna = np.array([msg.point.x, msg.point.y, msg.point.z]); self.t_consigna = time.time()

    def cb_odom(self, msg):
        with self.lock:
            if self.consigna is None or time.time() - self.t_consigna > self.edad_max:
                return
            goal = self.consigna.copy()
        t0 = time.perf_counter()
        p = msg.pose.position; q = msg.pose.orientation
        R = R_de_cuaternion((q.w, q.x, q.y, q.z))             # mundo ENU <- cuerpo FLU
        v_cuerpo = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        w_cuerpo = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        obs = np.concatenate([goal - np.array([p.x, p.y, p.z]), R[0, :], R[1, :], R @ v_cuerpo, w_cuerpo])
        h = np.maximum(obs @ self.W[0] + self.b[0], 0.0)
        h = np.maximum(h @ self.W[1] + self.b[1], 0.0)
        a = np.clip(h @ self.W[2] + self.b[2], -1.0, 1.0)
        salida = Actuators()
        for w in self.w_max * np.sqrt((a + 1.0) / 2.0):
            salida.velocity.append(float(w))
        self.pub.publish(salida)
        self.n += 1; self.t_inferencia += time.perf_counter() - t0

    def informe(self):
        if self.n:
            self.get_logger().info(f"{self.n/10:.0f} Hz, {1e6*self.t_inferencia/self.n:.0f} µs por paso")
            self.n = 0; self.t_inferencia = 0.0


def main():
    rclpy.init(); nodo = ControladorRed()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
