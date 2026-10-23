# -*- coding: utf-8 -*-
"""Utilidades comunes de los nodos: QoS de PX4, comandos y conversión de marcos.

PX4 publica en NED (posición) y FRD (cuerpo); el proyecto trabaja en ENU y FLU,
como Gazebo y el módulo mc_nn_control. (x, y, z)_ENU = P (x, y, z)_NED con
P = [[0,1,0],[1,0,0],[0,0,-1]], y R_ENU-FLU = P R_NED-FRD T con T = diag(1,-1,-1).
"""
import math

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand

QOS_PX4 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

P_NED_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
T_FRD_FLU = np.diag([1.0, -1.0, -1.0])


def ned_a_enu(p):
    return np.array([p[1], p[0], -p[2]], float)


def enu_a_ned(p):
    return [float(p[1]), float(p[0]), float(-p[2])]


def yaw_enu_a_ned(yaw):
    return float(math.pi/2 - yaw)


def R_de_cuaternion(q):
    """Matriz de rotación de un cuaternión (w, x, y, z)."""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def R_enu_de_odometria(msg):
    """Orientación ENU-FLU a partir del cuaternión NED-FRD de VehicleOdometry."""
    return P_NED_ENU @ R_de_cuaternion(msg.q) @ T_FRD_FLU


def comando(nodo, publicador, cmd, p1=0.0, p2=0.0, p3=0.0):
    m = VehicleCommand()
    m.command = cmd
    m.param1, m.param2, m.param3 = float(p1), float(p2), float(p3)
    m.target_system = 1; m.target_component = 1; m.source_system = 1; m.source_component = 1
    m.from_external = True
    m.timestamp = int(nodo.get_clock().now().nanoseconds / 1000)
    publicador.publish(m)


# NED <-> ENU (mundo) y FRD <-> FLU (cuerpo)
_P = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_D = np.diag([1.0, -1.0, -1.0])


def R_enu_flu(q_ned_frd):
    """Orientación en ENU-FLU (la de Gazebo y ROS) a partir del cuaternión NED-FRD de PX4/ArduPilot."""
    return _P @ R_de_cuaternion(q_ned_frd) @ _D
