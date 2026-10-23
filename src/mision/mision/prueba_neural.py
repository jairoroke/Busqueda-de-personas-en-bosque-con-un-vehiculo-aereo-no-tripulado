#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
prueba_neural.py -- Verificación del modo Neural Control en SITL (pasos 4 y 5 de
la Sección 7.5 del diseño original): despega en modo offboard con el control en cascada
de PX4, se mantiene a la altura de prueba, cambia al modo Neural Control y le
aplica la misma secuencia de escalones que la figura de nn_imitacion.py
(relativa al punto de vuelo estacionario). Al final regresa a offboard y
aterriza.

Registra en un CSV la consigna, la posición estimada, el modo y, si PX4 exporta
/fmu/out/neural_control (scripts/instalar_px4.sh lo agrega), las entradas de
la red y los comandos de motor. Imprime el error por tramo.

Ejecución: ros2 run mision prueba_neural --ros-args -p altura:=5.0 -p salida:=/tmp/prueba_neural.csv
"""
import csv
import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from px4_msgs.msg import (NeuralControl, OffboardControlMode, RegisterExtComponentReply, TrajectorySetpoint,
                          VehicleCommand, VehicleOdometry, VehicleStatus)

from mision.px4_util import QOS_PX4, comando, enu_a_ned, ned_a_enu, R_enu_de_odometria, yaw_enu_a_ned

# Escalones relativos al punto de vuelo estacionario (ENU, m) y su duración (s)
ESCALONES = [((0.0, 0.0, 0.0), 8.0), ((1.0, 1.0, -0.5), 8.0), ((0.0, 0.0, 0.0), 8.0),
             ((-1.5, 1.0, 0.5), 8.0), ((0.0, 0.0, 0.0), 8.0)]


class PruebaNeural(Node):
    def __init__(self):
        super().__init__("prueba_neural")
        for nombre, valor in [("altura", 5.0), ("salida", "/tmp/prueba_neural.csv"), ("controlador", "red"),
                              ("espera_hover_s", 6.0)]:
            self.declare_parameter(nombre, valor)
        g = lambda n: self.get_parameter(n).value
        self.h, self.salida, self.controlador, self.espera = g("altura"), g("salida"), g("controlador"), g("espera_hover_s")
        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.cb_odom, QOS_PX4)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.cb_status, QOS_PX4)
        self.create_subscription(RegisterExtComponentReply, "/fmu/out/register_ext_component_reply", self.cb_reply, QOS_PX4)
        self.create_subscription(NeuralControl, "/fmu/out/neural_control", self.cb_nn, QOS_PX4)
        self.pos = None; self.R = None; self.nav = None; self.armado = False
        self.mode_id = None; self.nn = None
        self.fase = "despegue"; self.t_fase = time.time(); self.k = 0
        self.consigna = np.array([0.0, 0.0, self.h]); self.base = self.consigna.copy()
        self.filas = []; self.t0 = time.time(); self.tramos = []
        self.timer = self.create_timer(0.02, self.ciclo)               # 50 Hz
        self.get_logger().info(f"prueba del controlador '{self.controlador}' a {self.h} m")

    def cb_odom(self, msg):
        self.pos = ned_a_enu(msg.position); self.R = R_enu_de_odometria(msg)

    def cb_status(self, msg):
        self.nav = msg.nav_state; self.armado = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def cb_reply(self, msg):
        nombre = bytes(msg.name).split(b"\x00")[0].decode(errors="ignore")
        if msg.success and nombre.startswith("Neural"):
            self.mode_id = int(msg.mode_id)
            self.get_logger().info(f"modo Neural Control registrado con mode_id = {self.mode_id}")

    def cb_nn(self, msg):
        self.nn = msg

    def publicar(self):
        mode = OffboardControlMode(); mode.position = True
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint(); sp.position = enu_a_ned(self.consigna); sp.yaw = yaw_enu_a_ned(0.0)
        sp.velocity = [math.nan]*3; sp.acceleration = [math.nan]*3
        sp.timestamp = mode.timestamp
        self.pub_sp.publish(sp)

    def cambiar(self, fase):
        self.get_logger().info(f"{self.fase} -> {fase}")
        self.fase = fase; self.t_fase = time.time()

    def ciclo(self):
        self.k += 1
        self.publicar()
        if self.k == 50:
            comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        if self.pos is None:
            return
        dt = time.time() - self.t_fase
        if self.fase == "despegue" and abs(self.pos[2] - self.h) < 0.3 and dt > 8.0:
            self.cambiar("hover")
        elif self.fase == "hover" and dt > self.espera:
            if self.controlador == "red":
                mid = self.mode_id if self.mode_id is not None else VehicleStatus.NAVIGATION_STATE_EXTERNAL1
                comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, float(mid))
            self.cambiar("escalones"); self.i_esc = 0
            self.base = self.pos.copy(); self.base[2] = self.h
        elif self.fase == "escalones":
            off, dur = ESCALONES[self.i_esc]
            self.consigna = self.base + np.array(off)
            if dt > dur:
                self.i_esc += 1; self.t_fase = time.time()
                if self.i_esc >= len(ESCALONES):
                    comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.cambiar("regreso")
        elif self.fase == "regreso" and dt > 3.0:
            comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.cambiar("aterrizaje")
        elif self.fase == "aterrizaje" and (dt > 25.0 or (not self.armado and dt > 3.0)):
            self.terminar()
            return
        inclinacion = math.degrees(math.acos(max(-1.0, min(1.0, self.R[2, 2])))) if self.R is not None else 0.0
        fila = [time.time() - self.t0, self.fase, getattr(self, "i_esc", -1), self.nav,
                *self.consigna.tolist(), *self.pos.tolist(), inclinacion]
        if self.nn is not None:
            fila += list(self.nn.observation) + list(self.nn.network_output) + [self.nn.inference_time]
        self.filas.append(fila)

    def terminar(self):
        with open(self.salida, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "fase", "escalon", "nav_state", "xd", "yd", "zd", "x", "y", "z", "inclinacion_deg"]
                       + [f"obs{i}" for i in range(15)] + [f"motor{i}" for i in range(4)] + ["inferencia_us"])
            w.writerows(self.filas)
        F = [f for f in self.filas if f[1] == "escalones"]
        if F:
            e = np.array([np.linalg.norm(np.array(f[7:10]) - np.array(f[4:7])) for f in F])
            esc = np.array([f[2] for f in F])
            for i in range(len(ESCALONES)):
                ei = e[esc == i]
                if len(ei):
                    n = len(ei)
                    self.get_logger().info(f"escalón {i}: error final {ei[-n//4:].mean():.3f} m, máximo {ei.max():.2f} m")
            navs = sorted({f[3] for f in F})
            self.get_logger().info(f"RMSE en escalones {np.sqrt(np.mean(e**2)):.3f} m; nav_state durante la prueba: {navs}; "
                                   f"inclinación máxima {max(f[10] for f in F):.1f}°")
        self.get_logger().info(f"registro en {self.salida}")
        self.timer.cancel()
        raise SystemExit


def main():
    rclpy.init(); nodo = PruebaNeural()
    try:
        rclpy.spin(nodo)
    except (SystemExit, KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
