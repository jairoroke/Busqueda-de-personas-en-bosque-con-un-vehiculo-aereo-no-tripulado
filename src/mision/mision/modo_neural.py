#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
modo_neural.py -- Activa el modo de vuelo "Neural Control" del módulo
mc_nn_control de PX4 desde ROS 2 (Sección 4.8). El módulo registra su modo al
arrancar y PX4 le asigna un identificador (mode_id, normalmente 23 =
NAVIGATION_STATE_EXTERNAL1); este nodo lee ese identificador en
/fmu/out/register_ext_component_reply y, pasados unos segundos, cambia de modo
con el comando VEHICLE_CMD_SET_NAV_STATE.

Ejecución: ros2 run mision modo_neural.py --ros-args -p espera_s:=12.0

Para las corridas automáticas mision_node hace el cambio él mismo
(controlador:=red) al llegar a la altura de trabajo; este nodo sirve para
activarlo a mano. Diferencia con el Código 7.4 del diseño original: en PX4 1.17 el
estado del vehículo se publica en /fmu/out/vehicle_status_v1 (VehicleStatus es
un mensaje versionado), no en /fmu/out/vehicle_status.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from px4_msgs.msg import RegisterExtComponentReply, VehicleCommand, VehicleStatus

from mision.px4_util import QOS_PX4, comando


class ModoNeural(Node):
    def __init__(self):
        super().__init__("modo_neural")
        self.declare_parameter("espera_s", 12.0)
        self.mode_id = None
        self.activado = False
        self.create_subscription(RegisterExtComponentReply, "/fmu/out/register_ext_component_reply", self.cb_reply, QOS_PX4)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.cb_status, QOS_PX4)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.timer = self.create_timer(self.get_parameter("espera_s").value, self.activar)

    def cb_reply(self, msg):
        nombre = bytes(msg.name).split(b"\x00")[0].decode(errors="ignore")
        if msg.success and nombre.startswith("Neural"):
            self.mode_id = int(msg.mode_id)
            self.get_logger().info(f"modo Neural Control registrado con mode_id = {self.mode_id}")

    def cb_status(self, msg):
        if self.activado and msg.nav_state == (self.mode_id or VehicleStatus.NAVIGATION_STATE_EXTERNAL1):
            self.get_logger().info("Neural Control activo"); self.timer.cancel(); self.activado = False

    def activar(self):
        mode_id = self.mode_id if self.mode_id is not None else VehicleStatus.NAVIGATION_STATE_EXTERNAL1
        comando(self, self.pub_cmd, VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, float(mode_id))
        self.activado = True
        self.get_logger().info(f"solicitado nav_state = {mode_id}")


def main():
    rclpy.init(); nodo = ModoNeural()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
