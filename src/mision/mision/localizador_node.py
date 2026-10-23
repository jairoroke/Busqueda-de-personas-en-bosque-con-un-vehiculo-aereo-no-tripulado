#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localizador_node.py -- Nodo de ROS 2 que convierte cada detección en una
posición sobre el terreno con su covarianza (Sección 4.11) y mantiene el
conjunto de personas candidatas por asociación de Mahalanobis y fusión.

Suscripciones
  /fmu/out/vehicle_odometry  px4_msgs/VehicleOdometry      estado estimado (NED/FRD)
  /camera_info               sensor_msgs/CameraInfo        matriz K
  /detecciones               vision_msgs/Detection2DArray  cajas y confianzas
  /clock                     rosgraph_msgs/Clock           (use_sim_time) reloj de Gazebo
Publicaciones
  /personas/estimaciones     vision_msgs/Detection3DArray  una estimación por detección
  /personas/candidatas       vision_msgs/Detection3DArray  candidatas fusionadas (score = confianza acumulada)

Ejecución: ros2 run mision localizador_node --ros-args -p use_sim_time:=true -p z_terreno:=0.0 -p sigma_px:=5.0

Diferencias con el Código 8.4 del diseño original: la cámara de x500_busqueda está 25 cm
debajo del centro de masa, bajo los patines (T_BC en mision/geometria.py); el
desfase entre el reloj de PX4 y el de la imagen se estima con el mínimo de
(llegada - marca de PX4) en tiempo de simulación, no con el primer mensaje en
tiempo de pared, para que no se desfase si Gazebo corre más lento que el tiempo
real; y el punto de apoyo es el centro de la caja, porque con la cámara hacia
abajo la persona se ve casi desde arriba.

Además se descartan las detecciones cuyo rayo forma más de `angulo_max_deg` (60
grados) con la vertical: cuando el vehículo se inclina mucho (vueltas, ráfagas),
la cámara alcanza a ver cerca del horizonte y un rayo casi horizontal se corta con
el suelo a cientos de metros con un error enorme. Con el vehículo nivelado las
esquinas de la imagen quedan a ~57 grados, así que en vuelo normal no se pierde nada.
"""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D, ObjectHypothesisWithPose

from mision.geometria import covarianza, euler_de_R, localizar
from mision.px4_util import QOS_PX4, ned_a_enu, R_enu_de_odometria


class Localizador(Node):
    def __init__(self):
        super().__init__("localizador")
        self.declare_parameter("z_terreno", 0.0)
        self.declare_parameter("sigma_px", 5.0)
        self.declare_parameter("umbral_chi2", 9.21)          # chi^2 (2 g.l.) al 99 %
        self.declare_parameter("conf_min", 0.05)
        self.declare_parameter("angulo_max_deg", 60.0)        # rayos más tendidos se descartan
        self.z_t = self.get_parameter("z_terreno").value
        self.sigma_px = self.get_parameter("sigma_px").value
        self.chi2 = self.get_parameter("umbral_chi2").value
        self.conf_min = self.get_parameter("conf_min").value
        self.tan_max = np.tan(np.radians(self.get_parameter("angulo_max_deg").value))
        self.K_inv = None
        self.buffer = []                                      # (t_sim, xi_enu, eta, Sp, Se)
        self.offset = None                                    # t_sim - t_px4
        self.candidatas = []                                  # dict(c, Sigma, conf, n)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.cb_odom, QOS_PX4)
        self.create_subscription(CameraInfo, "/camera_info", self.cb_info, 10)
        self.create_subscription(Detection2DArray, "/detecciones", self.cb_det, 10)
        self.pub_est = self.create_publisher(Detection3DArray, "/personas/estimaciones", 10)
        self.pub_cand = self.create_publisher(Detection3DArray, "/personas/candidatas", 10)
        self.get_logger().info("localizador listo")

    def cb_info(self, msg):
        if self.K_inv is None:
            K = np.array(msg.k).reshape(3, 3)
            self.K_inv = np.linalg.inv(K)
            self.get_logger().info(f"cámara {msg.width}x{msg.height}, f = {K[0, 0]:.1f} px")

    def cb_odom(self, msg):
        t_px4 = msg.timestamp_sample * 1e-6
        t_rec = self.get_clock().now().nanoseconds * 1e-9
        if t_rec <= 0.0:
            return                                            # todavía no llega /clock
        d = t_rec - t_px4
        self.offset = d if self.offset is None else min(self.offset, d)
        xi = ned_a_enu(msg.position)
        eta = euler_de_R(R_enu_de_odometria(msg))
        var_p = np.nan_to_num(np.array(msg.position_variance, float), nan=1.0)
        var_o = np.nan_to_num(np.array(msg.orientation_variance, float), nan=1e-4)
        Sp = np.diag([var_p[1], var_p[0], var_p[2]])         # reordenada a ENU
        Se = np.diag([var_o[2], var_o[1], var_o[0]])         # (psi, theta, phi)
        self.buffer.append((t_px4, xi, eta, Sp, Se))
        self.buffer = self.buffer[-500:]

    def pose_en(self, t):
        """Interpola la pose del buffer en el instante t (posición lineal, orientación más cercana)."""
        if len(self.buffer) < 2 or self.offset is None:
            return None
        t = t - self.offset                                   # al reloj de PX4
        ts = np.array([b[0] for b in self.buffer])
        if t < ts[0] - 0.05 or t > ts[-1] + 0.05:
            return None
        k = int(np.searchsorted(ts, t))
        if k == 0 or k >= len(ts):
            b = self.buffer[min(max(k, 0), len(ts)-1)]
            return b[1], b[2], b[3], b[4]
        a, b = self.buffer[k-1], self.buffer[k]
        w = (t - a[0]) / max(b[0] - a[0], 1e-6)
        xi = (1-w)*a[1] + w*b[1]
        cerca = a if w < 0.5 else b
        return xi, cerca[2], cerca[3], cerca[4]

    def cb_det(self, msg):
        if self.K_inv is None:
            return
        t_img = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.pose_en(t_img)
        if pose is None:
            return
        xi, eta, Sp, Se = pose
        salida = Detection3DArray(); salida.header = msg.header; salida.header.frame_id = "map"
        for det in msg.detections:
            conf = det.results[0].hypothesis.score if det.results else 0.0
            if conf < self.conf_min:
                continue
            # Con la cámara hacia abajo se ve a la persona casi desde arriba: el centro de la
            # caja es el mejor punto de apoyo (el borde inferior del diseño original es para cámaras oblicuas).
            uv = np.array([det.bbox.center.position.x, det.bbox.center.position.y])
            x = localizar(uv, xi, eta, self.K_inv, self.z_t)
            if x is None:
                continue
            altura = xi[2] - self.z_t
            if altura <= 0 or np.hypot(x[0] - xi[0], x[1] - xi[1]) > altura*self.tan_max:
                continue                                      # rayo demasiado tendido
            Su = np.eye(2) * (self.sigma_px**2)
            S, _ = covarianza(uv, xi, eta, self.K_inv, self.z_t, Sp, Se, Su)
            if not np.all(np.isfinite(S)):
                continue
            salida.detections.append(self.a_detection3d(x, S, conf))
            self.asociar(x[:2], S, conf)
        self.pub_est.publish(salida)
        self.publicar_candidatas(msg.header)

    def asociar(self, x, S, conf):
        """Asociación por distancia de Mahalanobis y fusión de información (Sección 4.11.4)."""
        mejor, dmin = None, np.inf
        for c in self.candidatas:
            d = x - c["c"]; Sc = S + c["Sigma"]
            m = float(d @ np.linalg.solve(Sc, d))
            if m < self.chi2 and m < dmin:
                mejor, dmin = c, m
        if mejor is None:
            self.candidatas.append({"c": x.copy(), "Sigma": S.copy(), "conf": conf, "n": 1})
            return
        Ic = np.linalg.inv(mejor["Sigma"]) + np.linalg.inv(S)
        mejor["c"] = np.linalg.solve(Ic, np.linalg.solve(mejor["Sigma"], mejor["c"]) + np.linalg.solve(S, x))
        mejor["Sigma"] = np.linalg.inv(Ic)
        mejor["conf"] = 1.0 - (1.0 - mejor["conf"]) * (1.0 - conf)    # combinación de confianzas
        mejor["n"] += 1

    def a_detection3d(self, x, S, conf, n=1):
        d = Detection3D()
        h = ObjectHypothesisWithPose(); h.hypothesis.class_id = "persona"; h.hypothesis.score = float(conf)
        h.pose.pose.position.x, h.pose.pose.position.y, h.pose.pose.position.z = map(float, x)
        cov = np.zeros((6, 6)); cov[:2, :2] = S
        h.pose.covariance = cov.ravel().tolist()
        d.results.append(h)
        d.id = str(n)                                         # número de detecciones fusionadas
        return d

    def publicar_candidatas(self, header):
        msg = Detection3DArray(); msg.header = header; msg.header.frame_id = "map"
        for c in self.candidatas:
            x = np.array([c["c"][0], c["c"][1], self.z_t])
            msg.detections.append(self.a_detection3d(x, c["Sigma"], c["conf"], c["n"]))
        self.pub_cand.publish(msg)


def main():
    rclpy.init(); nodo = Localizador()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
