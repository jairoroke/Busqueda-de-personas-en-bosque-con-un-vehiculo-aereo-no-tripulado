#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detector_node.py -- Nodo de ROS 2 que ejecuta el detector de personas (YOLO,
biblioteca ultralytics) sobre la imagen de la cámara y publica las cajas.

Suscripciones
  /camera                sensor_msgs/Image  imagen de Gazebo (ros_gz_bridge)
Publicaciones
  /detecciones           vision_msgs/Detection2DArray  cajas, confianza, misma marca de tiempo que la imagen
  /detector/latencia_ms  std_msgs/Float32              tiempo de inferencia por cuadro
  /camera_yolo           sensor_msgs/Image             imagen con las cajas (panel de Gazebo o rqt_image_view)

Ejecución (con el Python de ~/.venvs/yolo, que tiene ultralytics y ve ROS 2):
  ~/.venvs/yolo/bin/python -m mision.detector_node --ros-args -p pesos:=<paquete>/yolo/personas_yolo26s.pt \\
      -p umbral:=0.25 -p latencia_extra_ms:=0.0 -p saltar_cuadros:=0
Los parámetros latencia_extra_ms y saltar_cuadros permiten degradar artificialmente la
latencia y la tasa de cuadros para la pregunta P3.

Diferencias con el Código 8.3 del diseño original: no usa cv_bridge (convierte la imagen con
numpy, porque cv_bridge de Jazzy está compilado con numpy 1.x y ultralytics puede traer
otra versión), publica la imagen anotada y acepta modelos de una sola clase "persona"
(los entrenados con scripts/entrenar_detector.py) o los de COCO (clase 0 = person).
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from ultralytics import YOLO
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


class Detector(Node):
    def __init__(self):
        super().__init__("detector")
        self.declare_parameter("pesos", "yolo_personas.pt")
        self.declare_parameter("umbral", 0.25)
        self.declare_parameter("tam", 1280)
        self.declare_parameter("latencia_extra_ms", 0.0)
        self.declare_parameter("saltar_cuadros", 0)
        self.declare_parameter("dispositivo", "0")
        self.declare_parameter("anotar", True)
        self.modelo = YOLO(self.get_parameter("pesos").value)
        self.umbral = self.get_parameter("umbral").value
        self.tam = self.get_parameter("tam").value
        self.extra = self.get_parameter("latencia_extra_ms").value / 1000.0
        self.saltar = self.get_parameter("saltar_cuadros").value
        self.disp = self.get_parameter("dispositivo").value
        self.anotar = self.get_parameter("anotar").value
        nombres = self.modelo.names
        self.clases = [k for k, v in nombres.items() if v in ("persona", "person")] or [0]
        self.n = 0
        self.create_subscription(Image, "/camera", self.cb_imagen, 1)
        self.pub = self.create_publisher(Detection2DArray, "/detecciones", 10)
        self.pub_lat = self.create_publisher(Float32, "/detector/latencia_ms", 10)
        self.pub_img = self.create_publisher(Image, "/camera_yolo", 2)
        self.modelo.predict(np.zeros((960, 1280, 3), np.uint8), imgsz=self.tam, device=self.disp, verbose=False)
        self.get_logger().info(f"detector listo: clases {[nombres[k] for k in self.clases]}, imgsz {self.tam}")

    def cb_imagen(self, msg):
        self.n += 1
        if self.saltar and (self.n % (self.saltar + 1)) != 0:
            return                                            # degradación de la tasa de cuadros
        t0 = time.perf_counter()
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[..., :3]
        bgr = img[..., ::-1] if msg.encoding.lower().startswith("rgb") else img
        res = self.modelo.predict(bgr, imgsz=self.tam, conf=self.umbral, classes=self.clases,
                                  device=self.disp, verbose=False)[0]
        if self.extra > 0:
            time.sleep(self.extra)                            # degradación de la latencia
        salida = Detection2DArray(); salida.header = msg.header
        cajas = res.boxes.xywh.cpu().numpy(); confs = res.boxes.conf.cpu().numpy()
        for caja, conf in zip(cajas, confs):
            d = Detection2D(); d.header = msg.header
            d.bbox.center.position.x, d.bbox.center.position.y = float(caja[0]), float(caja[1])
            d.bbox.size_x, d.bbox.size_y = float(caja[2]), float(caja[3])
            h = ObjectHypothesisWithPose(); h.hypothesis.class_id = "persona"; h.hypothesis.score = float(conf)
            d.results.append(h); salida.detections.append(d)
        self.pub.publish(salida)
        ms = 1000.0 * (time.perf_counter() - t0)
        lat = Float32(); lat.data = ms; self.pub_lat.publish(lat)
        if self.anotar and self.pub_img.get_subscription_count() > 0:
            self.publicar_anotada(msg, bgr, cajas, confs, ms)

    def publicar_anotada(self, msg, bgr, cajas, confs, ms):
        dibujo = np.ascontiguousarray(bgr[..., ::-1])         # RGB
        for (cx, cy, w, h), conf in zip(cajas, confs):
            x0, y0, x1, y1 = int(cx - w/2), int(cy - h/2), int(cx + w/2), int(cy + h/2)
            cv2.rectangle(dibujo, (x0 - 4, y0 - 4), (x1 + 4, y1 + 4), (255, 40, 40), 2)
            cv2.putText(dibujo, f"persona {conf:.2f}", (x0, max(16, y0 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 40, 40), 2)
        cv2.putText(dibujo, f"YOLO {ms:.0f} ms, {len(confs)} personas", (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (255, 255, 255), 2)
        out = Image(header=msg.header, height=msg.height, width=msg.width, encoding="rgb8",
                    step=msg.width * 3, data=dibujo.tobytes())
        self.pub_img.publish(out)


def main():
    rclpy.init(); nodo = Detector()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C a media inferencia: el publish falla con el contexto ya cerrado
        if rclpy.ok():
            raise
    finally:
        nodo.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
