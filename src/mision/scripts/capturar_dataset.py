#!/usr/bin/env python3
"""Genera un dataset de detección de personas (formato YOLO) desde Gazebo.

Para cada configuración (nivel de oclusión, iluminación y semilla) genera un
bosque con generar_mundo.py y 30 personas, le agrega una cámara con la óptica de
la del dron (1280x960, campo horizontal de 1.74 rad, hacia abajo) y una cámara de
cajas (boundingbox_camera) con la misma óptica, arranca su propio Gazebo sin
interfaz (partición aislada) y repite: mueve la cámara a una pose aleatoria de
dron buscando (12 a 45 m de altura, inclinación de unos grados, cualquier
guiñada), toma la imagen y escribe las cajas de las personas.

La cámara de cajas da la caja de la parte VISIBLE de cada objeto con etiqueta (las
personas tienen la etiqueta 1 del sistema Label, ver importar_personas.py), así
que una persona tapada por una copa aparece recortada o no aparece.

Uso (con el Python de ROS, que trae gz.transport13 y gz.msgs10):
    python3 scripts/capturar_dataset.py ~/datasets/personas_bosque 3000
    python3 scripts/capturar_dataset.py ~/datasets/personas_bosque 3000 --por-mundo 250 --semilla 7
Después: ~/.venvs/yolo/bin/python scripts/entrenar_detector.py entrenar --datos ~/datasets/personas_bosque/datos.yaml
"""
import argparse
import math
import os
import random
import subprocess
import sys
import threading
import time

import numpy as np
from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node
from PIL import Image

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RAIZ, 'scripts'))
import generar_mundo  # noqa: E402

ANCHO, ALTO, HFOV = 1280, 960, 1.74
ETIQUETA_PERSONA = 1

CAMARA_SDF = f"""
    <model name="camara_datos">
      <pose>0 0 30 0 1.5707 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>1</mass></inertial>
        <sensor name="rgb" type="camera">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <topic>/datos/camara</topic>
          <camera>
            <horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{ANCHO}</width><height>{ALTO}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>300</far></clip>
          </camera>
        </sensor>
        <sensor name="cajas" type="boundingbox_camera">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <topic>/datos/cajas</topic>
          <camera>
            <horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{ANCHO}</width><height>{ALTO}</height></image>
            <clip><near>0.1</near><far>300</far></clip>
            <box_type>2d</box_type>
          </camera>
        </sensor>
      </link>
    </model>
"""


def cuaternion(R):
    """(w, x, y, z) de una matriz de rotación."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return w, x, y, z


def rot(yaw, pitch, roll):
    cy, sy, cp, sp, cr, sr = (math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch),
                              math.cos(roll), math.sin(roll))
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


class Capturador:
    def __init__(self, mundo):
        self.mundo = mundo
        self.nodo = Node()
        self.cv = threading.Condition()
        self.img = None; self.t_img = -1.0
        self.cajas = {}                                       # marca de tiempo -> lista de cajas
        self.nodo.subscribe(GzImage, '/datos/camara', self._img)
        self.nodo.subscribe(AnnotatedAxisAligned2DBox_V, '/datos/cajas', self._cajas)

    @staticmethod
    def _t(header):
        return header.stamp.sec + header.stamp.nsec * 1e-9

    def _img(self, msg):
        with self.cv:
            self.img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
            self.t_img = self._t(msg.header)
            self.cv.notify_all()

    def _cajas(self, msg):
        cajas = [(b.box.min_corner.x, b.box.min_corner.y, b.box.max_corner.x, b.box.max_corner.y)
                 for b in msg.annotated_box if b.label == ETIQUETA_PERSONA]
        with self.cv:
            self.cajas[round(self._t(msg.header), 3)] = cajas
            if len(self.cajas) > 50:
                for k in sorted(self.cajas)[:-50]:
                    del self.cajas[k]
            self.cv.notify_all()

    def mover(self, x, y, z, R):
        p = Pose(); p.name = 'camara_datos'
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = cuaternion(R)
        for _ in range(20):                                   # el servicio tarda en aparecer al arrancar
            ok, rep = self.nodo.request(f'/world/{self.mundo}/set_pose', p, Pose, Boolean, 2000)
            if ok and rep.data:
                return True
            time.sleep(0.5)
        return False

    def foto(self, despues_de, espera=5.0):
        """Imagen y cajas con la misma marca de tiempo, tomadas después de `despues_de` (tiempo de simulación)."""
        fin = time.time() + espera
        with self.cv:
            while time.time() < fin:
                if self.t_img > despues_de and round(self.t_img, 3) in self.cajas:
                    return self.img, self.cajas[round(self.t_img, 3)], self.t_img
                self.cv.wait(0.2)
        return None, None, None


def lanzar_gazebo(sdf, particion):
    env = dict(os.environ, GZ_PARTITION=particion, GZ_IP='127.0.0.1',
               GZ_SIM_RESOURCE_PATH=os.path.join(RAIZ, 'models'))
    env.pop('GZ_SIM_SERVER_CONFIG_PATH', None)
    return subprocess.Popen(['gz', 'sim', '-s', '-r', sdf], env=env, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)


def capturar(salida, total, por_mundo, semilla):
    rnd = random.Random(semilla)
    for p in ('images/train', 'images/val', 'labels/train', 'labels/val', 'mundos'):
        os.makedirs(os.path.join(salida, p), exist_ok=True)
    conteo = {'imagenes': 0, 'personas': 0, 'vacias': 0}
    i = len(os.listdir(os.path.join(salida, 'images', 'train'))) + len(os.listdir(os.path.join(salida, 'images', 'val')))
    k = 0
    while i < total:
        oclusion = ('baja', 'media', 'alta')[k % 3]
        iluminacion = ('alta', 'baja')[(k // 3) % 2]
        nombre, sdf, verdad = generar_mundo.generar(oclusion, iluminacion, 'no', 1000 + semilla * 100 + k,
                                                    lado=120.0, n_personas=30)
        k += 1
        sdf = sdf.replace('  </world>', CAMARA_SDF + '  </world>')
        ruta = os.path.join(salida, 'mundos', nombre + '.sdf')
        with open(ruta, 'w') as f:
            f.write(sdf)
        particion = f'datos_{os.getpid()}'
        os.environ['GZ_PARTITION'] = particion
        os.environ['GZ_IP'] = '127.0.0.1'
        gz = lanzar_gazebo(ruta, particion)
        try:
            cap = Capturador(nombre)
            if not cap.mover(0, 0, 30, rot(0, 0, 0) @ rot(0, math.pi/2, 0)):
                print(f'{nombre}: Gazebo no responde, se omite', flush=True); continue
            n_mundo = 0
            personas = verdad['personas']
            while n_mundo < por_mundo and i < total:
                h = math.exp(rnd.uniform(math.log(12), math.log(45)))
                if rnd.random() < 0.75:                       # cerca de una persona (dentro de la huella)
                    p = rnd.choice(personas)
                    m = 0.8 * h * math.tan(HFOV / 2)
                    x, y = p['x'] + rnd.uniform(-m, m), p['y'] + rnd.uniform(-0.75 * m, 0.75 * m)
                else:
                    x, y = rnd.uniform(-60, 60), rnd.uniform(-60, 60)
                yaw = rnd.uniform(-math.pi, math.pi)
                Rb = rot(yaw, rnd.gauss(0, 0.06), rnd.gauss(0, 0.06))
                t0 = cap.t_img
                if not cap.mover(x, y, h, Rb @ rot(0, math.pi/2, 0)):
                    continue
                img, cajas, _ = cap.foto(t0 + 0.25)
                if img is None:
                    continue
                filas = []
                for x0, y0, x1, y1 in cajas:
                    x0, x1 = np.clip([x0, x1], 0, ANCHO - 1); y0, y1 = np.clip([y0, y1], 0, ALTO - 1)
                    if x1 - x0 < 3 or y1 - y0 < 3:
                        continue
                    filas.append(f'0 {(x0 + x1) / 2 / ANCHO:.6f} {(y0 + y1) / 2 / ALTO:.6f} '
                                 f'{(x1 - x0) / ANCHO:.6f} {(y1 - y0) / ALTO:.6f}')
                parte = 'val' if i % 10 == 0 else 'train'
                base = f'{i:05d}'
                Image.fromarray(img).save(os.path.join(salida, 'images', parte, base + '.jpg'), quality=93)
                with open(os.path.join(salida, 'labels', parte, base + '.txt'), 'w') as f:
                    f.write('\n'.join(filas) + ('\n' if filas else ''))
                with open(os.path.join(salida, 'estratos.csv'), 'a') as f:
                    f.write(f'{base}.jpg,{parte},{oclusion},{iluminacion},{h:.1f},{len(filas)}\n')
                conteo['imagenes'] += 1; conteo['personas'] += len(filas); conteo['vacias'] += not filas
                i += 1; n_mundo += 1
                if i % 100 == 0:
                    print(f'{i}/{total} imágenes; {conteo}', flush=True)
        finally:
            os.killpg(gz.pid, 15)
            try:
                gz.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(gz.pid, 9)
    with open(os.path.join(salida, 'datos.yaml'), 'w') as f:
        f.write(f'path: {os.path.abspath(salida)}\ntrain: images/train\nval: images/val\nnames:\n  0: persona\n')
    print(f'Listo: {conteo} en {salida}')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('salida'); ap.add_argument('total', type=int)
    ap.add_argument('--por-mundo', type=int, default=250)
    ap.add_argument('--semilla', type=int, default=7)
    a = ap.parse_args()
    capturar(os.path.expanduser(a.salida), a.total, a.por_mundo, a.semilla)
