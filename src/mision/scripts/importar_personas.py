#!/usr/bin/env python3
"""Copia los modelos de personas de Gazebo Fuel al paquete para usarlos sin conexión.

Toma los modelos de OpenRobotics que ya están en la caché de Fuel
(~/.gz/fuel; se descargan con `gz fuel download -u <url>`), los copia a
models/<nombre>, cambia las URI absolutas por rutas relativas, los vuelve
estáticos, quita las colisiones de malla (no hacen falta y pesan) y les agrega
el sistema Label con la etiqueta 1. Esa etiqueta es la que usa la cámara de
cajas (boundingbox_camera) de scripts/capturar_dataset.py para etiquetar las
personas con su parte visible.

Uso: python3 scripts/importar_personas.py
"""
import glob
import os
import re
import shutil

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FUEL = os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org/openrobotics/models')
ETIQUETA_PERSONA = 1

# nombre local: (modelo de Fuel, malla, pose de la malla, URL)
MODELOS = {
    'persona_de_pie': ('standing person', 'standing.dae', '0 0 0.02 0.04 0 0'),
    'persona_caminando': ('walking person', 'walking.dae', '0 0 -0.02 0 0 0'),
    'persona_mujer': ('casual female', 'casual_female.dae', '0 0 0.02 0.04 0 0'),
    'persona_sentada': ('rescue randy sitting', 'rescue_randy.dae', '0 0 -0.623996 0 0 0'),
}

SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{nombre}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <pose>{pose}</pose>
        <geometry>
          <mesh>
            <uri>meshes/{malla}</uri>
          </mesh>
        </geometry>
        <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
          <label>{etiqueta}</label>
        </plugin>
      </visual>
    </link>
  </model>
</sdf>
"""


def main():
    for nombre, (fuel, malla, pose) in MODELOS.items():
        versiones = sorted(glob.glob(os.path.join(FUEL, fuel, '*')), key=lambda p: int(os.path.basename(p)))
        if not versiones:
            raise SystemExit(f'No está en la caché: gz fuel download -u '
                             f'"https://fuel.gazebosim.org/1.0/OpenRobotics/models/{fuel}"')
        origen = versiones[-1]
        destino = os.path.join(RAIZ, 'models', nombre)
        shutil.rmtree(destino, ignore_errors=True)
        shutil.copytree(os.path.join(origen, 'meshes'), os.path.join(destino, 'meshes'))
        if os.path.isdir(os.path.join(origen, 'materials')):
            shutil.copytree(os.path.join(origen, 'materials'), os.path.join(destino, 'materials'))
        # Las mallas a veces citan sus texturas con ruta absoluta de Fuel.
        for dae in glob.glob(os.path.join(destino, 'meshes', '*.dae')):
            texto = open(dae, encoding='utf-8', errors='ignore').read()
            texto = re.sub(r'https?://[^<"]*/files/', '../', texto)
            open(dae, 'w', encoding='utf-8').write(texto)
        with open(os.path.join(destino, 'model.sdf'), 'w') as f:
            f.write(SDF.format(nombre=nombre, pose=pose, malla=malla, etiqueta=ETIQUETA_PERSONA))
        with open(os.path.join(destino, 'model.config'), 'w') as f:
            f.write(f'<?xml version="1.0"?>\n<model>\n  <name>{nombre}</name>\n'
                    f'  <version>1.0</version>\n  <sdf version="1.9">model.sdf</sdf>\n</model>\n')
        with open(os.path.join(destino, 'ORIGEN.txt'), 'w') as f:
            f.write(f'Modelo "{fuel}" de OpenRobotics, descargado de Gazebo Fuel:\n'
                    f'https://fuel.gazebosim.org/1.0/OpenRobotics/models/{fuel}\n'
                    f'Licencia: Creative Commons Zero v1.0 Universal (dominio público).\n'
                    f'Copia local estática, sin colisiones y con la etiqueta {ETIQUETA_PERSONA} del sistema Label.\n')
        print(f'{nombre}: {origen}')


if __name__ == '__main__':
    main()
