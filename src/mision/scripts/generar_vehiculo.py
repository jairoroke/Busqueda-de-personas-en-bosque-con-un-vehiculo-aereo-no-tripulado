#!/usr/bin/env python3
"""Genera el vehículo de la simulación, x500_busqueda, a partir de los modelos de PX4.

Es el x500_mono_cam_down de PX4 (mismo cuerpo, motores, sensores y cámara de
1280x960 con campo horizontal de 1.74 rad, apuntando hacia abajo) con dos
diferencias:
  - el enlace base_link tiene <enable_wind>true</enable_wind>, para que el
    sistema WindEffects del mundo empuje al vehículo (Sección 4.4.2);
  - la cámara va 25 cm DEBAJO del centro (bajo los patines). En el modelo de PX4
    está 10 cm arriba y mira hacia abajo a través del cuerpo: el armazón, los
    motores y el tren de aterrizaje tapan más de la mitad de la imagen;
  - un LiDAR 3D (360 grados x 120 haces, 16 canales de -35 a +15 grados, 30 m,
    10 Hz) en base_link, para la evasión de obstáculos de mision_node.
Gazebo no
permite cambiar un elemento de un enlace incluido con <include merge>, así que
el modelo se arma copiando el texto de x500_base y agregando los complementos
de los motores de x500 y la cámara de x500_mono_cam_down.

Escribe models/x500_busqueda/ en el paquete; scripts/instalar_px4.sh lo copia a
PX4-Autopilot/Tools/simulation/gz/models, que es donde PX4 busca el vehículo.

También escribe models/x500_busqueda_ardupilot/, el mismo vehículo pilotado por
ArduPilot SITL (piloto:=ardupilot en el launch): ArduPilotPlugin (ardupilot_gazebo)
convierte cada salida de motor de 1000-2000 us en 0-1000 rad/s, el sistema
SelectorMotores (paquete mision_gz_plugins) deja pasar esos comandos o los de la
red neuronal (controlador_red) y OdometryPublisher da el estado a 400 Hz para la red.

Uso: python3 scripts/generar_vehiculo.py [~/PX4-Autopilot]
"""
import os
import re
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NOMBRE = 'x500_busqueda'
Z_CAMARA = -0.25        # m, debajo de los patines (mision/geometria.py usa el mismo valor)


LIDAR = """  <sensor name="lidar" type="gpu_lidar">
        <pose>0 0 0.1 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <topic>x500/lidar</topic>
        <lidar>
          <scan>
            <horizontal><samples>120</samples><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
            <vertical><samples>16</samples><min_angle>-0.6109</min_angle><max_angle>0.2618</max_angle></vertical>
          </scan>
          <range><min>0.8</min><max>30</max><resolution>0.05</resolution></range>
        </lidar>
      </sensor>
    """


def main(px4):
    modelos = os.path.join(px4, 'Tools', 'simulation', 'gz', 'models')
    base = open(os.path.join(modelos, 'x500_base', 'model.sdf')).read()
    x500 = open(os.path.join(modelos, 'x500', 'model.sdf')).read()
    cam = open(os.path.join(modelos, 'x500_mono_cam_down', 'model.sdf')).read()

    motores = re.findall(r'\s*<plugin filename="gz-sim-multicopter-motor-model-system".*?</plugin>', x500, re.S)
    if len(motores) != 4:
        raise SystemExit(f'Se esperaban 4 motores en x500/model.sdf, hay {len(motores)}')
    camara = re.search(r'\s*<include merge=\'true\'>\s*<uri>model://mono_cam</uri>.*?</joint>', cam, re.S)
    if camara is None:
        raise SystemExit('No se encontró la cámara en x500_mono_cam_down/model.sdf')

    bloque = camara.group(0).replace('<pose>0 0 .10 0 1.5707 0</pose>', f'<pose>0 0 {Z_CAMARA} 0 1.5707 0</pose>')
    if bloque == camara.group(0):
        raise SystemExit('Cambió la pose de la cámara en x500_mono_cam_down; revisar generar_vehiculo.py')
    sdf = base.replace("<model name='x500_base'>", f"<model name='{NOMBRE}'>", 1)
    sdf, n = re.subn(r'(<link name="base_link">)', r'\1\n      <enable_wind>true</enable_wind>', sdf, count=1)
    if n != 1:
        raise SystemExit('No se encontró base_link en x500_base/model.sdf')
    sdf = sdf.replace('  </model>', ''.join(motores) + bloque + '\n  </model>', 1)
    i = sdf.index('<link name="base_link">'); j = sdf.index('</link>', i)
    sdf = sdf[:j] + LIDAR + sdf[j:]

    destino = os.path.join(RAIZ, 'models', NOMBRE)
    os.makedirs(destino, exist_ok=True)
    with open(os.path.join(destino, 'model.sdf'), 'w') as f:
        f.write(sdf)
    with open(os.path.join(destino, 'model.config'), 'w') as f:
        f.write(f'<?xml version="1.0"?>\n<model>\n  <name>{NOMBRE}</name>\n  <version>1.0</version>\n'
                f'  <sdf version="1.9">model.sdf</sdf>\n  <description>x500 de PX4 con cámara hacia abajo '
                f'y viento habilitado (generado por generar_vehiculo.py)</description>\n</model>\n')
    print(f'{destino}/model.sdf: 4 motores, cámara hacia abajo a z = {Z_CAMARA} m, enable_wind en base_link')
    escribir_ardupilot(sdf)


# ArduPilot, quad "X": M1 adelante-derecha (ccw), M2 atrás-izquierda (ccw),
# M3 adelante-izquierda (cw), M4 atrás-derecha (cw): el mismo orden que los
# rotor_0..3 del x500 de PX4, así que la salida i mueve el rotor i.
W_MAX = 1000.0
NOMBRE_EN_MUNDO = 'x500_busqueda_0'      # como lo nombra PX4, para que los tópicos de la cámara coincidan


def escribir_ardupilot(sdf):
    nombre = NOMBRE + '_ardupilot'
    controles = ''.join(f"""
      <control channel="{i}">
        <jointName>rotor_{i}_joint</jointName>
        <type>COMMAND</type>
        <cmd_topic>/x500/ap/motor_{i}</cmd_topic>
        <useForce>0</useForce>
        <multiplier>{W_MAX}</multiplier>
        <offset>0</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
      </control>""" for i in range(4))
    entradas = ''.join(f"\n      <entrada>/x500/ap/motor_{i}</entrada>" for i in range(4))
    extra = f"""
    <plugin filename="ArduPilotPlugin" name="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <no_time_sync>1</no_time_sync>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>base_link::imu_ardupilot</imuName>{controles}
    </plugin>
    <plugin filename="SelectorMotores" name="mision::SelectorMotores">{entradas}
      <red>/x500/red/motor_speed</red>
      <modo>/x500/controlador</modo>
      <salida>/{NOMBRE_EN_MUNDO}/command/motor_speed</salida>
      <tiempo_max>0.05</tiempo_max>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_publish_frequency>400</odom_publish_frequency>
      <odom_topic>/x500/odometria</odom_topic>
      <dimensions>3</dimensions>
    </plugin>
  </model>"""
    imu = """<link name="base_link">
      <!-- IMU que lee ArduPilot (marco NED: girada 180 grados en x). -->
      <sensor name="imu_ardupilot" type="imu">
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on>
        <update_rate>1000</update_rate>
      </sensor>"""
    s = sdf.replace(f"<model name='{NOMBRE}'>", f"<model name='{nombre}'>", 1)
    s = s.replace('<link name="base_link">', imu, 1)
    i = s.rindex('  </model>')
    s = s[:i] + extra.lstrip('\n') + s[i + len('  </model>'):]
    destino = os.path.join(RAIZ, 'models', nombre)
    os.makedirs(destino, exist_ok=True)
    with open(os.path.join(destino, 'model.sdf'), 'w') as f:
        f.write(s)
    with open(os.path.join(destino, 'model.config'), 'w') as f:
        f.write(f'<?xml version="1.0"?>\n<model>\n  <name>{nombre}</name>\n  <version>1.0</version>\n'
                f'  <sdf version="1.9">model.sdf</sdf>\n  <description>x500_busqueda pilotado por ArduPilot SITL '
                f'(generado por generar_vehiculo.py)</description>\n</model>\n')
    print(f'{destino}/model.sdf: ArduPilotPlugin, SelectorMotores y odometría a 400 Hz')


if __name__ == '__main__':
    main(os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/PX4-Autopilot'))
