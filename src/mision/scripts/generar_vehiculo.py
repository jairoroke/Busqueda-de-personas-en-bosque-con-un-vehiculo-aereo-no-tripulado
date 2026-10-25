#!/usr/bin/env python3
"""Genera los vehículos de la simulación: tarot650_busqueda (el que se usa) y x500_busqueda.

tarot650_busqueda es el Tarot Ironman 650 de la simulación de incendios
(paquete cuadrirotor_sim: la tabla VEHICULOS['tarot650'] de
scripts/generar_modelo.py y sus mallas del CAD), con lo que PX4 necesita:
  - los sensores de x500_base de PX4 (IMU, barómetro, magnetómetro y GPS con el
    ruido de PX4), con los mismos nombres, porque el puente de PX4 los busca así;
  - los motores en el orden de PX4 (rotor_0 adelante-derecha ccw, rotor_1
    atrás-izquierda ccw, rotor_2 adelante-izquierda cw, rotor_3 atrás-derecha cw),
    que es también el de las salidas 1-4 de ArduPilot en un quad "X".
x500_busqueda es el x500_mono_cam_down de PX4 (se armaba antes con él).

A los dos se les agrega:
  - <enable_wind>true</enable_wind> en base_link, para que el sistema
    WindEffects del mundo empuje al vehículo (Sección 4.4.2);
  - la cámara mono_cam de PX4 (1280x960, campo horizontal de 1.74 rad) hacia
    abajo, justo debajo de los patines (Z_CAMARA). En el x500_mono_cam_down de PX4
    va 10 cm arriba y mira a través del cuerpo: el armazón, los motores y el tren
    de aterrizaje tapan más de la mitad de la imagen;
  - un LiDAR 3D (360 grados x 120 haces, 16 canales de -35 a +15 grados, 30 m,
    10 Hz) en base_link, para la evasión de obstáculos de mision_node.
Gazebo no permite cambiar un elemento de un enlace incluido con <include merge>,
así que el modelo se arma como texto.

Escribe models/<vehiculo>_busqueda/ en el paquete; scripts/instalar_px4.sh lo copia a
PX4-Autopilot/Tools/simulation/gz/models, que es donde PX4 busca el vehículo.

También escribe models/<vehiculo>_busqueda_ardupilot/, el mismo vehículo pilotado por
ArduPilot SITL (piloto:=ardupilot en el launch): ArduPilotPlugin (ardupilot_gazebo)
convierte cada salida de motor de 1000-2000 us en 0-w_max rad/s, el sistema
SelectorMotores (paquete mision_gz_plugins) deja pasar esos comandos o los de la
red neuronal (controlador_red) y OdometryPublisher da el estado a 400 Hz para la red.
ArduPilot vuela con su EKF3: no_time_sync=0 (con 1 el backend JSON fuerza
AHRS_EKF_TYPE=10, el estado verdadero del simulador) y su IMU tiene ruido y sesgo.

Uso: python3 scripts/generar_vehiculo.py [~/PX4-Autopilot]
"""
import importlib.util
import os
import re
import shutil
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Tabla de vehículos de la simulación de incendios (misma fuente para el Tarot).
GENERADOR_TAROT = os.path.join(os.path.dirname(RAIZ), 'cuadrirotor_sim', 'scripts', 'generar_modelo.py')

# Por vehículo: altura de la cámara bajo el origen de base_link (mision/geometria.py
# la lee de aquí) y velocidad máxima de los rotores.
VEHICULOS = {
    'tarot650': dict(z_camara=-0.24, w_max=680.0),   # patines a 0.2255 m bajo el origen
    'x500': dict(z_camara=-0.25, w_max=1000.0),
}


LIDAR = """  <sensor name="lidar" type="gpu_lidar">
        <pose>0 0 0.1 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <topic>dron/lidar</topic>
        <lidar>
          <scan>
            <horizontal><samples>120</samples><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
            <vertical><samples>16</samples><min_angle>-0.6109</min_angle><max_angle>0.2618</max_angle></vertical>
          </scan>
          <range><min>0.8</min><max>30</max><resolution>0.05</resolution></range>
        </lidar>
      </sensor>
    """


def camara(z):
    return f"""
    <include merge='true'>
      <uri>model://mono_cam</uri>
      <pose>0 0 {z} 0 1.5707 0</pose>
      <name>mono_cam</name>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose relative_to="base_link">0 0 0 0 1.5707 0</pose>
    </joint>"""


def sensores_px4(base):
    """Los sensores de base_link de x500_base (barómetro, magnetómetro, IMU y GPS)."""
    s = re.search(r'\s*<sensor name="air_pressure_sensor".*<sensor name="navsat_sensor".*?</sensor>', base, re.S)
    if s is None:
        raise SystemExit('No se encontraron los sensores en x500_base/model.sdf')
    return s.group(0)


def tabla_tarot():
    spec = importlib.util.spec_from_file_location('generar_modelo_tarot', GENERADOR_TAROT)
    if spec is None or not os.path.exists(GENERADOR_TAROT):
        raise SystemExit(f'No se encontró la tabla del Tarot: {GENERADOR_TAROT}')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.VEHICULOS['tarot650']


def orden_px4(rotores):
    """Índices de los rotores del Tarot en el orden de PX4/ArduPilot (FR ccw, BL ccw, FL cw, BR cw)."""
    esquinas = [(1, -1, 'ccw'), (-1, 1, 'ccw'), (1, 1, 'cw'), (-1, -1, 'cw')]
    orden = []
    for sx, sy, d in esquinas:
        r = [j for j, (x, y, dj) in enumerate(rotores) if (x > 0) == (sx > 0) and (y > 0) == (sy > 0)]
        if len(r) != 1 or rotores[r[0]][2] != d:
            raise SystemExit(f'El Tarot no tiene un rotor {d} en la esquina ({sx}, {sy})')
        orden.append(r[0])
    return orden


def sdf_tarot(nombre, base):
    """tarot650_busqueda: cuerpo, mallas, rotores y motores del Tarot; sensores de PX4."""
    v = tabla_tarot()
    malla = f'model://{nombre}/meshes'
    ixx, iyy, izz = v['inercia_cuerpo']
    ixx_r, iyy_r, izz_r = v['inercia_rotor']
    visuales = ''.join(f"""
      <visual name="{m}">
        <geometry><mesh><uri>{malla}/{m}.stl</uri></mesh></geometry>
        <material><ambient>{c}</ambient><diffuse>{c}</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>""" for m, c in v['mallas'])
    patas = ''.join(f"""
      <collision name="pata_{k}">
        <pose>0 {y} {z} 0 1.5707963 0</pose>
        <geometry><cylinder><radius>0.005</radius><length>0.2</length></cylinder></geometry>
      </collision>""" for k, (y, z) in enumerate(v['patas']))
    rotores, juntas, motores = [], [], []
    for i, j in enumerate(orden_px4(v['rotores'])):
        x, y, d = v['rotores'][j]
        color = '0.9 0.35 0.05 1' if x > 0 else '0.1 0.1 0.1 1'   # delanteros en naranja
        rotores.append(f"""
    <link name="rotor_{i}">
      <pose>{x} {y} {v['z_rotor']} 0 0 0</pose>
      <inertial>
        <mass>{v['masa_rotor']}</mass>
        <inertia>
          <ixx>{ixx_r}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy_r}</iyy><iyz>0</iyz><izz>{izz_r}</izz>
        </inertia>
      </inertial>
      <visual name="helice">
        <geometry><mesh><uri>{malla}/helice_{j}.stl</uri></mesh></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>""")
        juntas.append(f"""
    <joint name="rotor_{i}_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_{i}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
      </axis>
    </joint>""")
        motores.append(f"""
    <plugin filename="gz-sim-multicopter-motor-model-system"
      name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_{i}_joint</jointName>
      <linkName>rotor_{i}</linkName>
      <turningDirection>{d}</turningDirection>
      <timeConstantUp>{v['tau_subida']}</timeConstantUp>
      <timeConstantDown>{v['tau_bajada']}</timeConstantDown>
      <maxRotVelocity>{v['w_max']}</maxRotVelocity>
      <motorConstant>{v['kf']}</motorConstant>
      <momentConstant>{v['km']}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{i}</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>""")
    sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generado por scripts/generar_vehiculo.py; no editar a mano.
     {v['descripcion']}: la tabla VEHICULOS['tarot650'] de cuadrirotor_sim.
     kf = {v['kf']} N/(rad/s)^2, km = {v['km']} m, w_max = {v['w_max']} rad/s. -->
<sdf version='1.9'>
  <model name='{nombre}'>
    <pose>0 0 {v['z_inicial']} 0 0 0</pose>
    <self_collide>false</self_collide>
    <static>false</static>
    <link name="base_link">
      <enable_wind>true</enable_wind>
      <inertial>
        <mass>{v['masa_cuerpo']}</mass>
        <inertia>
          <ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy}</iyy><iyz>0</iyz><izz>{izz}</izz>
        </inertia>
      </inertial>
      <gravity>true</gravity>
      <velocity_decay />
      <collision name="cuerpo_col">
        <geometry><box><size>0.16 0.16 0.06</size></box></geometry>
      </collision>{patas}{visuales}{sensores_px4(base)}
    {LIDAR}</link>{''.join(rotores)}{''.join(juntas)}{''.join(motores)}{camara(VEHICULOS['tarot650']['z_camara'])}
  </model>
</sdf>
"""
    return sdf, v


def sdf_x500(nombre, base, x500):
    """x500_busqueda: x500_base y los motores de x500 de PX4."""
    motores = re.findall(r'\s*<plugin filename="gz-sim-multicopter-motor-model-system".*?</plugin>', x500, re.S)
    if len(motores) != 4:
        raise SystemExit(f'Se esperaban 4 motores en x500/model.sdf, hay {len(motores)}')
    sdf = base.replace("<model name='x500_base'>", f"<model name='{nombre}'>", 1)
    sdf, n = re.subn(r'(<link name="base_link">)', r'\1\n      <enable_wind>true</enable_wind>', sdf, count=1)
    if n != 1:
        raise SystemExit('No se encontró base_link en x500_base/model.sdf')
    sdf = sdf.replace('  </model>', ''.join(motores) + camara(VEHICULOS['x500']['z_camara']) + '\n  </model>', 1)
    i = sdf.index('<link name="base_link">'); j = sdf.index('</link>', i)
    return sdf[:j] + LIDAR + sdf[j:]


def escribir(nombre, sdf, descripcion, mallas=None):
    destino = os.path.join(RAIZ, 'models', nombre)
    os.makedirs(destino, exist_ok=True)
    with open(os.path.join(destino, 'model.sdf'), 'w') as f:
        f.write(sdf)
    with open(os.path.join(destino, 'model.config'), 'w') as f:
        f.write(f'<?xml version="1.0"?>\n<model>\n  <name>{nombre}</name>\n  <version>1.0</version>\n'
                f'  <sdf version="1.9">model.sdf</sdf>\n  <description>{descripcion} '
                f'(generado por generar_vehiculo.py)</description>\n</model>\n')
    if mallas:
        shutil.copytree(mallas, os.path.join(destino, 'meshes'), dirs_exist_ok=True)
    return destino


def main(px4):
    modelos = os.path.join(px4, 'Tools', 'simulation', 'gz', 'models')
    base = open(os.path.join(modelos, 'x500_base', 'model.sdf')).read()
    x500 = open(os.path.join(modelos, 'x500', 'model.sdf')).read()

    sdf, v = sdf_tarot('tarot650_busqueda', base)
    mallas = os.path.join(os.path.dirname(GENERADOR_TAROT), '..', 'models', 'tarot650', 'meshes')
    d = escribir('tarot650_busqueda', sdf, 'Tarot 650 con cámara hacia abajo, LiDAR y viento habilitado',
                 mallas=os.path.normpath(mallas))
    print(f"{d}/model.sdf: Tarot 650 ({v['masa_cuerpo']} kg + rotores), cámara a z = "
          f"{VEHICULOS['tarot650']['z_camara']} m, sensores de PX4")
    escribir_ardupilot('tarot650_busqueda', sdf, VEHICULOS['tarot650']['w_max'])

    sdf = sdf_x500('x500_busqueda', base, x500)
    d = escribir('x500_busqueda', sdf, 'x500 de PX4 con cámara hacia abajo, LiDAR y viento habilitado')
    print(f"{d}/model.sdf: x500, cámara a z = {VEHICULOS['x500']['z_camara']} m")
    escribir_ardupilot('x500_busqueda', sdf, VEHICULOS['x500']['w_max'])


def ruido_imu(stddev, sesgo, sesgo_din):
    return ''.join(f"""
            <{e}><noise type="gaussian"><mean>0</mean><stddev>{stddev}</stddev><bias_mean>0</bias_mean>"""
                   f"""<bias_stddev>{sesgo}</bias_stddev><dynamic_bias_stddev>{sesgo_din}</dynamic_bias_stddev>"""
                   f"""<dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></{e}>"""
                   for e in 'xyz')


def escribir_ardupilot(nombre_base, sdf, w_max):
    """Variante pilotada por ArduPilot SITL. ArduPilot, quad "X": M1 adelante-derecha (ccw),
    M2 atrás-izquierda (ccw), M3 adelante-izquierda (cw), M4 atrás-derecha (cw): el mismo orden
    que los rotor_0..3, así que la salida i mueve el rotor i."""
    nombre = nombre_base + '_ardupilot'
    en_mundo = nombre_base + '_0'        # como lo nombra PX4, para que los tópicos de la cámara coincidan
    controles = ''.join(f"""
      <control channel="{i}">
        <jointName>rotor_{i}_joint</jointName>
        <type>COMMAND</type>
        <cmd_topic>/dron/ap/motor_{i}</cmd_topic>
        <useForce>0</useForce>
        <multiplier>{w_max}</multiplier>
        <offset>0</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
      </control>""" for i in range(4))
    entradas = ''.join(f"\n      <entrada>/dron/ap/motor_{i}</entrada>" for i in range(4))
    extra = f"""
    <plugin filename="ArduPilotPlugin" name="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <!-- Con no_time_sync=1 el backend JSON pone AHRS_EKF_TYPE=10 y ArduCopter vuela con
           la posición y la actitud verdaderas del simulador: el EKF no hace nada. -->
      <no_time_sync>0</no_time_sync>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>base_link::imu_ardupilot</imuName>{controles}
    </plugin>
    <plugin filename="SelectorMotores" name="mision::SelectorMotores">{entradas}
      <red>/dron/red/motor_speed</red>
      <modo>/dron/controlador</modo>
      <salida>/{en_mundo}/command/motor_speed</salida>
      <tiempo_max>0.05</tiempo_max>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_publish_frequency>400</odom_publish_frequency>
      <odom_topic>/dron/odometria</odom_topic>
      <dimensions>3</dimensions>
    </plugin>
  </model>"""
    imu = f"""<link name="base_link">
      <!-- IMU que lee ArduPilot (marco NED: girada 180 grados en x), con ruido y sesgo
           del orden de la IMU de una Pixhawk más la vibración del marco (supuestos). Sin
           ruido el EKF integra la inercia casi perfecta. -->
      <sensor name="imu_ardupilot" type="imu">
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on>
        <update_rate>1000</update_rate>
        <imu>
          <angular_velocity>{ruido_imu(0.004, 0.003, 0.0003)}
          </angular_velocity>
          <linear_acceleration>{ruido_imu(0.1, 0.04, 0.003)}
          </linear_acceleration>
        </imu>
      </sensor>"""
    s = sdf.replace(f"<model name='{nombre_base}'>", f"<model name='{nombre}'>", 1)
    s = s.replace('<link name="base_link">', imu, 1)
    i = s.rindex('  </model>')
    s = s[:i] + extra.lstrip('\n') + s[i + len('  </model>'):]
    # Las mallas se quedan en model://<vehiculo>_busqueda/meshes (también está en la ruta de Gazebo).
    d = escribir(nombre, s, f'{nombre_base} pilotado por ArduPilot SITL')
    print(f'{d}/model.sdf: ArduPilotPlugin (EKF3), SelectorMotores y odometría a 400 Hz')


if __name__ == '__main__':
    main(os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/PX4-Autopilot'))
