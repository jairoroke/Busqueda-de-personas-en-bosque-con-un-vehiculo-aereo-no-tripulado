"""Simulación integrada de la búsqueda de personas.

Genera el bosque, arranca Gazebo con él (sin el server.config de PX4), el agente
uXRCE-DDS, PX4 SITL (px4_sitl_neural, airframe 4100_gz_tarot650_busqueda) conectado
a ese mundo, el puente Gazebo-ROS 2, el detector, el localizador y la misión.
El vehículo es el Tarot 650 (vehiculo:=tarot650); vehiculo:=x500 usa el x500 de PX4
(airframe 4099_gz_x500_busqueda), con el que se hicieron las primeras corridas.
Cuando la misión termina (escribe `salida`) se cierra todo.

    ros2 launch mision simulacion.launch.py
    ros2 launch mision simulacion.launch.py oclusion:=alta iluminacion:=baja viento:=si semilla:=3
    ros2 launch mision simulacion.launch.py estrategia:=reobs controlador:=red gui:=false salida:=$HOME/resultados/r1.json
    ros2 launch mision simulacion.launch.py piloto:=ardupilot controlador:=red

Con piloto:=ardupilot vuela ArduCopter SITL (GUIDED por MAVLink, mision_ardupilot) y
controlador:=red usa controlador_red + SelectorMotores en lugar del módulo mc_nn_control.

Argumentos: vehiculo (tarot650|x500), oclusion (baja|media|alta), iluminacion (alta|baja), viento (no|si), semilla,
personas (número; vacío = 1 a 3), estrategia (fija|reobs), controlador (cascada|red), piloto (px4|ardupilot),
altura, velocidad, lado, pesos (modelo YOLO), umbral, gui, salida, px4 (ruta de PX4-Autopilot),
latencia_extra_ms y saltar_cuadros (degradación del detector, pregunta P3).

Con GUI se abre además MAVProxy en su propia terminal (consola, mapa y el módulo
mision.mavproxy_trayectoria, que dibuja la ruta, lo recorrido y las personas
localizadas), conectado a PX4 por UDP 14550 o a ArduPilot por TCP 5762, y
mision.ventanas_sitl acomoda las ventanas. mavproxy:=false lo omite.
"""
import os
import signal
import shutil
import sys
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from ament_index_python.packages import get_package_prefix
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction, UnsetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Por vehículo: airframe de PX4, velocidad máxima de los rotores y altura de la cámara bajo
# base_link (scripts/generar_vehiculo.py).
VEHICULOS = {'tarot650': dict(airframe='4100', w_max=680.0, z_camara=-0.24),
             'x500': dict(airframe='4099', w_max=1000.0, z_camara=-0.25)}

ARGUMENTOS = {
    'vehiculo': 'tarot650', 'oclusion': 'media', 'iluminacion': 'alta', 'viento': 'no', 'semilla': '1', 'personas': '',
    'estrategia': 'fija', 'controlador': 'cascada', 'altura': '30.0', 'velocidad': '3.0', 'lado': '120.0',
    'pesos': '', 'umbral': '0.25', 'gui': 'true', 'salida': os.path.expanduser('~/resultados/corrida.json'),
    'px4': os.path.expanduser('~/PX4-Autopilot'), 'latencia_extra_ms': '0.0', 'saltar_cuadros': '0',
    'python_yolo': os.path.expanduser('~/.venvs/yolo/bin/python'),
    'piloto': 'px4', 'ardupilot': os.path.expanduser('~/ardupilot'),
    'python_ardupilot': os.path.expanduser('~/venv-ardupilot/bin/python3'),
    'pesos_red': '', 'mavproxy': 'true',
}


def terminar_restos(event, context):
    """Al cerrar, termina MAVProxy (y su consola y mapa) y su terminal, que no siempre se cierran solos."""
    procesos = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                cmd = fh.read().replace(b'\0', b' ').decode(errors='replace')
            with open(f'/proc/{pid}/stat') as fh:
                ppid = int(fh.read().rsplit(')', 1)[1].split()[1])
            if os.stat(f'/proc/{pid}').st_uid == os.getuid():
                procesos[int(pid)] = (ppid, cmd)
        except (OSError, ValueError, IndexError):
            continue
    raices = {p for p, (_, c) in procesos.items()
              if 'mision.mavproxy_trayectoria' in c or (c.startswith('xterm') and 'MAVProxy (mision)' in c)}
    todos, nuevos = set(raices), set(raices)
    while nuevos:
        nuevos = {p for p, (pp, _) in procesos.items() if pp in nuevos} - todos
        todos |= nuevos
    todos.discard(os.getpid())
    for sig in (signal.SIGTERM, signal.SIGKILL):
        vivos = [p for p in todos if _senal(p, sig)]
        if not vivos:
            break
        time.sleep(1.5)


def _senal(pid, sig):
    try:
        os.kill(pid, sig)
        return True
    except OSError:
        return False


def preparar(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGUMENTOS}
    share = get_package_share_directory('mision')
    sys.path.insert(0, os.path.join(share, 'scripts'))
    import generar_mundo

    # 1. Mundo y personas verdaderas
    tmp = f'/tmp/mision_{os.getuid()}/mundos'
    os.makedirs(tmp, exist_ok=True)
    ardupilot = a['piloto'] == 'ardupilot'
    if a['piloto'] not in ('px4', 'ardupilot'):
        raise RuntimeError(f"piloto:={a['piloto']}: debe ser px4 o ardupilot")
    if a['vehiculo'] not in VEHICULOS:
        raise RuntimeError(f"vehiculo:={a['vehiculo']}: debe ser {' o '.join(VEHICULOS)}")
    veh = VEHICULOS[a['vehiculo']]
    modelo = f"{a['vehiculo']}_busqueda"
    # Con ArduPilot la física corre a 1 kHz (lo que espera el SITL por JSON) y el vehículo va en el mundo.
    nombre, sdf, verdad = generar_mundo.generar(a['oclusion'], a['iluminacion'], a['viento'], int(a['semilla']),
                                                lado=float(a['lado']),
                                                n_personas=int(a['personas']) if a['personas'] else None,
                                                paso=0.001 if ardupilot else 0.004,
                                                vehiculo=modelo + '_ardupilot' if ardupilot else None)
    ruta_mundo = os.path.join(tmp, nombre + '.sdf')
    ruta_verdad = os.path.join(tmp, f'personas_{nombre}.json')
    with open(ruta_mundo, 'w') as f:
        f.write(sdf)
    import json
    with open(ruta_verdad, 'w') as f:
        json.dump(verdad, f, indent=2)
    salida = os.path.expanduser(a['salida'])
    os.makedirs(os.path.dirname(salida) or '.', exist_ok=True)
    if os.path.exists(salida):
        os.remove(salida)

    px4 = os.path.expanduser(a['px4'])
    rootfs = os.path.join(px4, 'build', 'px4_sitl_neural', 'rootfs')
    ap = os.path.expanduser(a['ardupilot'])
    requisitos = [(os.path.join(px4, 'Tools/simulation/gz/models/x500_base'), 'modelo x500 de PX4 (sus sensores también los usa el Tarot)'),
                  (os.path.join(share, 'models', modelo), f'vehículo {modelo} (scripts/generar_vehiculo.py)')]
    if ardupilot:
        requisitos += [(os.path.join(ap, 'build', 'sitl', 'bin', 'arducopter'), 'ArduCopter SITL compilado'),
                       (a['python_ardupilot'], 'Python de ArduPilot (pymavlink)')]
    else:
        requisitos += [(os.path.join(rootfs, '..', 'bin', 'px4'), 'PX4 compilado (make px4_sitl_neural)'),
                       (os.path.join(px4, 'Tools/simulation/gz/models', modelo), 'vehículo en PX4 (scripts/instalar_px4.sh)')]
    for ruta, que in requisitos:
        if not os.path.exists(ruta):
            raise RuntimeError(f'No se encontró el {que}: {ruta}')
    agente = shutil.which('MicroXRCEAgent') or os.path.expanduser('~/.local/bin/MicroXRCEAgent')
    pesos = a['pesos'] or os.path.join(share, 'yolo', 'personas_yolo26s.pt')
    if not os.path.exists(pesos):
        raise RuntimeError(f'No existe el modelo YOLO {pesos}; entrénalo con scripts/entrenar_detector.py o pasa pesos:=')
    gui = a['gui'].lower() in ('true', '1', 'si')

    # 2. Gazebo
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r'] + (['--gui-config', os.path.join(share, 'config', 'gazebo_gui.config')] if gui else ['-s'])
        + [ruta_mundo], output='log', name='gazebo', sigterm_timeout='5')

    # 3. Agente uXRCE-DDS y PX4 conectado al mundo ya abierto (PX4_GZ_STANDALONE)
    xrce = ExecuteProcess(cmd=[agente, 'udp4', '-p', '8888'], output='log', name='agente_xrce')
    env_px4 = {'PX4_GZ_STANDALONE': '1', 'PX4_SYS_AUTOSTART': veh['airframe'], 'PX4_SIM_MODEL': f'gz_{modelo}',
               'PX4_GZ_WORLD': nombre, 'GZ_IP': '127.0.0.1'}
    px4_proc = ExecuteProcess(cmd=['../bin/px4', '-d'], cwd=rootfs, additional_env=env_px4, output='log', name='px4',
                              sigterm_timeout='3', sigkill_timeout='3')

    # 4. Puente Gazebo -> ROS 2 (cámara, reloj) y ROS 2 -> Gazebo (imagen anotada para el panel)
    cam = f'/world/{nombre}/model/{modelo}_0/link/camera_link/sensor/imager'
    puente = Node(package='ros_gz_bridge', executable='parameter_bridge', name='puente', output='log',
                  arguments=[f'{cam}/image@sensor_msgs/msg/Image[gz.msgs.Image',
                             f'{cam}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                             f'/world/{nombre}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/dron/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                             '/camera_yolo@sensor_msgs/msg/Image]gz.msgs.Image'],
                  remappings=[(f'{cam}/image', '/camera'), (f'{cam}/camera_info', '/camera_info'),
                              (f'/world/{nombre}/clock', '/clock')])

    # 5. Nodos de la misión
    detector = ExecuteProcess(
        cmd=[a['python_yolo'], '-m', 'mision.detector_node', '--ros-args', '-p', f'pesos:={pesos}',
             '-p', f'umbral:={a["umbral"]}', '-p', f'latencia_extra_ms:={a["latencia_extra_ms"]}',
             '-p', f'saltar_cuadros:={a["saltar_cuadros"]}', '-p', f'anotar:={str(gui).lower()}',
             '-p', 'use_sim_time:=true'], output='screen', name='detector')
    localizador = Node(package='mision', executable='localizador_node', output='screen',
                       parameters=[{'use_sim_time': True, 'z_camara': veh['z_camara']}])
    archivo_mapa = f'/tmp/mision_{os.getuid()}/mapa_busqueda.json'
    if os.path.exists(archivo_mapa):
        os.remove(archivo_mapa)
    param_mision = {'estrategia': a['estrategia'], 'controlador': a['controlador'], 'altura': float(a['altura']),
                    'velocidad': float(a['velocidad']), 'lado': float(a['lado']), 'verdad': ruta_verdad, 'salida': salida,
                    'archivo_mapa': archivo_mapa}
    if not ardupilot:
        mision = Node(package='mision', executable='mision_node', output='screen', parameters=[param_mision])
        piloto = [xrce, TimerAction(period=3.0, actions=[px4_proc])]
    else:
        # ArduCopter SITL por JSON (ArduPilotPlugin), el mismo comando que arma sim_vehicle.py pero sin
        # MAVProxy ni ventana xterm, para que el launch lo cierre; la misión se conecta a su puerto TCP 5760
        # (para una estación de tierra: mavproxy.py --master tcp:127.0.0.1:5762).
        trabajo = f'/tmp/mision_{os.getuid()}/ardupilot'
        os.makedirs(trabajo, exist_ok=True)
        d = os.path.join(ap, 'Tools', 'autotest', 'default_params')
        sitl = [os.path.join(ap, 'build', 'sitl', 'bin', 'arducopter'), '-w', '--model', 'JSON', '--speedup', '1',
                '--slave', '0', '--defaults', ','.join([os.path.join(d, 'copter.parm'), os.path.join(d, 'gazebo-iris.parm'),
                                                       os.path.join(share, 'config', 'ardupilot', f"{a['vehiculo']}.parm")]),
                '--sim-address=127.0.0.1', '-I0']
        from mision.geo import ORIGEN                        # home = origen del mundo: el GPS coincide con los avisos
        sitl += ['--home', f'{ORIGEN[0]},{ORIGEN[1]},{ORIGEN[2]},0']
        sitl_proc = ExecuteProcess(cmd=sitl, cwd=trabajo, output='log', name='arducopter', sigterm_timeout='3')
        param_mision['conexion'] = 'tcp:127.0.0.1:5760'
        args = [f'{k}:={v}' for k, v in param_mision.items()]
        mision = ExecuteProcess(cmd=[a['python_ardupilot'], '-m', 'mision.mision_ardupilot', '--ros-args']
                                + sum((['-p', x] for x in args), []), output='screen', name='mision')
        piloto = [sitl_proc]
        if a['controlador'] == 'red':
            pesos_red = a['pesos_red'] or os.path.join(share, 'codigo', f'pesos_nn_{modelo}.npz')
            piloto.append(Node(package='mision', executable='controlador_red', output='screen',
                               parameters=[{'pesos': pesos_red, 'w_max': veh['w_max']}]))
    fin = RegisterEventHandler(OnProcessExit(target_action=mision, on_exit=[EmitEvent(event=Shutdown(reason='misión terminada'))]))
    # Estación de tierra: MAVProxy con consola, mapa y la trayectoria (sólo con GUI)
    gcs = []
    if gui and a['mavproxy'].lower() in ('true', '1', 'si'):
        maestro = 'tcp:127.0.0.1:5762' if ardupilot else 'udp:127.0.0.1:14550'
        mavproxy = os.path.join(os.path.dirname(a['python_ardupilot']), 'mavproxy.py')
        gcs = [TimerAction(period=12.0, actions=[ExecuteProcess(
                   cmd=['xterm', '-T', 'MAVProxy (mision): escribe aquí los comandos', '-geometry', '110x32+0+0',
                        '-fa', 'Monospace', '-fs', '11', '-e', a['python_ardupilot'], mavproxy, '--master', maestro,
                        '--console', '--map', '--load-module=mision.mavproxy_trayectoria'],
                   output='log', name='mavproxy')]),
               ExecuteProcess(cmd=[a['python_ardupilot'], '-m', 'mision.ventanas_sitl', '150'], output='log', name='ventanas')]
    print(f'[mision] mundo {nombre}: {verdad["n_arboles"]} árboles, cobertura {100*verdad["cobertura_copas"]:.0f} %, '
          f'{len(verdad["personas"])} personas; registro en {salida}')
    recursos = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(
        [os.path.join(share, 'models'), os.path.join(px4, 'Tools/simulation/gz/models'),
         os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))]
    if ardupilot:
        recursos.append(AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                                  os.path.join(get_package_prefix('ardupilot_gazebo'), 'lib', 'ardupilot_gazebo')))
    return recursos + [gz] + piloto + [puente, detector, localizador, TimerAction(period=20.0, actions=[mision]), fin] + gcs


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGUMENTOS.items()] + [
            UnsetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH'),   # no cargar GstCameraSystem de PX4
            SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
            RegisterEventHandler(OnShutdown(on_shutdown=terminar_restos)),
            OpaqueFunction(function=preparar),
        ])
