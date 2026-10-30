# Búsqueda de personas en bosque con un vehículo aéreo no tripulado

Percepción y estimación visual embarcada para que un x500 busque y localice personas en un bosque por
sí solo. Dr. Jairo Olguín Roque, FES Acatlán, UNAM.

<!-- demo -->
## Demostración

![El Tarot 650 barre el bosque a 30 m siguiendo la ruta de búsqueda (vista de Gazebo que sigue al dron)](https://github.com/jairoroke/Busqueda-de-personas-en-bosque-con-un-vehiculo-aereo-no-tripulado/raw/main/docs/demo/demo.gif)

Misión de barrido fijo con `piloto:=ardupilot` y el controlador en cascada: despegue, 8 puntos de ruta a 30 m sobre un bosque de 295 árboles (cobertura 45 %) y aterrizaje; la misión terminó en 356 s con un RMSE de seguimiento de 0.10 m. Video: [docs/demo/demo.mp4](https://github.com/jairoroke/Busqueda-de-personas-en-bosque-con-un-vehiculo-aereo-no-tripulado/raw/main/docs/demo/demo.mp4)

| Inicio del barrido | Mitad | Final |
|---|---|---|
| ![1](https://github.com/jairoroke/Busqueda-de-personas-en-bosque-con-un-vehiculo-aereo-no-tripulado/raw/main/docs/demo/camara_150.jpg) | ![2](https://github.com/jairoroke/Busqueda-de-personas-en-bosque-con-un-vehiculo-aereo-no-tripulado/raw/main/docs/demo/camara_241.jpg) | ![3](https://github.com/jairoroke/Busqueda-de-personas-en-bosque-con-un-vehiculo-aereo-no-tripulado/raw/main/docs/demo/camara_332.jpg) |

Corrida en Ubuntu 24.04 (WSL2), ROS 2 Jazzy, Gazebo Harmonic y ArduCopter 4.7 SITL.

<!-- /demo -->

Simulación integrada en ROS 2 Jazzy + Gazebo Harmonic: bosque generado por factores, x500 con
cámara y LiDAR, detector YOLO26s, localización rayo–terreno con fusión, avisos con coordenadas GPS,
misión de barrido fijo o con re-observación, controlador en cascada o red neuronal, con PX4 1.17 o
ArduCopter 4.7, y estación de tierra MAVProxy.

| Qué | Dónde |
|---|---|
| Libro para principiantes (todo el proyecto paso a paso) | [`src/mision/doc/manual/main.pdf`](../src/mision/doc/manual/main.pdf) |
| Referencia técnica, argumentos y resultados | [`src/mision/README.md`](../src/mision/README.md) |
| Paquete principal (nodos, launch, scripts, modelos) | `src/mision` |
| Plugin de Gazebo para elegir motores (ArduPilot + red) | `src/mision_gz_plugins` |

## Clonar y compilar

El repositorio tiene la forma de un espacio de trabajo de ROS 2:

```bash
git clone https://github.com/jairoroke/Busqueda-de-personas-en-bosque-con-un-vehiculo-aereo-no-tripulado.git ~/ros2_ws
cd ~/ros2_ws/src
git clone -b release/1.17 https://github.com/PX4/px4_msgs.git
git clone https://github.com/ArduPilot/ardupilot_gazebo.git      # solo para piloto:=ardupilot
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

PX4 (`release/1.17`, objetivo `px4_sitl_neural`), el agente Micro XRCE-DDS 2.4.3 y los
entornos de Python se instalan con `src/mision/scripts/instalar_entorno.sh` (sección
«Instalación» del README del paquete). ArduCopter 4.7.2 SITL va en `~/ardupilot`. El dataset del
detector (`~/datasets/personas_bosque`, 3000 imágenes) no está en el repositorio: se regenera con
`src/mision/scripts/capturar_dataset.py`.

## Lanzar

```bash
ros2 launch mision simulacion.launch.py                                    # PX4, cascada, barrido fijo
ros2 launch mision simulacion.launch.py estrategia:=reobs controlador:=red
ros2 launch mision simulacion.launch.py piloto:=ardupilot controlador:=red
```
