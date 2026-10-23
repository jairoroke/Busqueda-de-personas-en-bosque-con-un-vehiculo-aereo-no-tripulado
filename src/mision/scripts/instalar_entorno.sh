#!/usr/bin/env bash
# instalar_entorno.sh -- Instalación del entorno de simulación integrada en Ubuntu 24.04
# (ROS 2 Jazzy y Gazebo Harmonic ya instalados, ver ~/ros2_ws/README.md de cuadrirotor_sim):
# PX4 release/1.17 con el módulo de control neuronal, agente uXRCE-DDS, px4_msgs, el
# paquete "mision" y los entornos de Python. Ejecutar bloque por bloque; ninguno usa sudo.
#
# Diferencias con el Código 2.1 del diseño original (comprobadas el 22 oct 2026):
#   - La etiqueta v1.17.0 de PX4 no existe (sólo v1.17.0-rc2); se usa la rama release/1.17.
#   - Ubuntu 24.04 no permite `pip install --user` sobre el Python del sistema (PEP 668):
#     las dependencias de PX4 van en ~/.venvs/px4 y las de los scripts en ~/venv-mision.
#   - El agente se instala en ~/.local (no hace falta sudo make install).
#   - Hay que instalar el vehículo, el airframe y la red en PX4 con scripts/instalar_px4.sh.
set -e

# 1. PX4 release/1.17 (con submódulos, ~2 GB) y sus dependencias de Python
cd ~ && git clone --recursive -b release/1.17 https://github.com/PX4/PX4-Autopilot.git
python3 -m venv --system-site-packages ~/.venvs/px4
~/.venvs/px4/bin/pip install -r ~/PX4-Autopilot/Tools/setup/requirements.txt ninja
# (si faltan paquetes del sistema para compilar: bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx, con sudo)

# 2. Agente uXRCE-DDS 2.4.3 en ~/.local (sin ROS en el entorno para que compile su propio Fast-DDS)
cd ~ && git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir -p build && cd build
env -i HOME=$HOME PATH=/usr/bin:/bin cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local \
    -DCMAKE_INSTALL_RPATH=$HOME/.local/lib -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON
env -i HOME=$HOME PATH=/usr/bin:/bin make -j"$(nproc)" && make install

# 3. px4_msgs de la misma versión que el firmware y el paquete mision
cd ~/ros2_ws/src && git clone -b release/1.17 https://github.com/PX4/px4_msgs.git
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select px4_msgs mision

# 4. Python: scripts del proyecto (numpy, scipy, matplotlib, TensorFlow para exportar la red)
python3 -m venv ~/venv-mision
~/venv-mision/bin/pip install numpy scipy matplotlib pillow tensorflow-cpu
#    Detector: ~/.venvs/yolo (ultralytics con PyTorch y CUDA, numpy<2 para ver los mensajes de ROS 2),
#    el mismo de cuadrirotor_sim (instalar_vision_yolo.sh).

# 5. Red, vehículo y airframe en PX4, y compilación de px4_sitl_neural
cd ~/ros2_ws/src/mision/codigo
python3 nn_imitacion.py --masa 2.114 --salida pesos_nn_x500_busqueda.npz --figura ../imagenes/nn_imitacion_x500_busqueda.pdf
~/venv-mision/bin/python exportar_tflite.py --pesos pesos_nn_x500_busqueda.npz --salida control_net_x500_busqueda
bash ../scripts/instalar_px4.sh ~/ros2_ws/src/mision/codigo/control_net_x500_busqueda

# 6. Prueba rápida: ros2 launch mision simulacion.launch.py   (ver README.md)
