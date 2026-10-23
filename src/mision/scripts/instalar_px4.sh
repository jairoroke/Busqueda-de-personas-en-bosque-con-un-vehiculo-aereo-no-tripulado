#!/usr/bin/env bash
# instalar_px4.sh -- Instala en PX4-Autopilot lo que el proyecto agrega al firmware
# y recompila el objetivo px4_sitl_neural:
#   1. el vehículo x500_busqueda (Tools/simulation/gz/models),
#   2. el airframe 4099_gz_x500_busqueda (ROMFS) con los parámetros del modo
#      Neural Control,
#   3. la red del controlador neuronal (control_net.cpp/.hpp del módulo
#      mc_nn_control), exportada con codigo/exportar_tflite.py.
# La red original de PX4 se guarda una sola vez como control_net.*.px4.
#
# Uso: bash scripts/instalar_px4.sh [ruta/a/control_net] [~/PX4-Autopilot]
#      (sin argumentos usa codigo/control_net.cpp y .hpp del paquete)
set -e
PAQ="$(cd "$(dirname "$0")/.." && pwd)"
RED="${1:-$PAQ/codigo/control_net}"
PX4="${2:-$HOME/PX4-Autopilot}"

python3 "$PAQ/scripts/generar_vehiculo.py" "$PX4"
rm -rf "$PX4/Tools/simulation/gz/models/x500_busqueda"
cp -r "$PAQ/models/x500_busqueda" "$PX4/Tools/simulation/gz/models/"

AF="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"
cp "$PAQ/px4/4099_gz_x500_busqueda" "$AF/"
grep -q 4099_gz_x500_busqueda "$AF/CMakeLists.txt" || \
    sed -i 's/^\(\s*\)4014_gz_x500_mono_cam_down$/&\n\14099_gz_x500_busqueda/' "$AF/CMakeLists.txt"
grep -q 4099_gz_x500_busqueda "$AF/CMakeLists.txt" || { echo "No se pudo registrar el airframe"; exit 1; }

NN="$PX4/src/modules/mc_nn_control"
for ext in cpp hpp; do
    [ -e "$NN/control_net.$ext.px4" ] || cp "$NN/control_net.$ext" "$NN/control_net.$ext.px4"
    cp "$RED.$ext" "$NN/control_net.$ext"
done
echo "Red instalada: $RED.{cpp,hpp} ($(grep -o 'size = [0-9]*' "$NN/control_net.hpp"))"

# Tópico de depuración del módulo (entradas de la red y comando de motores) en
# ROS 2, para comparar con Python (paso 5 de la Sección 7.5 del diseño original).
DDS="$PX4/src/modules/uxrce_dds_client/dds_topics.yaml"
grep -q /fmu/out/neural_control "$DDS" || sed -i 's#^publications:$#&\n\n  - topic: /fmu/out/neural_control\n    type: px4_msgs::msg::NeuralControl\n    rate_limit: 100.#' "$DDS"

source /opt/ros/jazzy/setup.bash
[ -d "$HOME/.venvs/px4" ] && export PATH="$HOME/.venvs/px4/bin:$PATH"
cd "$PX4" && make px4_sitl_neural
echo "Listo: PX4_SYS_AUTOSTART=4099 PX4_SIM_MODEL=gz_x500_busqueda"
