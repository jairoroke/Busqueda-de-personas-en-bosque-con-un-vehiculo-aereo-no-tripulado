#!/usr/bin/env bash
# instalar_px4.sh -- Instala en PX4-Autopilot lo que el proyecto agrega al firmware
# y recompila el objetivo px4_sitl_neural:
#   1. los vehículos tarot650_busqueda y x500_busqueda (Tools/simulation/gz/models),
#   2. los airframes 4100_gz_tarot650_busqueda y 4099_gz_x500_busqueda (ROMFS) con
#      los parámetros del modo Neural Control,
#   3. la red del controlador neuronal (control_net.cpp/.hpp del módulo
#      mc_nn_control), exportada con codigo/exportar_tflite.py. PX4 lleva una sola
#      red compilada: la del vehículo con el que se va a volar.
# La red original de PX4 se guarda una sola vez como control_net.*.px4.
#
# Uso: bash scripts/instalar_px4.sh [ruta/a/control_net] [~/PX4-Autopilot]
#      (sin argumentos usa codigo/control_net_tarot650_busqueda.cpp y .hpp, la del Tarot)
set -e
PAQ="$(cd "$(dirname "$0")/.." && pwd)"
RED="${1:-$PAQ/codigo/control_net_tarot650_busqueda}"
PX4="${2:-$HOME/PX4-Autopilot}"

python3 "$PAQ/scripts/generar_vehiculo.py" "$PX4"
AF="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"
for V in tarot650_busqueda:4100 x500_busqueda:4099; do
    M=${V%%:*}; N=${V##*:}
    rm -rf "$PX4/Tools/simulation/gz/models/$M"
    cp -r "$PAQ/models/$M" "$PX4/Tools/simulation/gz/models/"
    cp "$PAQ/px4/${N}_gz_$M" "$AF/"
    grep -q "${N}_gz_$M" "$AF/CMakeLists.txt" || \
        sed -i "s/^\(\s*\)4014_gz_x500_mono_cam_down$/&\n\1${N}_gz_$M/" "$AF/CMakeLists.txt"
    grep -q "${N}_gz_$M" "$AF/CMakeLists.txt" || { echo "No se pudo registrar el airframe ${N}_gz_$M"; exit 1; }
done

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
echo "Listo: PX4_SYS_AUTOSTART=4100 PX4_SIM_MODEL=gz_tarot650_busqueda (x500: 4099, gz_x500_busqueda)"
