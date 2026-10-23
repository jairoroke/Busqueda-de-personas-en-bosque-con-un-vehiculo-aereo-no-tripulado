#!/usr/bin/env bash
# puentes.sh -- Puentes Gazebo <-> ROS 2 para la simulación integrada (Sección 4.4),
# para lanzar a mano como en la Sección 8.2 del diseño original (el launch ya los incluye).
# Uso: bash puentes.sh <nombre_del_mundo> [modelo]
# Expone la imagen y los parámetros de la cámara del vehículo, el reloj de la
# simulación y las poses verdaderas de todos los modelos del mundo.
#
# Diferencias con el Código 8.2 del diseño original: el sensor de la cámara se llama
# "imager" (no "camera"), el vehículo es x500_busqueda_0 y el panel de Gazebo
# recibe la imagen anotada del detector (/camera_yolo).
MUNDO=${1:?nombre del mundo, p. ej. bosque_media_alta_no_1}
MODELO=${2:-x500_busqueda_0}
CAM=/world/${MUNDO}/model/${MODELO}/link/camera_link/sensor/imager
exec ros2 run ros_gz_bridge parameter_bridge \
    "${CAM}/image@sensor_msgs/msg/Image[gz.msgs.Image" \
    "${CAM}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
    "/world/${MUNDO}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" \
    "/world/${MUNDO}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
    "/camera_yolo@sensor_msgs/msg/Image]gz.msgs.Image" \
    --ros-args -r "${CAM}/image:=/camera" -r "${CAM}/camera_info:=/camera_info" \
    -r "/world/${MUNDO}/pose/info:=/tf" -r "/world/${MUNDO}/clock:=/clock"
