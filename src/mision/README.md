# Búsqueda de personas en bosque con PX4, Gazebo y ROS 2

Percepción y estimación visual embarcada para que un vehículo aéreo no tripulado busque y localice
personas en un bosque. Dr. Jairo Olguín Roque, FES Acatlán, UNAM.

Incluye el modelo dinámico, localización rayo–terreno, controlador neuronal para PX4 y simulación integrada.
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.15, PX4 `release/1.17` (objetivo
`px4_sitl_neural`), agente Micro XRCE-DDS 2.4.3, `px4_msgs` `release/1.17`.

Libro para principiantes con todo el proyecto explicado paso a paso:
[`doc/manual/main.pdf`](doc/manual/main.pdf) (fuente LaTeX en `doc/manual`, `tectonic -X compile main.tex`).

## Lanzar

    source ~/ros2_ws/install/setup.bash
    ros2 launch mision simulacion.launch.py                                   # bosque medio, sol alto, estrategia fija
    ros2 launch mision simulacion.launch.py estrategia:=reobs controlador:=red
    ros2 launch mision simulacion.launch.py oclusion:=alta iluminacion:=baja viento:=si semilla:=4 gui:=false
    ros2 launch mision simulacion.launch.py piloto:=ardupilot                 # el mismo vuelo con ArduCopter SITL
    ros2 launch mision simulacion.launch.py piloto:=ardupilot controlador:=red

El launch genera el bosque, abre Gazebo con él (panel "Detección de personas" a la derecha),
arranca el agente XRCE, PX4 SITL conectado a ese mundo, el puente, el detector YOLO, el
localizador y la misión. El Tarot 650 despega del origen, barre el área de 120 × 120 m en franjas
a 30 m, localiza a las personas y aterriza; al terminar escribe `salida` (por omisión
`~/resultados/corrida.json`) con el registro y las métricas, y se cierra todo.

| Argumento | Valores | |
|---|---|---|
| `vehiculo` | tarot650, x500 | Tarot 650 (por omisión) o el x500 de PX4 de las primeras corridas (ver «Modelos») |
| `oclusion` | baja, media, alta | cobertura de copas 15, 45, 75 % |
| `iluminacion` | alta, baja | sol de mediodía o sombras largas |
| `viento` | no, si | WindEffects, 4 m/s medios con ráfagas |
| `semilla`, `personas` | entero | posiciones de árboles y personas (1 a 3 por omisión) |
| `estrategia` | fija, reobs | barrido simple o con re-observación de candidatas dudosas (Algoritmo 1) |
| `piloto` | px4, ardupilot | PX4 1.17 SITL o ArduCopter 4.7 SITL (ver abajo) |
| `controlador` | cascada, red | control del piloto o red 15-64-32-4 (Neural Control en PX4; `controlador_red` con ArduPilot) |
| `altura`, `velocidad`, `lado` | m, m/s, m | 30, 3, 120 |
| `pesos`, `umbral` | ruta, conf. | modelo YOLO (por omisión `yolo/personas_yolo26s.pt`) |
| `latencia_extra_ms`, `saltar_cuadros` | | degradación del detector (pregunta P3) |
| `gui`, `salida`, `px4` | | |

Una simulación a la vez: el launch usa `/clock`, `/camera` y el puerto 8888 del agente. Si hay
otra abierta (por ejemplo la de `cuadrirotor_sim`), lanza con otra partición y dominio:
`GZ_PARTITION=busqueda ROS_DOMAIN_ID=7 ros2 launch mision simulacion.launch.py`.

Con GUI se abre además **MAVProxy** en su propia terminal (consola, mapa y el módulo
`mision.mavproxy_trayectoria`, que dibuja la ruta, lo recorrido y las personas avisadas),
conectado a PX4 por UDP 14550 o a ArduPilot por TCP 5762; `mision.ventanas_sitl` acomoda las
ventanas (terminal y consola a la izquierda, Gazebo al centro, mapa a la derecha). `mavproxy:=false`
lo omite. Al cerrar, un manejador termina MAVProxy y su terminal.

**Aviso con GPS de cada persona.** Cuando una candidata se confirma durante el barrido (confianza
acumulada ≥ 0.6 y al menos `n_aviso` = 10 detecciones fusionadas, dentro del área y a más de 5 m de
las ya avisadas), `mision_node` publica una sola vez sus coordenadas en `/personas/gps`
(`NavSatFix`, `frame_id` persona_k) y el texto en `/personas/alerta` (QoS transient local), lo
escribe en la terminal y en `~/personas_localizadas.csv`:

    PERSONA 2 LOCALIZADA 2026-10-23 07:50:56
      GPS: 19.482212, -99.245049  (19°28'55.9"N 99°14'42.2"W)
      Mapa: https://www.google.com/maps?q=19.482212,-99.245049
      Local: (-40.0, 35.2) m   confianza 1.00, 10 detecciones

El origen es el del mundo (`<spherical_coordinates>`, 19.4826° N, 99.2447° O, 2250 m; `mision/geo.py`,
conversión exacta WGS84), el mismo que usa el GPS de PX4 y el *home* de ArduPilot, así que todos
coinciden. Ojo: con el detector actual también se avisan falsas alarmas persistentes (grupos de
robles): en las pruebas del 23 de octubre hubo de 4 a 6 avisos para 3 personas.

**Evasión de obstáculos.** El vehículo lleva un LiDAR 3D (`generar_vehiculo.py`; 360° × 120 haces, 16
canales de −35° a +15°, 30 m, 10 Hz, `/dron/lidar/points`) y `mision_node` corrige la consigna con
`mision/evasion.py` (histograma polar: si el tramo hacia el siguiente punto pasa a menos de 3 m de un
obstáculo a la altura del vehículo, toma el rumbo libre más cercano o sube). Es el mismo algoritmo del
Tarot de `cuadrirotor_sim`. A 30 m sobre copas de 6.5 m no actúa en el barrido.

## Con ArduPilot (`piloto:=ardupilot`)

El mismo bosque, cámara, detector, localizador y lógica de misión, pilotados por ArduCopter
4.7.2 SITL (`~/ardupilot`, el de `cuadrirotor_sim`):

- Vehículo `tarot650_busqueda_ardupilot` (generado por `scripts/generar_vehiculo.py`): el mismo Tarot
  con `ArduPilotPlugin` (salidas 1000–2000 µs → 0–680 rad/s, mismo orden de motores que PX4), una
  IMU para ArduPilot, `OdometryPublisher` a 400 Hz y el sistema `SelectorMotores`
  (paquete `mision_gz_plugins`). El mundo corre a 1 kHz e incluye el vehículo.
- ArduCopter vuela con su **EKF3** (`AHRS_EKF_TYPE 3`). El modelo lleva `no_time_sync=0`: con
  `no_time_sync=1` (lo que traen los modelos de `ardupilot_gazebo`, y lo que tenía este paquete hasta
  el 24 oct) el backend JSON fuerza `AHRS_EKF_TYPE=10` y ArduPilot vuela con la posición y la actitud
  **verdaderas** del simulador. La IMU de ArduPilot tiene ruido y sesgo supuestos (giróscopo 0.004
  rad/s, acelerómetro 0.1 m/s²). Las corridas con ArduPilot anteriores al 24 oct no usaban el EKF.
- `arducopter` se lanza directo (sin MAVProxy ni xterm) con `config/ardupilot/tarot650.parm`
  (los parámetros de vuelo del Tarot de `cuadrirotor_sim`);
  `mision_ardupilot` (subclase de `mision_node`) se conecta por TCP 5760, vuela en GUIDED con
  `SET_POSITION_TARGET_LOCAL_NED` y publica el estimado del EKF en `/fmu/out/vehicle_odometry`
  para que el localizador sea el mismo. Para una estación de tierra: `mavproxy.py --master tcp:127.0.0.1:5762`.
- `controlador:=red`: ArduPilot no tiene un módulo como `mc_nn_control`, así que la red corre en
  `controlador_red` (ROS 2 + gz-transport, ~330 Hz, 70 µs por paso) con el estado de Gazebo y manda
  w_i = w_max·√((a_i+1)/2) rad/s (680 en el Tarot). La misión publica "red" en `/dron/controlador` y `SelectorMotores`
  deja pasar esos comandos en lugar de los de ArduPilot, que sigue estimando, recibe las mismas
  consignas y recupera los motores para aterrizar (o si la red deja de publicar 50 ms).

## Campaña experimental

    python3 src/mision/scripts/correr_experimentos.py --semillas 30 --salida ~/resultados     # 840 corridas
    python3 src/mision/scripts/correr_experimentos.py --semillas 2 --solo oclusion=media
    ~/venv-mision/bin/python src/mision/scripts/analisis_wilcoxon.py --resultados ~/resultados

Cada corrida es un `ros2 launch` sin interfaz en su propia partición de Gazebo y dominio de
ROS 2 (unos 5 min). El JSON de cada corrida trae `personas_localizadas`, `falsas_alarmas`,
`error_localizacion_m`, `rmse_seguimiento` (distancia a la ruta durante el barrido),
`reobservaciones`, `duracion_s`, la trayectoria y las candidatas finales.

## Instalación (ya hecha en esta máquina)

`scripts/instalar_entorno.sh` reúne los pasos, sin sudo. Después de cambiar la red o el
vehículo:

    bash src/mision/scripts/instalar_px4.sh ~/ros2_ws/src/mision/codigo/control_net_tarot650_busqueda

copia a `~/PX4-Autopilot` los vehículos `tarot650_busqueda` y `x500_busqueda`, los airframes
`4100_gz_tarot650_busqueda` y `4099_gz_x500_busqueda` (`px4/`), la red (`control_net.cpp/.hpp`; la
original de PX4 queda como `control_net.*.px4`), exporta `/fmu/out/neural_control` a ROS 2 y recompila
`px4_sitl_neural`. PX4 lleva **una sola red compilada**: ahora es la del Tarot. Para volar el x500 con
`controlador:=red` en PX4 hay que reinstalar `control_net_x500_busqueda` (con ArduPilot no hace falta:
`controlador_red` carga los pesos del vehículo).

## Programas

| Archivo | Qué hace |
|---|---|
| `codigo/sim_modelo.py` | Modelo (3.12)–(3.17) con el x500 y el control base; reproduce el Cuadro 9 |
| `codigo/sim_localizacion.py` | Localización rayo–terreno, covarianza (4.7) y Monte Carlo; reproduce el Cuadro 16 |
| `codigo/nn_imitacion.py` | Red 15-64-32-4 por clonación + DAgger; reproduce el Cuadro 11 (`--vehiculo x500`). `--vehiculo tarot650` para el Tarot |
| `codigo/ppo_refinamiento.py` | Refinamiento por PPO (`--horizonte`, `--log-std`, `--masa`) |
| `codigo/exportar_tflite.py` | TFLite float32 y `control_net.cpp/.hpp` para `mc_nn_control` |
| `mision/detector_node.py` | YOLO sobre `/camera` → `/detecciones`, `/camera_yolo` |
| `mision/localizador_node.py` | Rayo–terreno, covarianza, asociación de Mahalanobis y fusión → `/personas/candidatas` |
| `mision/mision_node.py` | Barrido offboard, re-observación, cambio a Neural Control, métricas |
| `mision/mision_ardupilot.py` | La misma misión con ArduCopter (GUIDED por MAVLink) |
| `mision/controlador_red.py` | La red como controlador de motores con ArduPilot |
| `mision/modo_neural.py` | Activa Neural Control a mano |
| `mision/prueba_neural.py` | Escalones en SITL con cascada o red (pasos 4 y 5 de la Sección 7.5) |
| `scripts/generar_mundo.py` | Bosque por oclusión, iluminación, viento y semilla + JSON de personas verdaderas |
| `scripts/capturar_dataset.py` | Dataset YOLO desde Gazebo con la cámara de cajas visibles |
| `scripts/entrenar_detector.py` | Entrenamiento y evaluación por estrato del detector |
| `scripts/preparar_datos.py` | Conversión y mosaicos de conjuntos reales (HERIDAL, VOC o CSV) |
| `scripts/correr_experimentos.py`, `scripts/analisis_wilcoxon.py` | Plan factorial y pruebas estadísticas |
| `scripts/generar_vehiculo.py`, `generar_suelo.py`, `importar_personas.py` | Modelos del paquete |

Pruebas: `cd src/mision && python3 -m pytest -q test/` (geometría, métricas y el mapeo de la
red por la cadena de PX4).

## Modelos

- `tarot650_busqueda` (el que se usa): el Tarot Ironman 650 de `cuadrirotor_sim` (mallas del CAD,
  masa, inercias, hélices y motores de la tabla `VEHICULOS['tarot650']` de su `generar_modelo.py`, así
  que los dos proyectos usan el mismo dron), 2.25 kg con rotores y cámara, con los sensores de PX4 del
  x500 (IMU, barómetro, magnetómetro y GPS), la cámara mono de 1280 × 960 y campo horizontal de 1.74
  rad hacia abajo, 24 cm bajo el centro, el LiDAR y viento habilitado. **Masa, inercias, kf y retardos
  de los motores son supuestos**: hay que reemplazarlos por los del dron real en esa tabla y volver a
  generar (`generar_vehiculo.py`), reentrenar la red e instalarla.
- `x500_busqueda`: el x500 de PX4 (2.114 kg con rotores y cámara) con la misma cámara, 25 cm bajo el
  centro. Con él se hicieron las corridas del 22 y 23 de octubre.
- `persona_de_pie`, `persona_caminando`, `persona_mujer`, `persona_sentada`: OpenRobotics en Gazebo
  Fuel (CC0), estáticos y con la etiqueta 1 del sistema `Label`. La persona acostada es una de pie
  girada 90°.
- `roble`, `pino` (OpenRobotics, CC0; copas de 10 y 3.2 m de diámetro), `suelo` (pasto y tierra procedurales).

## Resultados con el Tarot 650 (24 oct 2026)

Mismo escenario que las tablas del x500 (bosque medio, sol alto, sin viento, semilla 1, 3 personas,
30 m, detector final, estrategia fija). Red del Tarot `pesos_nn_tarot650_busqueda.npz` (imitación con
`--vehiculo tarot650 --adelanto`); ArduPilot ya vuela con el EKF3.

| Piloto | Controlador | Localizadas | Falsas alarmas (n ≥ 1 / n ≥ 10) | Avisos GPS | Error medio | RMSE ruta | Duración |
|---|---|---|---|---|---|---|---|
| PX4 | cascada | 3/3 | 16 / 2 | 4 | 0.54 m | 0.08 m | 248 s |
| PX4 | red | 3/3 | 10 / 3 | 4 | 0.49 m | 0.27 m | 213 s |
| ArduPilot (EKF3) | cascada | 3/3 | 18 / 2 | 5 | 0.65 m | 0.10 m | 337 s |
| ArduPilot (EKF3) | red | 3/3 | 19 / 5 | 4 | 0.71 m | 0.29 m | 269 s |

**Corrección de las esquinas.** En las primeras corridas del Tarot con la red el RMSE de la ruta era
0.39–0.44 m y con ArduPilot y la red hubo 51 / 17 falsas alarmas. La red seguía bien la consigna: era la
consigna la que se salía de la ruta. `mision_node` cambiaba de punto de ruta cuando el *vehículo*
llegaba a 1.5 m del punto; con la cascada el vehículo va atrás de la consigna y ella siempre llega
primero, pero con la red y la prealimentación el vehículo va a la par y la consigna, a ~1 m de la
esquina, se iba en diagonal al siguiente punto. Recortaba la esquina y el vehículo quedaba corrido
0.3–0.5 m hacia el lado de la vuelta durante todo el tramo siguiente. Ahora el punto cambia cuando el
vehículo está a menos de 1.5 m **y** la consigna ya llegó a él. Con eso, en los tramos rectos la red
sigue la ruta igual que la cascada (0.08 m RMSE a más de 20 m de una esquina); lo que queda es el rebase
en las vueltas de 90° a 3 m/s (0.7 m RMSE en los 4 m alrededor de cada esquina). La corrección también
aplica al x500: sus cifras con la red de la tabla del 23 oct tenían el mismo efecto.

**EKF3 y falsas alarmas.** Comparando la actitud que recibe el localizador con la verdadera de Gazebo:
error de 0.30° RMS en alabeo y cabeceo con la cascada y 0.38–0.46° con la red (1° en guiñada en los
dos); a 30 m eso son ~0.2 m en el suelo, no explica falsas alarmas. Con la corrección, las falsas
alarmas con la red quedan como con la cascada; entre corridas iguales varían bastante (con cascada
26 / 3 y 18 / 2), así que la comparación de verdad es la de la campaña con varias semillas.

## Controlador neuronal mejorado (23 oct 2026)

Tres problemas de la red del 22 de octubre y cómo se corrigieron:

1. **Sesgo horizontal de ~0.2 m.** La cámara (50 g) tiene su centro de masa a 3 cm del eje y corre el del
   vehículo 0.7 mm: sin integral, la red se quedaba inclinada ~1°. `nn_imitacion.py --com 0.00071 0.00071`
   usa la geometría real en el simulador y en el experto: sesgo en punto fijo de 25.9 cm a 2.1 cm.
2. **La red iba a la mitad de velocidad.** Una PD sin prealimentación se atrasa kd/kp · v = 4.8 m a 3 m/s y,
   con el límite de adelanto de la consigna, avanzaba a ~1.6 m/s. Ahora `mision_node` manda con
   `controlador:=red` la consigna adelantada p_ref + (kd/kp) v_ref (`prealimentacion`), que equivale a
   una PD con prealimentación, y la red se reentrenó con `--adelanto` (referencias móviles de hasta 4 m/s;
   la red anterior con la consigna adelantada se desbocaba porque nunca había visto esos errores).
3. **PPO divergía.** Con la ec. (4.4) cada paso valía hasta −25 y estrellarse solo −10: terminar pronto
   "convenía". `ppo_refinamiento.py --mejorado` usa recompensa acotada y positiva, bootstrap en los cortes
   por tiempo, calentamiento del crítico, KL y guarda la mejor política. Ya no diverge (0 fallos en 140
   iteraciones), pero tampoco mejora a la imitación (con `exp` de 1.245 a 1.53 m; con `--forma lineal`
   a 1.46 m): **se usa la red de imitación**. Sin las opciones nuevas, `nn_imitacion.py` y
   `ppo_refinamiento.py` dan pesos idénticos a los anteriores (siguen reproduciendo los resultados de referencia).

Red del x500: `pesos_nn_x500_busqueda.npz` / `control_net_x500_busqueda.*` (la anterior respaldada fuera
del paquete). Además el localizador descarta rayos a más de 60° de la vertical (`angulo_max_deg`): en las
vueltas el vehículo se inclinaba, la cámara veía cerca del horizonte y salían candidatas a ~250 m.

Misión integrada (mismo escenario que la tabla de abajo, detector final, 23 oct):

| Piloto | Controlador | Localizadas | Falsas alarmas (n ≥ 1 / n ≥ 10) | Error medio | RMSE ruta | Duración |
|---|---|---|---|---|---|---|
| PX4 | cascada | 3/3 | 31 / 4 | 0.61 m | 0.07 m | 238 s |
| PX4 | red (nueva) | 3/3 | 28 / 5 | 0.52 m | 0.23 m | **211 s** (antes 427 s) |
| ArduPilot | red (nueva) | 3/3 | 30 / 5 | 0.47 m | 0.19 m | **240 s** (antes 451 s) |

## Resultados del 22 oct 2026

**Programas de referencia.** `sim_modelo.py`, `sim_localizacion.py`, `nn_imitacion.py`, la prueba de 10
iteraciones de `ppo_refinamiento.py` y `exportar_tflite.py` reproducen exactamente las tablas
y salidas de referencia (14 936 bytes, diferencia TFLite–numpy 1.1e-6).

**Detector** (YOLO26s ajustado con 3000 imágenes del simulador, 13 202 personas, 1280 px; validación de
300 imágenes): AP50 0.954, AP50-95 0.692, P 0.96, R 0.92, 15.6 ms por imagen en la RTX 5060 Ti. El
YOLO26n de COCO localizó 0 de 3 personas en la misma misión.

| Estrato | AP50 | AP50-95 | Recall |
|---|---|---|---|
| Oclusión baja / media / alta | 0.984 / 0.947 / 0.887 | 0.753 / 0.672 / 0.598 | 0.98 / 0.92 / 0.84 |
| Altura < 20 m / 20–32 m / > 32 m | 0.972 / 0.956 / 0.943 | 0.802 / 0.723 / 0.609 | 0.96 / 0.92 / 0.91 |
| Sol alto / bajo | 0.961 / 0.947 | 0.716 / 0.672 | 0.92 / 0.92 |

**Controlador neuronal en SITL** (`prueba_neural`, escalones de 1–1.8 m a 5 m de altura). Con la red
de m = 2.0 kg queda un error estacionario en z de −0.27 m: el x500 de Gazebo pesa 2.114 kg y la red
imita una cascada PD sin integral (Δz = Δm g / (m k_p)). Con `--masa 2.114` el error en z es 0; queda
un sesgo horizontal de ~0.2 m por el centro de masa de la cámara (3 cm fuera del eje; ~1° de
inclinación estacionaria), que la red no puede corregir porque la interfaz de mc_nn_control no tiene
memoria. RMSE en los escalones: red 0.65 m, cascada de PX4 0.50 m. PPO (300 iteraciones, e^-3,
horizonte 1024) no mejoró a la imitación: estuvo a la par hasta la iteración ~100 (1.31–1.60 m) y luego
divergió (3.2 m, 4 fallos de 40); hay que ajustar la recompensa (4.4) antes de usarlo.

**Misión integrada** (bosque medio, sol alto, sin viento, semilla 1, 3 personas, 30 m; detector
provisional de la época ~40):

| Piloto | Controlador | Estrategia | Localizadas | Falsas alarmas (n ≥ 1 / n ≥ 10) | Error medio | RMSE ruta | Duración |
|---|---|---|---|---|---|---|---|
| PX4 | cascada | fija | 3/3 | 12 / 1 | 0.54 m | 0.07 m | 238 s |
| ArduPilot | cascada | fija | 3/3 | 10 / – | 0.45 m | 0.05 m | 285 s |
| PX4 | red | fija | 3/3 | 18 / 3 | 0.71 m | 0.11 m | 427 s |
| ArduPilot | red | fija | 3/3 | 17 / 7 | 0.46 m | 0.12 m | 451 s |
| PX4 | cascada | reobs | 3/3 | 11 / 3 | 0.65 m | 0.25 m | 277 s |
| ArduPilot | cascada | reobs | 3/3 | 13 / 2 | 0.49 m | 0.33 m | 325 s |

Con la red la misión dura casi el doble: la consigna avanza a lo más 3 m por delante y la red, sin
prealimentación, sigue a ~1.6 m/s. Las falsas alarmas son candidatas de 1 a 3 detecciones cuya
confianza acumulada 1 − Π(1 − c_i) pasa de 0.6; exigir n ≥ 10 detecciones fusionadas
(`min_detecciones`, el JSON trae el barrido 1/3/5/10) las reduce a 1–7 sin perder personas. Una sola
semilla no permite comparar estrategias: eso es lo que hace la campaña con `correr_experimentos.py`.

## Problemas encontrados en el diseño original (ya corregidos aquí)

1. La etiqueta `v1.17.0` de PX4 no existe; se usa la rama `release/1.17`.
2. El sensor de la cámara se llama `imager`, no `camera` (`puentes.sh`).
3. `VehicleStatus` es versionado: el tópico es `/fmu/out/vehicle_status_v1` (`modo_neural.py`).
4. En `x500_mono_cam_down` la cámara está 10 cm arriba del centro y ve el propio vehículo (más de la
   mitad de la imagen); el diseño original supone 10 cm abajo. Aquí va 25 cm abajo.
5. `server.config` de PX4 carga `GstCameraSystem`, que acumula los cuadros de la cámara si el
   codificador se atora: 110 MB/s, 23 GB en minutos y el OOM killer. Aquí Gazebo arranca sin él.
6. `MC_NN_THRST_COEF` trabaja con revoluciones por segundo, no rpm, y faltaba `MC_NN_MIN_RPM`
   (por omisión 1000) y `THR_MDL_FAC`; los valores correctos están en `px4/4099_gz_x500_busqueda`
   y los verifica `test/test_mision.py`.
7. La masa del x500 completo es 2.114 kg, no 2.0.
8. `exportar_tflite.py` usaba `--salida` como nombre del símbolo en C; el módulo exige `control_net_tflite`.
9. Con `make px4_sitl_neural gz_x500_mono_cam_down`, PX4 marca sus mensajes con el reloj de pared, no
   con el de Gazebo; el localizador estima el desfase con el mínimo de (llegada − marca).
