#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
exportar_tflite.py -- Convierte los pesos de la red 15-64-32-4 (pesos_nn.npz,
producidos por nn_imitacion.py o por ppo_refinamiento.py) al formato de
TensorFlow Lite en punto flotante de 32 bits, comprueba que el modelo use
sólo los operadores del módulo mc_nn_control (FullyConnected, ReLU, Add),
verifica que reproduce las salidas de numpy y genera el arreglo en C que se
copia en control_net.cpp / control_net.hpp de PX4 (Sección 4.8.4).

Uso: python3 exportar_tflite.py --pesos pesos_nn.npz --salida control_net
Requiere: numpy, tensorflow (>= 2.16)
"""
import argparse
import numpy as np
import tensorflow as tf


def cargar(ruta):
    z = np.load(ruta)
    W = [z[f"W{i}"] for i in range(3)]; b = [z[f"b{i}"] for i in range(3)]
    return W, b, z["scale"]


def construir(W, b, scale):
    """Red Keras equivalente; la normalización de la entrada se absorbe en la primera capa."""
    W0 = (scale[:, None] * W[0]).astype(np.float32)        # x_norm = x*scale => (x*scale) W0 = x (diag(scale) W0)
    ent = tf.keras.Input(shape=(15,), dtype=tf.float32, name="observacion")
    h = tf.keras.layers.Dense(64, activation="relu", name="oculta_1")(ent)
    h = tf.keras.layers.Dense(32, activation="relu", name="oculta_2")(h)
    sal = tf.keras.layers.Dense(4, activation=None, name="acciones")(h)
    modelo = tf.keras.Model(ent, sal)
    modelo.get_layer("oculta_1").set_weights([W0, b[0].astype(np.float32)])
    modelo.get_layer("oculta_2").set_weights([W[1].astype(np.float32), b[1].astype(np.float32)])
    modelo.get_layer("acciones").set_weights([W[2].astype(np.float32), b[2].astype(np.float32)])
    return modelo


def a_tflite(modelo):
    conv = tf.lite.TFLiteConverter.from_keras_model(modelo)
    conv.optimizations = []                                 # sin cuantización: float32
    conv.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]
    return conv.convert()


def operadores(tflite_bytes):
    """Lista de operadores del modelo (deben ser sólo FULLY_CONNECTED, RELU y ADD)."""
    interp = tf.lite.Interpreter(model_content=tflite_bytes)
    return sorted({op["op_name"] for op in interp._get_ops_details()})


def verificar(tflite_bytes, W, b, scale, n=1000):
    """Compara la salida del intérprete de TFLite con la red de numpy para entradas aleatorias."""
    rng = np.random.default_rng(0)
    X = rng.normal(0, 1, (n, 15)).astype(np.float32)
    interp = tf.lite.Interpreter(model_content=tflite_bytes); interp.allocate_tensors()
    i_in = interp.get_input_details()[0]["index"]; i_out = interp.get_output_details()[0]["index"]
    y_tfl = np.zeros((n, 4), np.float32)
    for k in range(n):
        interp.set_tensor(i_in, X[k:k+1]); interp.invoke(); y_tfl[k] = interp.get_tensor(i_out)[0]
    h = np.maximum((X*scale) @ W[0] + b[0], 0); h = np.maximum(h @ W[1] + b[1], 0); y_np = h @ W[2] + b[2]
    return float(np.abs(y_tfl - y_np).max())


def a_c(tflite_bytes, nombre):
    """Equivalente de `xxd -i modelo.tflite`, con el formato de control_net.cpp."""
    lineas = [f"// Generado por exportar_tflite.py: red 15-64-32-4, float32, {len(tflite_bytes)} bytes",
              "#include <cstdint>", "#include \"control_net.hpp\"", "",
              f"alignas(16) const unsigned char {nombre}_tflite[] = {{"]
    for k in range(0, len(tflite_bytes), 12):
        lineas.append("  " + ", ".join(f"0x{c:02x}" for c in tflite_bytes[k:k+12]) + ",")
    lineas.append("};")
    cpp = "\n".join(lineas) + "\n"
    hpp = (f"#pragma once\n#include <cstdint>\n\nconstexpr unsigned int {nombre}_tflite_size = {len(tflite_bytes)};\n"
           f"extern const unsigned char {nombre}_tflite[];\n")
    return cpp, hpp


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--pesos", default="pesos_nn.npz")
    ap.add_argument("--salida", default="control_net")
    a = ap.parse_args()
    W, b, scale = cargar(a.pesos)
    modelo = construir(W, b, scale)
    tfl = a_tflite(modelo)
    ops = operadores(tfl)
    print("Operadores del modelo:", ops)
    assert set(ops) <= {"FULLY_CONNECTED", "RELU", "ADD"}, "operador no soportado por mc_nn_control"
    err = verificar(tfl, W, b, scale)
    print(f"Diferencia máxima TFLite vs numpy: {err:.2e} ({'OK' if err < 1e-5 else 'REVISAR'})")
    with open(a.salida + ".tflite", "wb") as fh:
        fh.write(tfl)
    # El módulo busca los símbolos control_net_tflite y control_net_tflite_size, sin
    # importar cómo se llamen los archivos de salida.
    cpp, hpp = a_c(tfl, "control_net")
    with open(a.salida + ".cpp", "w") as fh: fh.write(cpp)
    with open(a.salida + ".hpp", "w") as fh: fh.write(hpp)
    print(f"Modelo de {len(tfl)} bytes escrito en {a.salida}.tflite, {a.salida}.cpp y {a.salida}.hpp")
    print("Copiar los dos últimos a PX4-Autopilot/src/modules/mc_nn_control/ y recompilar: make px4_sitl_neural")
