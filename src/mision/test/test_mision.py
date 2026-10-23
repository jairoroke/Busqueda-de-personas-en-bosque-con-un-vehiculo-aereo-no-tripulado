"""Pruebas de la geometría, las métricas y el mapeo de la red en PX4.

    cd ~/ros2_ws/src/mision && python3 -m pytest -q test/
"""
import math
import os
import re
import sys

import numpy as np
import pytest

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
sys.path.insert(0, os.path.join(RAIZ, 'scripts'))

from mision.geometria import T_BC, covarianza, euler_de_R, localizar, proyectar, R_euler  # noqa: E402
from mision.metricas import distancia_a_ruta, evaluar_candidatas  # noqa: E402

F = 640 / math.tan(0.87)
K = np.array([[F, 0, 640], [0, F, 480], [0, 0, 1.0]])
K_INV = np.linalg.inv(K)


def test_localizar_invierte_proyectar():
    rng = np.random.default_rng(0)
    for _ in range(200):
        xi = np.array([rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(15, 45)])
        eta = np.array([rng.uniform(-np.pi, np.pi), rng.normal(0, 0.1), rng.normal(0, 0.1)])
        X = xi + np.array([rng.uniform(-10, 10), rng.uniform(-10, 10), 0.0]); X[2] = 0.0
        uv = proyectar(X, xi, eta, K)
        assert uv is not None
        np.testing.assert_allclose(localizar(uv, xi, eta, K_INV, 0.0), X, atol=1e-6)


def test_ejes_de_la_camara():
    """Con el vehículo nivelado mirando al este (psi = 0), un punto al frente sale arriba en la imagen y uno a
    la izquierda sale a la izquierda (derecha de la imagen = -A_y, abajo = -A_x)."""
    xi = np.array([0.0, 0.0, 30.0]); eta = np.zeros(3)
    u_frente, v_frente = proyectar([5.0, 0.0, 0.0], xi, eta, K)
    u_izq, v_izq = proyectar([0.0, 5.0, 0.0], xi, eta, K)
    assert v_frente < 480 and abs(u_frente - 640) < 1e-6
    assert u_izq < 640 and abs(v_izq - 480) < 1e-6


def test_euler_ida_y_vuelta():
    for eta in ([0.3, 0.1, -0.2], [-2.5, -0.4, 0.3]):
        np.testing.assert_allclose(euler_de_R(R_euler(*eta)), eta, atol=1e-12)


def test_covarianza_crece_con_la_altura():
    Sp = np.diag([0.25, 0.25, 1.0]); Se = np.eye(3) * np.deg2rad(0.5)**2; Su = np.eye(2) * 25
    uv = np.array([640 + 256, 480 + 192])
    tr = [np.trace(covarianza(uv, np.array([0, 0, h]), np.zeros(3), K_INV, 0.0, Sp, Se, Su)[0]) for h in (20, 30, 40)]
    assert tr[0] < tr[1] < tr[2]


def test_camara_del_vehiculo_coincide_con_la_geometria():
    """T_BC del localizador = pose de la cámara en models/x500_busqueda (generar_vehiculo.py)."""
    ruta = os.path.join(RAIZ, 'models', 'x500_busqueda', 'model.sdf')
    if not os.path.exists(ruta):
        pytest.skip('falta generar el vehículo (scripts/generar_vehiculo.py)')
    m = re.search(r'<uri>model://mono_cam</uri>\s*<pose>([-\d. ]+)</pose>', open(ruta).read())
    x, y, z, r, p, yaw = map(float, m.group(1).split())
    np.testing.assert_allclose([x, y, z], T_BC)
    assert abs(p - math.pi/2) < 1e-3 and r == 0 and yaw == 0


def test_metricas():
    personas = [{'x': 0, 'y': 0}, {'x': 20, 'y': 0}, {'x': 40, 'y': 40}]
    cand = [(1.0, 1.0, 0.9, 3), (19.0, 0.0, 0.7, 2), (21.0, 0.5, 0.95, 4), (80, 80, 0.9, 1), (40, 40, 0.2, 1)]
    r = evaluar_candidatas(cand, personas, umbral_conf=0.6, radio=5.0)
    assert r['personas_localizadas'] == 2
    assert r['falsas_alarmas'] == 2          # la segunda candidata cerca de (20, 0) y la de (80, 80)
    assert r['localizadas'] == [0, 1]
    assert distancia_a_ruta([5, 3, 30], [[0, 0, 30], [10, 0, 30]]) == pytest.approx(3.0)


def parametros_airframe():
    texto = open(os.path.join(RAIZ, 'px4', '4099_gz_x500_busqueda')).read()
    return {k: float(v) for k, v in re.findall(r'param set-default (\w+) ([-\d.]+)', texto)}


def test_mapeo_de_la_red_en_px4():
    """La cadena de PX4 (RescaleActions de mc_nn_control, THR_MDL_FAC y el ESC simulado de 150 a 1000 rad/s)
    debe dar la misma velocidad de rotor que el simulador de nn_imitacion.py: w = w_max sqrt((a + 1)/2)."""
    p = parametros_airframe()
    C, rmin, rmax, fac = p['MC_NN_THRST_COEF'] / 1e5, p['MC_NN_MIN_RPM'], p['MC_NN_MAX_RPM'], p['THR_MDL_FAC']
    wmax, ec_min, ec_max = 1000.0, 150.0, 1000.0
    for a in np.linspace(-0.85, 1.0, 40):
        # mc_nn_control.cpp, RescaleActions
        rpm = math.sqrt((a + 1.0) / C) * 60.0
        n = (rpm * 2.0 - rmax - rmin) / (rmax - rmin)
        x = (n + 1) / 2
        empuje = 0.8 * x * x + 0.2 * x
        # mixer de PX4 (FunctionMotors con THR_MDL_FAC) y ESC simulado
        u = -(1 - fac) / (2 * fac) + math.sqrt((1 - fac)**2 / (4 * fac * fac) + empuje / fac)
        w = ec_min + u * (ec_max - ec_min)
        assert w == pytest.approx(wmax * math.sqrt((a + 1) / 2), rel=2e-3)


def test_origen_gps_igual_al_del_mundo():
    """geo.ORIGEN (avisos de personas) debe ser el <spherical_coordinates> del mundo (GPS de PX4/ArduPilot)."""
    import re
    from mision.geo import ORIGEN, enu_a_geodesica
    texto = open(os.path.join(os.path.dirname(__file__), '..', 'scripts', 'generar_mundo.py')).read()
    lat = float(re.search(r'<latitude_deg>([-\d.]+)<', texto).group(1))
    lon = float(re.search(r'<longitude_deg>([-\d.]+)<', texto).group(1))
    alt = float(re.search(r'<elevation>([-\d.]+)<', texto).group(1))
    assert ORIGEN == (lat, lon, alt)
    la, lo, h = enu_a_geodesica(0.0, 1000.0, 0.0)                 # 1 km al norte
    assert abs((la - lat) * 110_780 - 1000.0) < 3.0 and abs(lo - lon) < 1e-9
