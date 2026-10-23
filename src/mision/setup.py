import os
from glob import glob

from setuptools import setup

paquete = 'mision'


def arbol(carpeta):
    """data_files para copiar una carpeta completa (modelos con mallas y texturas)."""
    salida = []
    for raiz, _, archivos in os.walk(carpeta):
        if archivos:
            salida.append((os.path.join('share', paquete, raiz), [os.path.join(raiz, a) for a in archivos]))
    return salida


setup(
    name=paquete,
    version='0.1.0',
    packages=[paquete],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + paquete]),
        ('share/' + paquete, ['package.xml']),
        ('share/' + paquete + '/launch', glob('launch/*.py')),
        ('share/' + paquete + '/config', glob('config/*.*')),
        ('share/' + paquete + '/config/ardupilot', glob('config/ardupilot/*')),
        ('share/' + paquete + '/scripts', [f for f in glob('scripts/*') if os.path.isfile(f)]),
        ('share/' + paquete + '/yolo', glob('yolo/*.pt')),
        ('share/' + paquete + '/codigo', glob('codigo/*.npz')),
    ] + arbol('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jairo Olguín Roque',
    maintainer_email='jairoolguin1@gmail.com',
    description='Búsqueda de personas en bosque con PX4, Gazebo y ROS 2',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'detector_node = mision.detector_node:main',
        'localizador_node = mision.localizador_node:main',
        'mision_node = mision.mision_node:main',
        'modo_neural.py = mision.modo_neural:main',
        'modo_neural = mision.modo_neural:main',
        'prueba_neural = mision.prueba_neural:main',
        'mision_ardupilot = mision.mision_ardupilot:main',
        'controlador_red = mision.controlador_red:main',
    ]},
)
