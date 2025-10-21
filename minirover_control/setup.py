from setuptools import setup

package_name = 'minirover_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu@correo.com',
    description='Nodo ROS2 para controlar Arduino con flechas del teclado',
    entry_points={
        'console_scripts': [
            'keyboard_node = minirover_control.keyboard_control_node:main',
        ],
    },
)
