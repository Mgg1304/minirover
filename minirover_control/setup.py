from setuptools import setup, find_packages

package_name = 'minirover_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=[
        'setuptools',
        'pynput'
    ],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu@correo.com',
    description='Nodo ROS2 para controlar el MiniRover mediante teclado.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_control_node = minirover_control.keyboard_control_node:main'
        ],
    },
)