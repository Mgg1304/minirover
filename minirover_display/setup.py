from setuptools import setup, find_packages

package_name = 'minirover_display'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu@correo.com',
    description='Nodo ROS2 para mostrar datos del Minirover (GPS, brújula, velocidad, cámara).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'display_node = minirover_display.display_node:main'
        ],
    },
)
