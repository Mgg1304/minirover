from setuptools import setup, find_packages

package_name = 'minirover_sensores'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gps_node = minirover_sensores.gps_node:main'
        ],
    },
)