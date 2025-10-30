from setuptools import find_packages
from setuptools import setup

setup(
    name='minirover_control',
    version='0.1.0',
    packages=find_packages(
        include=('minirover_control', 'minirover_control.*')),
)
