from setuptools import find_packages
from setuptools import setup

setup(
    name='quadruped',
    version='0.0.0',
    packages=find_packages(
        include=('quadruped', 'quadruped.*')),
)
