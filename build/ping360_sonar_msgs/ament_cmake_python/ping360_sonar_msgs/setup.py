from setuptools import find_packages
from setuptools import setup

setup(
    name='ping360_sonar_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('ping360_sonar_msgs', 'ping360_sonar_msgs.*')),
)
