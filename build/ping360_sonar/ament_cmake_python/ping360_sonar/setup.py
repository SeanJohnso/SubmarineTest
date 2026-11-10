from setuptools import find_packages
from setuptools import setup

setup(
    name='ping360_sonar',
    version='2.0.1',
    packages=find_packages(
        include=('ping360_sonar', 'ping360_sonar.*')),
)
