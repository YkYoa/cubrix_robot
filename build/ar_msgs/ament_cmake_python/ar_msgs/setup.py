from setuptools import find_packages
from setuptools import setup

setup(
    name='ar_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('ar_msgs', 'ar_msgs.*')),
)
