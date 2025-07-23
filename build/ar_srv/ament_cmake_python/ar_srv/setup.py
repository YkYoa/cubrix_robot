from setuptools import find_packages
from setuptools import setup

setup(
    name='ar_srv',
    version='0.0.0',
    packages=find_packages(
        include=('ar_srv', 'ar_srv.*')),
)
