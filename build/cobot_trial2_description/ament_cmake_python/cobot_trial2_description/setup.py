from setuptools import find_packages
from setuptools import setup

setup(
    name='cobot_trial2_description',
    version='0.0.1',
    packages=find_packages(
        include=('cobot_trial2_description', 'cobot_trial2_description.*')),
)
