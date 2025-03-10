from setuptools import find_packages, setup

setup(
    name='calibration_toolbox',
    version='0.0.0',
    packages=find_packages('calibration_toolbox'),
    package_dir={'': 'calibration_toolbox'},
    install_requires=['setuptools'],
)
