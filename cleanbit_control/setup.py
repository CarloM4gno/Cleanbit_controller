#!/usr/bin/env python3

from setuptools import setup

package_name = 'cleanbit_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='Federico Giuliana',
    maintainer_email='federico.giuliana12@gmail.com',
    description='Controller for CleanBit',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arduino_bridge = cleanbit_control.arduino_bridge:main'
        ],
    },
)
