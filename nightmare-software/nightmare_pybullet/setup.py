#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['nightmare_pybullet'],
    package_dir={'nightmare_pybullet': 'ros/src/nightmare_pybullet'}
)

setup(**d)
