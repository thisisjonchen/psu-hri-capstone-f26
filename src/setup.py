#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=['localization'], package_dir={'': 'scripts'},
    package_data={'localization': ['anchors.json']}))
