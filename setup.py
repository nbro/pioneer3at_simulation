#!/usr/bin/env python
# -*- coding: utf-8 -*-


from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup

setup_args = generate_distutils_setup(
    packages=['pioneer3at_simulation'],
    package_dir={'': 'src'})

setup(**setup_args)
