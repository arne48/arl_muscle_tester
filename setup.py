#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['arl_muscle_tester'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_arl_muscle_tester']
)

setup(**d)
