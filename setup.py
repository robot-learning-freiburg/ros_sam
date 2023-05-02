#!/usr/bin/env python

# Try catkin install
from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup

from pathlib import Path


with open(f'{Path(__file__).parent}/requirements.txt', 'r') as f:
   pip_dependencies = f.readlines()


d = generate_distutils_setup(
   packages=['ros_sam'],
   package_dir={'': 'src'}
)

d.update({'install_requires': pip_dependencies})

setup(**d)
