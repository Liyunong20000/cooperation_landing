"""Catkin installation metadata; build this package with Catkin."""

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=[
            'cooperation_landing',
            'cooperation_landing.gripper',
            'cooperation_landing.simulation',
        ],
        package_dir={'': 'src'},
    )
)
