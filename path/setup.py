#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['path'],
    package_dir={'':'include'},
    py_modules = ['path.finding', 'path.retrieve_map', 'path.follow'],
    requires=['nav_msgs', 'geometry_msgs','rospy'],
)


setup(**setup_args)
