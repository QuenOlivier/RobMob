#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['util'],
    package_dir={'':'include'},
    requires=['nav_msgs', 'rospy', 'geometry_msgs','rospy'],
)


setup(**setup_args)
