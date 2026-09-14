from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup(**generate_distutils_setup(
    packages=['object_detection_core'],
    package_dir={'': 'src'},
))
