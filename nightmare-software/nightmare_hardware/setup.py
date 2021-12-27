# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=find_packages(
        where='src',
        include=['*']
    ),
    package_dir={'': 'src'},
)

setup(**setup_args)
