## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['nightmare'],
    package_dir={'': '/home/nik/catkin_ws/src'},
    requires=['roscpp', 'rospy', 'std_msgs', 'tf', 'robot_state_publisher']
)

setup(**setup_args)