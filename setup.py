## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rospy_crazyflie',
              'rospy_crazyflie.client',
              'rospy_crazyflie.server',
              'rospy_crazyflie.motion_commands'],
    package_dir={'': 'src'},
)

setup(**setup_args)
