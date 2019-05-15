# Please see http://docs.ros.org/kinetic/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['patroller'],
    scripts=['scripts'],
    package_dir={'': 'src'}
)

setup(**d)
