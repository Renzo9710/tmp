## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mhp_planner'],
    scripts=['scripts/time_opt_erg_plan.py'],
    package_dir={'': 'src'}
)

setup(**d)