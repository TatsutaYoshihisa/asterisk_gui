from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['asterisk_gui'],
    package_dir={'': 'src'},
    scripts=[
        'scripts/leg_controller_gui.py',
        'scripts/hexapod_controller_gui.py'
    ]
)

setup(**d)