import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roboracer_stack'

setup(
    name=package_name,
    version='0.2.0',
    # find_packages, not [package_name]: the code is split into subsystem
    # submodules (common, localization, control) and a bare list would install
    # only the top-level __init__.py.
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.xml')),
        # The track, baked in. common/frames.py finds all of these through the
        # ament index, so nothing depends on where the repo was cloned.
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        # The racing line, the centreline the recovery orders checkpoints
        # against, and the segmentation the localizer keys its gates off.
        # Generated on the development branch; this branch ships the finished
        # artefacts only.
        (os.path.join('share', package_name, 'raceline'), glob('raceline/*.csv')),
        # nodelay.c is source, not data: the Dockerfile compiles it into this
        # same directory, where bridge.launch.py looks for the .so.
        (os.path.join('share', package_name, 'tools'), glob('tools/*.c')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Usman',
    maintainer_email='muawan2001@gmail.com',
    description='Localization, planning and control for the AutoDRIVE RoboRacer',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dead_reckoning = roboracer_stack.localization.dead_reckoning:main',
            'localization_bootstrap = roboracer_stack.localization.bootstrap:main',
            'localization_v2 = roboracer_stack.localization.localization_v2:main',
            'pure_pursuit = roboracer_stack.control.pure_pursuit:main',
        ],
    },
)
