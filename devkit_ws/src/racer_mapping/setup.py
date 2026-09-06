from glob import glob

from setuptools import setup

package_name = 'racer_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # The Porto track, baked in. frames.MAPS_DIR finds this through the
        # ament index, so nothing depends on where the repo was cloned.
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdullah Bin Naeem',
    maintainer_email='abdullahbinnaeempro@gmail.com',
    description='SLAM mapping bringup for the AutoDRIVE RoboRacer',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Moved from racer_control: serving a .pgm is a mapping concern, and
            # this package already owns maps/.
            'map_publisher = racer_mapping.map_publisher:main',
        ],
    },
)
