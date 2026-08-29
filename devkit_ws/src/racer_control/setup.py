from glob import glob

from setuptools import setup

package_name = 'racer_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz') + glob('config/*.xml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdullah Bin Naeem',
    maintainer_email='abdullahbinnaeempro@gmail.com',
    description='Pure pursuit path follower for the AutoDRIVE RoboRacer',
    license='BSD',
    entry_points={
        'console_scripts': [
            'pure_pursuit = racer_control.pure_pursuit:main',
            'dead_reckoning = racer_control.dead_reckoning:main',
            'calibrate_steering = racer_control.calibrate_steering:main',
            'map_publisher = racer_control.map_publisher:main',
            'localization_error = racer_control.localization_error:main',
            'localization_bootstrap = racer_control.localization_bootstrap:main',
        ],
    },
)
