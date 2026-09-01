from glob import glob

from setuptools import setup

package_name = 'racer_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdullah Bin Naeem',
    maintainer_email='abdullahbinnaeempro@gmail.com',
    description='Dead reckoning plus AMCL / slam_toolbox localization for the RoboRacer',
    license='BSD',
    entry_points={
        'console_scripts': [
            'dead_reckoning = racer_localization.dead_reckoning:main',
            'localization_bootstrap = racer_localization.localization_bootstrap:main',
            'localization_error = racer_localization.localization_error:main',
        ],
    },
)
