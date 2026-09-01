from glob import glob

from setuptools import setup

package_name = 'racer_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdullah Bin Naeem',
    maintainer_email='abdullahbinnaeempro@gmail.com',
    description='Composition root: bridge + chassis + localizer + follower',
    license='BSD',
    entry_points={},
)
