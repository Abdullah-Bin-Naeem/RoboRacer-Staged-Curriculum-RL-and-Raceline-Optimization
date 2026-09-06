from setuptools import setup

package_name = 'racer_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # CycloneDDS config lives here because every package's terminals need it:
        #   export CYCLONEDDS_URI=file://$(ros2 pkg prefix racer_common)/share/racer_common/config/cyclonedds.xml
        ('share/' + package_name + '/config', ['config/cyclonedds.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdullah Bin Naeem',
    maintainer_email='abdullahbinnaeempro@gmail.com',
    description='Shared frames, paths and restricted-topic list for the RoboRacer stack',
    license='BSD',
    entry_points={},
)
