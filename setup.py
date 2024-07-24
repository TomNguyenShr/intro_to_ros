import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'intro_to_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nhan',
    maintainer_email='tomnguyen.shr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'MotorControl = intro_to_ros.MotorControl:main',
        'subscriber = intro_to_ros.subscriber:main',
        'Dancing = intro_to_ros.Dancing:main',
        'pressure_sens = intro_to_ros.pressure_sens:main',
        'sub = intro_to_ros.sub:main',
        'ROVV = intro_to_ros.ROVV:main',
        'depth_publisher = intro_to_ros.depth_publisher:main',
        'move_rov = intro_to_ros.move_rov:main'],
    },
)
