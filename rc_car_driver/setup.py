import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rc_car_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taro',
    maintainer_email='taro@furo.org',
    description='Hardware-PWM low-level driver for a Tamiya RC car on Raspberry Pi.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_driver = rc_car_driver.vehicle_driver:main',
            'key_navigation = rc_car_driver.key_navigation:main',
        ],
    },
)
