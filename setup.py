from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dvl_a50_serial_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'crcmod'],
    zip_safe=True,
    maintainer='sohaib',
    maintainer_email='kaidalisohaib@gmail.com',
    description='Python serial driver for the Water Linked DVL A50',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dvl_python_node = dvl_a50_serial_python.dvl_python_node:main'
        ],
    },
)
