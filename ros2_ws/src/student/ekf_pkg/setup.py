from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ekf_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhilash Somayajula',
    maintainer_email='abhilash@iitm.ac.in',
    description='EKF Node for Vessel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'ekf_node = ekf_pkg.ekf_node:main',
        'guidance_node = ekf_pkg.guidance:main',
        'control_node = ekf_pkg.control:main',
        ],
    },

)
