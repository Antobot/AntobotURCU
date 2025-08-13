from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'antobot_ekf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install the params directory
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='So-Young Kim',
    maintainer_email='soyoung.kim@antobot.ai',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
