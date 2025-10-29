from setuptools import find_packages, setup

package_name = 'antobot_urcu'

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
    maintainer='Zhuang Zhou',
    maintainer_email='zhuang.zhou@antobot.ai',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'softshutdown = antobot_urcu.softShutdown:main',
            'urcuMonitor = antobot_urcu.urcuMonitor:main'
        ],
    },
)