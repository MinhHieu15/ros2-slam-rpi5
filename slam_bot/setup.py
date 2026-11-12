from glob import glob

from setuptools import setup

package_name = 'slam_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pham Minh Hieu',
    maintainer_email='phamminhhieuu0906@gmail.com',
    description='ROS 2 SLAM simulation and Raspberry Pi 5 bringup',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wander = slam_bot.wander:main',
        ],
    },
)
