from glob import glob

from setuptools import find_packages, setup

package_name = 'my_robot_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dyh',
    maintainer_email='DYH-hxyz@user.noreply.github.com',
    description='云台视觉识别与目标追踪',
    license='BSD',
    entry_points={
        'console_scripts': [
            'target_detector = my_robot_vision.target_detector:main',
            'target_tracker = my_robot_vision.target_tracker:main',
        ],
    },
)
