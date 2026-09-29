from setuptools import find_packages, setup

package_name = 'my_robot_moveit_demo'

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
    maintainer='dyh',
    maintainer_email='DYH-hxyz@user.noreply.github.com',
    description='云台MoveIt控制demo，使用MoveGroupInterface',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yuntai_move_group_ctrl = scripts.yuntai_move_group_ctrl:main',
        ],
    },
)
