import os
import subprocess
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def build_world(urdf_xacro, world_template, output_dir):
    urdf_path = os.path.join(output_dir, 'my_robot.urdf')
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(xacro.process_file(urdf_xacro).toxml())

    sdf = subprocess.run(['gz', 'sdf', '-p', urdf_path], capture_output=True, text=True, check=True).stdout
    model = sdf[sdf.index('<model'):sdf.rindex('</model>') + len('</model>')]

    with open(world_template) as template_file:
        world = template_file.read()

    world_path = os.path.join(output_dir, 'camera_world.sdf')
    with open(world_path, 'w') as world_file:
        world_file.write(world.replace('<!-- ROBOT_MODEL -->', model))

    return urdf_path, world_path


def generate_launch_description():
    gazebo_share = get_package_share_directory('my_robot_gazebo')
    description_share = get_package_share_directory('my_robot_description')
    moveit_share = get_package_share_directory('my_robot_moveit_config')

    urdf_xacro = os.path.join(gazebo_share, 'urdf', 'my_robot_gazebo.urdf.xacro')
    world_template = os.path.join(gazebo_share, 'world', 'camera_world.sdf')

    output_dir = os.path.join(tempfile.gettempdir(), 'my_robot_gazebo')
    os.makedirs(output_dir, exist_ok=True)
    urdf_path, world_path = build_world(urdf_xacro, world_template, output_dir)

    moveit_config = (
        MoveItConfigsBuilder('my_robot', package_name='my_robot_moveit_config')
        .robot_description(file_path=urdf_xacro)
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + world_path}.items(),
    )

    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(description_share) + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)}],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        output='screen',
        arguments=['/camera/image'],
    )

    camera_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        output='screen',
        arguments=['/target/image_debug'],
    )

    virtual_joint_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, 'launch', 'static_virtual_joint_tfs.launch.py')),
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': True,
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
            },
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(moveit_share, 'config', 'moveit.rviz')],
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    spawn_controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[controller, '--service-call-timeout', '60.0'],
        )
        for controller in ('joint_state_broadcaster', 'yuntai_planning_groups_controller')
    ]

    return LaunchDescription([
        resource_path,
        gazebo,
        clock_bridge,
        robot_state_publisher,
        virtual_joint_tfs,
        move_group,
        rviz,
        TimerAction(period=5.0, actions=[camera_bridge, camera_view]),
        TimerAction(period=5.0, actions=spawn_controllers),
    ])
