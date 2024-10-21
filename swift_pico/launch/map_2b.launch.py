import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('swift_pico')
    pkg_project_gazebo = get_package_share_directory('swift_pico')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'warehouse_world.sdf -r'
        ])}.items(),
    )
    

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'map_bridge.yaml'),
        }],
        output='screen'
    )

    image_view = Node(
            package='image_view',
            executable='image_view',
            namespace='arena_display',
            name='image_view',
            output='screen',
            remappings=[
                ('image', '/image_raw')
            ]
        )
    
    return LaunchDescription([
        gz_sim,
        bridge,
        image_view
    ])