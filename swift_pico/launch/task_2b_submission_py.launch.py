import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('rotors_swift_gazebo')
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
        
    roll_pitch_yawrate_thrust_controller = Node(
        package='rotors_control',
        namespace='rotors',
        executable='roll_pitch_yawrate_thrust_controller_node',
        name='roll_pitch_yawrate_thrust_controller',
    )

    swift_interface = Node(
        package='rotors_swift_interface',
        namespace='rotors',
        executable='rotors_swift_interface',
        name='rotors_swift_interface'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'swift_pico_bridge.yaml'),
        }],
        output='screen'
    )

    whycon = Node(
            package='whycon',
            name='whycon',
            namespace='whycon',
            executable='whycon',
            output='screen',
            parameters=[{
                'targets': 1,
                'name': 'whycon',
                'outer_diameter': 0.38,
                'inner_diameter': 0.14,
            }],
            remappings=[
                ('image_raw', '/camera/image_raw')
            ]
        )

    image_view = Node(
            package='image_view',
            executable='image_view',
            namespace='whycon_display',
            name='image_view',
            output='screen',
            remappings=[
                ('image', '/whycon/image_out')
            ]
        )

    planner_server = Node(
        package='swift_pico',
        executable='planner_server',
        name='planner_server'

    )

    pico_server = Node(
        package='swift_pico',
        executable='pico_server_2b.py',
        name='pico_server_2b'

    )
    pico_cilent = Node(
        package='swift_pico',
        executable='pico_client_2b.py',
        name='pico_client_2b'

    )
    path_planning_service = Node(
        package='swift_pico',
        executable='path_planning_service.py',
        name='path_planning_service'

    )

    rosbag = launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'task_2b', '/whycon/poses','/random_points','/rosin'],
            output='screen'
        )

    return LaunchDescription([
        gz_sim,
        bridge,
        roll_pitch_yawrate_thrust_controller,
        swift_interface,
        whycon,
        image_view,
        planner_server,
        pico_server, 
        path_planning_service,
        pico_cilent,
        rosbag
    ])
