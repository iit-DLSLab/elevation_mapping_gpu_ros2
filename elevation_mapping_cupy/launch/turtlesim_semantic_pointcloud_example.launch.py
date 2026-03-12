from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    elevation_mapping_cupy_dir = get_package_share_directory('elevation_mapping_cupy')
    semantic_sensor_dir = get_package_share_directory('semantic_sensor')

    return LaunchDescription([
        # Include the turtlesim_init launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    elevation_mapping_cupy_dir,
                    'launch',
                    'turtlesim_init.launch.py'
                ])
            ),
            launch_arguments={
                'rviz_config': PathJoinSubstitution([
                    elevation_mapping_cupy_dir,
                    'rviz',
                    'turtle_semantic_example.rviz'
                ])
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    semantic_sensor_dir,
                    'launch',
                    'semantic_pointcloud.launch.py'
                ])
            ),
            launch_arguments={
                'namespace': 'front_cam',
                'sensor_name': 'front_cam_pointcloud',
                'config_path': PathJoinSubstitution([
                    semantic_sensor_dir,
                    'config',
                    'sensor_parameter.yaml'
                ]),
            }.items()
        ),

        # Elevation Mapping Node
        Node(
            package='elevation_mapping_cupy',
            executable='elevation_mapping_node.py',
            name='elevation_mapping_node',
            parameters=[
                PathJoinSubstitution([
                    elevation_mapping_cupy_dir,
                    'config',
                    'core',
                    'core_param.yaml'
                ]),
                PathJoinSubstitution([
                    elevation_mapping_cupy_dir,
                    'config',
                    'setups',
                    'turtle_bot',
                    'turtle_bot_semantics_pointcloud.yaml'
                ])
            ],
            output='screen'
        )
    ])
