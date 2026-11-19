import os
import re
import tempfile
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    package_name = 'elevation_mapping_cupy'
    share_dir = get_package_share_directory(package_name)


    # Define paths
    core_param_path = os.path.join(share_dir, 'config', 'core', 'core_param.yaml')

    # Declare launch arguments
    robot_param_arg = DeclareLaunchArgument(
        'robot_config',
        # default_value='turtle_bot/turtle_bot_simple.yaml',
        default_value='menzi/base.yaml',
        description='Name of the robot-specific config file within config/setups/'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([share_dir, 'rviz', 'elevation_mapping.rviz']),
        # 'true',
        description='Path to the RViz config file'
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='',
        description='Input PointCloud2 topic(s), comma-separated (e.g. /front_down/depth/points,/front_up/depth/points). '
                    'If set, it REPLACES all subscribers of robot_config. Empty = use the subscribers of robot_config.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Get launch configurations
    robot_config = LaunchConfiguration('robot_config')
    robot_param_path = PathJoinSubstitution([share_dir, 'config', 'setups', robot_config])
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Verify core config exists
    if not os.path.exists(core_param_path):
        raise FileNotFoundError(f"Config file {core_param_path} does not exist")

    # Define nodes
    def make_elevation_mapping_node(context):
        robot_path = robot_param_path.perform(context)
        topics = [t.strip() for t in LaunchConfiguration('pointcloud_topic').perform(context).split(',') if t.strip()]
        extra = {}
        if topics:
            # Parameters loaded from a YAML cannot be removed afterwards: write a copy of robot_config
            # without its subscribers, then declare one subscriber per topic.
            with open(robot_path) as f:
                robot_cfg = yaml.safe_load(f)
            for node_cfg in robot_cfg.values():
                node_cfg.get('ros__parameters', {}).pop('subscribers', None)
            tmp = tempfile.NamedTemporaryFile('w', prefix='elevation_robot_', suffix='.yaml', delete=False)
            yaml.safe_dump(robot_cfg, tmp)
            tmp.close()
            robot_path = tmp.name
            for i, t in enumerate(topics):
                # Sensor name from the first topic segment (/front_down/depth/points -> front_down)
                name = re.sub(r'\W', '_', t.strip('/').split('/')[0]) or f'sensor_{i}'
                if f'subscribers.{name}.topic_name' in extra:
                    name = f'{name}_{i}'
                extra[f'subscribers.{name}.topic_name'] = t
                extra[f'subscribers.{name}.data_type'] = 'pointcloud'
        params = [core_param_path, robot_path, {'use_sim_time': use_sim_time}]
        if extra:
            params.append(extra)
        return [Node(
            package=package_name,
            executable='elevation_mapping_node',
            name='elevation_mapping_node',
            output='screen',
            parameters=params,
        )]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(launch_rviz)
    )

    return LaunchDescription([
        robot_param_arg,
        launch_rviz_arg,
        rviz_config_arg,
        pointcloud_topic_arg,
        use_sim_time_arg,
        OpaqueFunction(function=make_elevation_mapping_node),
        rviz_node
    ])
