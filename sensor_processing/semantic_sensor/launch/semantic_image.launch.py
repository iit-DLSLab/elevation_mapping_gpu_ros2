from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    sensor_name = LaunchConfiguration("sensor_name")
    config_path = LaunchConfiguration("config_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="front_cam"),
            DeclareLaunchArgument("sensor_name", default_value="front_cam_image"),
            DeclareLaunchArgument(
                "config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("semantic_sensor"), "config", "sensor_parameter.yaml"]
                ),
            ),
            Node(
                package="semantic_sensor",
                executable="image_node",
                namespace=namespace,
                name="semantic_image_node",
                output="screen",
                parameters=[{"sensor_name": sensor_name, "config_path": config_path}],
            ),
        ]
    )
