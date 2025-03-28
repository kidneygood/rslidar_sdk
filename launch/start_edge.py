from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_file = ""  # your config file path

    rsildar_sdk_node = Node(
        namespace="rslidar_sdk",
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        output="screen",
        parameters=[{"config_path": config_file}],
    )

    static_transform_publisher_laser_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser",
        arguments=["0", "0", "0.3", "0", "0", "0", "1", "base_link", "laser_link"],
    )

    return LaunchDescription(
        [
            rsildar_sdk_node,            
            static_transform_publisher_laser_node,
        ]
    )
