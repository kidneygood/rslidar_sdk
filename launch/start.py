from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config = get_package_share_directory("rslidar_sdk") + "/rviz/rviz2.rviz"

    config_file = ""  # your config file path

    rsildar_sdk_node = Node(
        namespace="rslidar_sdk",
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        output="screen",
        parameters=[{"config_path": config_file}],
    )
    rviz2_node = Node(
        namespace="rviz2",
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/rslidar_points"), ("scan", "/rslidar_scan")],
        parameters=[
            {
                "target_frame": "rslidar",
                "transform_tolerance": 0.01,
                "min_height": -0.4,
                "max_height": 1.0,
                "angle_min": -3.1415926,
                "angle_max": 3.1415926,
                "angle_increment": 0.003,
                "scan_time": 0.1,
                "range_min": 0.2,
                "range_max": 100.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        name="pointcloud_to_laserscan",
    )

    return LaunchDescription(
        [
            rsildar_sdk_node,
            rviz2_node,
            pointcloud_to_laserscan_node,
        ]
    )
