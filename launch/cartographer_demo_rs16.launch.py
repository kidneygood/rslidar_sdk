import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = "rslidar_sdk"
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(
        package_name
    )
    # 配置文件路径
    rviz_config = get_package_share_directory("rslidar_sdk") + "/rviz/test.rviz"

    config_file = ""  # 替换为实际配置文件路径

    # 激光雷达驱动节点
    rslidar_sdk_node = Node(
        namespace="rslidar_sdk",
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        output="screen",
        parameters=[{"config_path": config_file}],
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            os.path.join(
                get_package_share_directory("cartographer_ros"), "configuration_files"
            ),
            "-configuration_basename",
            "rs16_lidar.lua",
        ],
        remappings=[
            ("/scan", "/scan"),
            ("/imu", "/IMU_data"),
        ],  # 点云扫描话题重映射 ('/odom', '/odometry/filtered')
    )
    #  启动 static_transform_publisher
    # static_transform_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_to_laser',
    #     arguments=['0.0', '0.0', '0.45', '0.0', '0.0', '0.0', 'map', 'odom']
    # )
    static_transform_publisher_base_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser",
        arguments=["0", "0", "0.3", "0", "0", "0", "1", "base_link", "laser_link"],
    )
    static_transform_publisher_imu_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "imu_link"],
    )
    # 启动 Cartographer 生成地图的节点
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        arguments=["-resolution", "0.05"],
    )

    rviz_node = Node(
        namespace="rviz2",
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    # 点云转换为激光雷达扫描节点
    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan")],
        parameters=[
            {
                "target_frame": "laser_link",
                "transform_tolerance": 0.01,
                "min_height": 0.2,
                "max_height": 2.0,
                "angle_min": -3.1415926,
                "angle_max": 3.1415926,
                "angle_increment": 0.003,
                "scan_time": 0.1,
                "range_min": 0.2,
                "range_max": 200.0,
                "use_inf": False,
                "inf_epsilon": 0.5,
            }
        ],
        name="pointcloud_to_laserscan",
    )

    # 返回 LaunchDescription，并在最后调用这些节点
    return LaunchDescription(
        [
            # rslidar_sdk_node,
            rviz_node,
            # pointcloud_to_laserscan_node,
            # static_transform_publisher_base_node,
            # static_transform_publisher_imu_node,
            # robot_localization_node,
            # static_transform_publisher,
            cartographer_node,
            occupancy_grid_node,
        ]
    )
