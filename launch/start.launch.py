import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 配置文件路径
    rviz_config = get_package_share_directory('rslidar_sdk') + '/rviz/rviz2.rviz'
    config_file = ''  # 替换为实际配置文件路径

    # 声明节点（先声明节点配置）
    rslidar_sdk_node = Node(
        namespace='rslidar_sdk',
        package='rslidar_sdk', 
        executable='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_path': config_file}]
    )

    rviz_node = Node(
        namespace='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', '/rslidar_points'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'laser_link',
            'transform_tolerance': 0.01,
            'min_height': 0.2,
            'max_height': 2.0,
            'angle_min': -3.1415926,
            'angle_max': 3.1415926,
            'angle_increment': 0.003,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0.3', '0', '0', '0', '1', 'base_link', 'laser_link']
    )
    static_transform_publisher_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link']
    )

    # 0
    # 启动 Cartographer 生成地图的节点
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        arguments=["-resolution", "0.05"],
    )
    

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
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

    # 返回 LaunchDescription，并在最后调用这些节点
    return LaunchDescription([
        rslidar_sdk_node,
        pointcloud_to_laserscan_node,
        static_transform_publisher_node,
        static_transform_publisher_imu_node,
        rviz_node, # 0

        # cartographer_node,#0
        # occupancy_grid_node,#0
    ])

