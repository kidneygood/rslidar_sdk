from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 直接定义常量
pi = 3.141592653589793  # 定义圆周率

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
                "target_frame": "laser_link",      # 目标坐标系
                "transform_tolerance": 0.01,       # 坐标变换容差
                "min_height": 0.2,                # 点云转换为激光扫描的最小高度
                "max_height": 2.0,                # 点云转换为激光扫描的最大高度
                "angle_min": -pi,                 # 扫描角度最小值
                "angle_max": pi,                  # 扫描角度最大值
                "angle_increment": 0.2/180*pi,    # 角度增量
                "scan_time": 0.1,                 # 扫描时间
                "range_min": 0.2,                 # 最小测量距离
                "range_max": 200.0,               # 最大测量距离
                "use_inf": False,                 # 是否使用无穷大值
                "inf_epsilon": 0.5,               # 无穷大值的阈值，`use_inf=False`时实际上不起作用
            }
        ],
        name="pointcloud_to_laserscan",
    )

    return LaunchDescription(
        [
            rsildar_sdk_node,            
            pointcloud_to_laserscan_node,
            rviz2_node,
        ]
    )
