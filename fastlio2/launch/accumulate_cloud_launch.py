import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio_robosense.yaml"]
    )

    saved_cloud_path = "/home/xiangweizeng/my_project/fastlio2-ros2_ws/data/calib_cloud.pcd"


    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="accumulate_lidar_cloud",
                output="screen",
                parameters=[{"config_path": config_path.perform(launch.LaunchContext()),
                             "save_path": saved_cloud_path}],
            )
        ]
    )
