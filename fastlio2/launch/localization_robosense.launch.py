import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "localization.rviz"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio_localizer_robosense.yaml"]
    )

    saved_pose_file_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "saved_pose.txt"]
    )


    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="localization_node",
                # name="localization_node",
                output="screen",
                # prefix=['xterm -e gdb -ex run --args'],
                parameters=[{"config_path": config_path.perform(launch.LaunchContext()),
                             "saved_pose_file_path": saved_pose_file_path.perform(launch.LaunchContext())}],
                remappings=[
                    ('/localization/custom_lidar_frec_pose', '/localization/custom_pose'),
                    ('/localization/lidar_frec_pose', '/localization/standard_pose'),
                ]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="fastlio2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            ),
        ]
    )
