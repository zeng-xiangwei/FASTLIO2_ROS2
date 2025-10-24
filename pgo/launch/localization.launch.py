import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("pgo"), "rviz", "pgo.rviz"]
    )
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo_localization.yaml"]
    )

    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio_localizer_op.yaml"]
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
                # name="lio_node",
                output="screen",
                parameters=[{"config_path": lio_config_path.perform(launch.LaunchContext()),
                             "saved_pose_file_path": saved_pose_file_path.perform(launch.LaunchContext())}],
                remappings=[
                    ('/localization/custom_lidar_frec_pose', '/localization/custom_pose'),
                    ('/localization/lidar_frec_pose', '/localization/standard_pose'),
                ]
            ),

            launch_ros.actions.Node(
                package="pgo",
                namespace="pgo",
                executable="pgo_node",
                name="pgo_node",
                output="screen",
                # prefix=['xterm -e gdb -ex run --args'],
                parameters=[{"config_path": pgo_config_path.perform(launch.LaunchContext())}],
            ),

            launch_ros.actions.Node(
                package="rviz2",
                namespace="pgo",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext()),'--ros-args', '--log-level', 'WARN'],
            )
        ]
    )
