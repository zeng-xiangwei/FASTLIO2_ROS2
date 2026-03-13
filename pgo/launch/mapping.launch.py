import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("pgo"), "rviz", "pgo_mapping.rviz"]
    )
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio_op.yaml"]
    )

    visualize_arg = DeclareLaunchArgument('visualize', default_value='true')


    return launch.LaunchDescription(
        [
            visualize_arg,
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                # name="lio_node",
                output="screen",
                parameters=[{"config_path": lio_config_path.perform(launch.LaunchContext())}],
                remappings=[
                    ('/localization/custom_imu_frec_pose', '/localization/custom_pose'),
                    ('/localization/imu_frec_pose', '/localization/standard_pose'),
                ]
            ),
            launch_ros.actions.Node(
                package="pgo",
                namespace="pgo",
                executable="pgo_node",
                name="pgo_node",
                output="screen",
                # prefix=['xterm -e gdb -ex run --args'],
                parameters=[{"config_path": pgo_config_path.perform(launch.LaunchContext())}]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="pgo",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
                condition=IfCondition(LaunchConfiguration('visualize')),
            )
        ]
    )
