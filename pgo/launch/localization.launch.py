import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    global_pcd_file_arg = DeclareLaunchArgument('global_pcd_file', 
                                default_value='/home/diana/vln/fastlio2-ros2_ws/data/map.pcd',
                                description='Path to the global PCD map file')
    initial_pose_file_arg = DeclareLaunchArgument('initial_pose_file', 
                                default_value='/home/diana/vln/fastlio2-ros2_ws/data/initial_pose.txt',
                                description='Path to the initial pose file')
    pose_load_mode_arg = DeclareLaunchArgument('pose_load_mode', 
                                default_value='0',
                                description='Pose load mode: 0=from topic, 1=from file first')
    
    visualize_arg = DeclareLaunchArgument('visualize', default_value='true')

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
            global_pcd_file_arg,
            initial_pose_file_arg,
            pose_load_mode_arg,
            visualize_arg,

            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                # name="lio_node",
                output="screen",
                parameters=[{"config_path": lio_config_path.perform(launch.LaunchContext()),
                             "saved_pose_file_path": saved_pose_file_path.perform(launch.LaunchContext())}],
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
                parameters=[{"config_path": pgo_config_path.perform(launch.LaunchContext()),
                             "global_pcd_file": LaunchConfiguration('global_pcd_file'),
                             "initial_pose_file": LaunchConfiguration('initial_pose_file'),
                             "pose_load_mode": LaunchConfiguration('pose_load_mode')}],
            ),

            launch_ros.actions.Node(
                package="rviz2",
                namespace="pgo",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext()),'--ros-args', '--log-level', 'WARN'],
                condition=IfCondition(LaunchConfiguration('visualize')),
            )
        ]
    )
