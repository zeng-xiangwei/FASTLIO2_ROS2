import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "grid_map", "offline.yaml"]
    )


    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="pgo",
                namespace="grid_map_tool",
                executable="tools_3d_to_2d_gridmap",
                # name="lio_node",
                output="screen",
                parameters=[{"config_file": config_path.perform(launch.LaunchContext())}]
            )
        ]
    )
