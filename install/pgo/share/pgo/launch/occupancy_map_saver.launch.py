import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="pgo",
                namespace="grid_map_tool",
                executable="occupancy_map_saver",
                # name="lio_node",
                output="screen",
                parameters=[
                    {"map_dir": "/home/xiangweizeng/my_project/fastlio2-ros2_ws/data/0707"}, 
                    {"map_prefix": "map_2d"}],
                remappings=[
                    ('map', '/mapUGV')]
            )
        ]
    )
