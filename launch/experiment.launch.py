from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = "ros2_latency"
    # We need to convert the desired MB into 
    conf_mb = LaunchConfiguration("megabytes", default=10.0)
    points_per_mb = round(1024.0 * 1024.0 / 32.0)  # See field point_step in `ros2 topic echo /pc_source`
    resulting_points = PythonExpression(["int(", str(points_per_mb), " * ", conf_mb, ")"])
    return LaunchDescription([
        Node(
            package=pkg,
            name='ros2_latency',
            executable='source',
            parameters=[
                {"num_points": resulting_points},
            ],
            output="screen"
        ),
        Node(
            package=pkg,
            name='ros2_latency',
            executable='repeater',
            output="screen"
        ),
        Node(
            package=pkg,
            name='ros2_latency',
            executable='repeater.py',
            output="screen"
        ),
        Node(
            package=pkg,
            name='ros2_latency',
            executable='measure',
            output="screen"
        ),
    ])
