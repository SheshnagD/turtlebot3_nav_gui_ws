from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_gazebo = FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')
    world_file = os.path.join(
        FindPackageShare('custom_turtlebot3_world').find('custom_turtlebot3_world'),
        'worlds',
        'custom_world.world'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        )
    ])

