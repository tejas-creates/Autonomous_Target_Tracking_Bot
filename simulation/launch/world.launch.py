from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = 'Autonomous_Target_Tracking_Bot'


def generate_launch_description():
    simulation_dir = Path(__file__).resolve().parent.parent
    worlds = {
        'garden': simulation_dir / 'garden_world.sdf',
        'plain': simulation_dir / 'simulation_world.sdf',
    }
    agv_file = simulation_dir / 'URDF_with_meshes/urdf/simple_assem.urdf'
    target_file = simulation_dir / 'Target_AGV.sdf'

    world_arg = DeclareLaunchArgument(
        'world', default_value='garden', choices=list(worlds),
        description='garden = garden_world.sdf, plain = simulation_world.sdf')
    mesh_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=str(agv_file.parent.parent / 'meshes')
    )

    gazebo_launch = Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py'

    def start_gazebo(context):
        world_file = worlds[LaunchConfiguration('world').perform(context)]
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch)),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        )]

    agv = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', WORLD_NAME,
                   '-file', str(agv_file), '-name', 'AGV',
                   '-x', '-0.8', '-y', '4', '-z', '0.1205'],
        output='screen',
    )

    target = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', WORLD_NAME,
                   '-file', str(target_file), '-name', 'Target_AGV',
                   '-x', '0', '-y', '4', '-z', '0.035'],
        output='screen',
    )

    # The moving obstacles are started by ros_ws/launch/simulation.launch.py (random_obstacles).
    return LaunchDescription([world_arg, mesh_path, OpaqueFunction(function=start_gazebo),
                              agv, target])
