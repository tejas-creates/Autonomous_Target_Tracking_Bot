from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'Autonomous_Target_Tracking_Bot'


def generate_launch_description():
    trajectory = Path(__file__).resolve().parents[2] / 'docs/trajectory.csv'
    communication_file = Path(__file__).resolve().parents[1] / 'config/communication_bridges.yaml'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': str(communication_file)}
        ],
        output='screen',
    )
    
    controller = Node(
        package='waypoint_controller',
        executable='waypoint_controller',
        parameters=[{'csv_file': str(trajectory)}],
        output='screen',
    )
    
    camera = Node(
        package='camera_module',
        executable='camera_module',
        parameters=[{'use_sim_time': True, 'show_image': False}],
        output='screen',
    )

    # Moving obstacles (ros_ws/src/random_obstacles): moves the 20 Moving_Cylinder_* models.
    # Change default_value of obstacle_motion here ('random' or 'crossing'),
    # or pick it per run: ros2 launch ros_ws/launch/simulation.launch.py obstacle_motion:=random
    obstacles_arg = DeclareLaunchArgument(
        'random_obstacles', default_value='true',
        description='Start the moving obstacle controller (random_obstacles package)')
    motion_arg = DeclareLaunchArgument(
        'obstacle_motion', default_value='random', choices=['crossing', 'random'],
        description='crossing = shuttle across the target route, random = random straight legs')
    seed_arg = DeclareLaunchArgument(
        'obstacle_seed', default_value='-1', description='Random seed, -1 = different every run')
    obstacles = Node(
        package='random_obstacles',
        executable='random_obstacle_controller',
        parameters=[{
            'world_name': WORLD_NAME,
            'motion': LaunchConfiguration('obstacle_motion'),
            'seed': ParameterValue(LaunchConfiguration('obstacle_seed'), value_type=int),
            'trajectory_file': str(trajectory),
        }],
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('random_obstacles'), "'.lower() in ('true', '1', 'yes')"])),
    )

    lidar = Node(
        package='lidar',
        executable='obstacle_detector',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # The NMPC needs CasADi, which is installed only in ros_ws/.venv_nmpc.
    # 'prefix' runs the node with that Python, however the workspace was built.
    venv_python = Path(__file__).resolve().parents[1] / '.venv_nmpc/bin/python'
    nmpc = Node(
        package='nmpc',
        executable='nmpc_controller',
        prefix=str(venv_python),
        parameters=[str(Path(__file__).resolve().parents[1] / 'config/nmpc.yaml'),
                    {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([obstacles_arg, motion_arg, seed_arg,
                              bridge, controller, camera, lidar, nmpc, obstacles])
