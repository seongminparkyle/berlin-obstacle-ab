"""Headless Berlin trial using the installed MISYS Forza stack."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from race_manager.track import Track


def generate_launch_description():
    root = Path(__file__).resolve().parent
    map_dir = Path(os.environ.get('AB_MAP_DIR', root.parent / 'berlin_analysis')).resolve()
    waypoints = map_dir / 'global_waypoints.json'
    track = Track(str(waypoints))
    start = int(abs(track.s - (track.length - 0.8)).argmin())
    stack = Path(get_package_share_directory('stack_master'))
    sim = stack / 'config' / 'SIM'
    seed = int(os.environ['AB_SEED'])
    obstacle_config = Path(os.environ.get('AB_OBSTACLE_CONFIG', root / 'obstacle_config.yaml'))
    spawner_env = {}
    race_manager_source = os.environ.get('AB_RACE_MANAGER_SOURCE')
    if race_manager_source:
        python_paths = [race_manager_source, os.environ.get('PYTHONPATH', '')]
        spawner_env['PYTHONPATH'] = os.pathsep.join(filter(None, python_paths))

    def include(package, filename, **arguments):
        path = Path(get_package_share_directory(package)) / 'launch' / filename
        return IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())

    return LaunchDescription([
        Node(package='f1tenth_gym_ros', executable='gym_bridge', name='bridge', parameters=[
            str(sim / 'sim.yaml'),
            {'map_path': str(map_dir / 'berlin.yaml'), 'sim_params': str(sim / 'sim_params.yaml'),
             'sx': float(track.x[start]), 'sy': float(track.y[start]),
             'stheta': float(track.psi[start]), 'kb_teleop': False}]),
        Node(package='stack_master', executable='global_parameter_node', name='global_parameters',
             parameters=[{'racecar_version': 'SIM', 'sim': True, 'map_path': str(map_dir)}]),
        Node(package='global_planner', executable='global_trajectory_publisher', name='global_republisher',
             parameters=[{'map_path': str(map_dir)}]),
        Node(package='speed_scaler', executable='speed_scaler', name='speed_scaler',
             parameters=[str(map_dir / 'speed_scaling.yaml')]),
        Node(package='frenet_odom_republisher', executable='frenet_odom_republisher_node',
             name='frenet_odom_republisher', parameters=[{'pose_topic': '/car_state/pose_GT'}],
             remappings=[('/early_fusion/odom', '/car_state/odom_GT')]),
        include('state_machine', 'state_machine_launch.xml', mode='head_to_head'),
        Node(package='controller_cpp', executable='controller_manager_cpp', name='controller',
             parameters=[{'mode': 'MAP', 'LU_table': 'SIM_linear', 'mapping': False}],
             remappings=[('/drive', '/ab/controller_drive')]),
        include('spline_planner', 'spline_planner_launch.xml'),
        include('perception', 'perception_launch.xml',
                param_file=str(stack / 'config' / 'opponent_tracker_params.yaml')),
        include('perception_cpp', 'perception_launch.xml',
                param_file=str(stack / 'config' / 'opponent_tracker_params_cpp.yaml')),
        Node(package='race_manager', executable='obstacle_spawner', name='obstacle_spawner',
             additional_env=spawner_env,
             parameters=[str(obstacle_config.resolve()),
                         {'waypoints_file': str(waypoints), 'seed': seed, 'num_obstacles': 4}]),
    ])

