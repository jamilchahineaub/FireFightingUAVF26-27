"""cl415 in gazebo harmonic with ros 2 jazzy

    ros2 launch cl415_description sim.launch.py                          # water, tank full, with gui
    ros2 launch cl415_description sim.launch.py world:=ground tank:=empty headless:=true

args
    world          water | ground | path to an .sdf             (default water)
    tank           full | empty                                 (default full)
    water_damping  false | true   hydrodynamic damper, float tests only (it damps in the air too)
    headless       false | true   server only, no gui
    x y z yaw      spawn pose, z defaults to just above the water / ground
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SPAWN_Z = {'water': '0.29', 'ground': '0.32'}   # keel is 0.31 m below the wing le root


def setup(context):
    pkg = get_package_share_directory('cl415_description')
    arg = {k: LaunchConfiguration(k).perform(context) for k in
           ('world', 'tank', 'water_damping', 'headless', 'x', 'y', 'z', 'yaw')}
    world = arg['world']
    world_path = world if world.endswith('.sdf') else os.path.join(pkg, 'worlds', world + '.sdf')
    z = arg['z'] or SPAWN_Z.get(world, '1.0')

    robot_description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'cl415.urdf.xacro'),
        mappings={'tank': arg['tank'], 'water_damping': arg['water_damping']}).toxml()

    server = '-s ' if arg['headless'] == 'true' else ''
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                                                   'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {server}-v 3 {world_path}', 'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'cl415', '-topic', 'robot_description',
                            '-x', arg['x'], '-y', arg['y'], '-z', z, '-Y', arg['yaw']])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'), 'use_sim_time': True}])

    return [gz, rsp, spawn, bridge]


def generate_launch_description():
    share = os.path.dirname(get_package_share_directory('cl415_description'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='water'),
        DeclareLaunchArgument('tank', default_value='full'),
        DeclareLaunchArgument('water_damping', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value=''),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # model:// and package:// mesh uris resolve against <install>/share
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share),
        OpaqueFunction(function=setup),
    ])
