# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


# Map name from the environment, so simulation and the real robot need no code change:
#   simulation : unset                               -> 'dymap_slam'
#   real robot : export LINOROBOT2_MAP=<map name>
MAP_NAME = os.environ.get('LINOROBOT2_MAP', 'dymap_slam')


def _resolve_map_path(map_name: str) -> str:
    """Return the .yaml path of the map, accepting both layouts.

        maps/<name>/<name>.yaml   (map in its own folder)
        maps/<name>.yaml          (flat)
    """
    maps_dir = os.path.join(
        get_package_share_directory('linorobot2_navigation'), 'maps'
    )
    nested = os.path.join(maps_dir, map_name, f'{map_name}.yaml')
    if os.path.isfile(nested):
        return nested

    flat = os.path.join(maps_dir, f'{map_name}.yaml')
    if os.path.isfile(flat):
        return flat

    # Not found in either place: return the nested form so map_server reports the
    # expected path instead of failing somewhere less obvious.
    return nested

# Default start pose when the three initial_pose_* arguments are empty:
#   simulation (sim:=true) : (0.5, 0.0, 0.0) -- must match spawn_x:=0.5 of gazebo.launch.py
#   real robot             : read from ~/linorobot2_ws1/start_pose_<LINOROBOT2_MAP>.txt
#                            (lines of the form initial_pose_x:=...), so AMCL starts
#                            where the robot stands without a manual 2D Pose Estimate.
# Explicit initial_pose_x:=... arguments always take precedence.
SIM_DEFAULT_POSE = (0.5, 0.0, 0.0)


def _initial_pose(context):
    args = {k: LaunchConfiguration(f'initial_pose_{k}').perform(context).strip()
            for k in ('x', 'y', 'yaw')}
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    base, source = SIM_DEFAULT_POSE, 'simulation default (0.5, 0, 0)'
    pose_file = os.path.expanduser(f'~/linorobot2_ws1/start_pose_{MAP_NAME}.txt')
    if not sim and os.path.isfile(pose_file):
        # A broken file (empty, half-written, missing keys) must not kill the launch:
        # this is an OpaqueFunction, and an exception here would stop Nav2 and the
        # safety layer. Fall back to the default and warn loudly.
        try:
            with open(pose_file) as f:
                saved = dict(tok.split(':=', 1) for tok in f.read().split() if ':=' in tok)
            base = tuple(float(saved[f'initial_pose_{k}']) for k in ('x', 'y', 'yaw'))
            # Warn when the file is old: if the robot was moved since it was saved, AMCL
            # would start at the wrong place.
            age_hours = (time.time() - os.path.getmtime(pose_file)) / 3600.0
            source = f'{pose_file} (saved {age_hours:.1f} h ago)'
            if age_hours > 6.0:
                print('!' * 70)
                print(f'[navigation.launch] START POSE FILE IS {age_hours:.1f} HOURS OLD')
                print('[navigation.launch] if the robot was moved, set a 2D Pose Estimate again')
                print('!' * 70)
        except (KeyError, ValueError, OSError) as e:
            print('!' * 70)
            print(f'[navigation.launch] FILE {pose_file} IS BROKEN ({type(e).__name__}: {e})')
            print('[navigation.launch] using (0.5, 0, 0) -- set a 2D Pose Estimate in RViz')
            print('!' * 70)
            source = f'BROKEN FILE {pose_file}, using (0.5, 0, 0)'
    elif not sim:
        source = f'NO {pose_file}, using (0.5, 0, 0) -- set a 2D Pose Estimate'
    pose = tuple(float(args[k]) if args[k] else b for k, b in zip(('x', 'y', 'yaw'), base))
    if any(args.values()):
        source = 'command-line arguments' + ('' if all(args.values()) else f' + {source}')
    print(f'[navigation.launch] AMCL start pose x={pose[0]:.3f} y={pose[1]:.3f} '
          f'yaw={pose[2]:.3f} <- {source}')
    return pose


def _params_with_initial_pose(context):
    """Write the initial_pose_x/y/yaw arguments into the amcl section of navigation.yaml.

    nav2_bringup/bringup_launch.py (Jazzy) does not declare initial_pose_x/y/yaw, so
    passing them directly would be silently ignored: AMCL would get no initial pose,
    publish no map->odom, and the lifecycle manager would abort the bringup.

    nav2_common RewrittenYaml is not used because it replaces keys by NAME at every
    level, and 'x'/'y'/'yaw' appear in many places; here only
    amcl -> ros__parameters -> initial_pose is modified.
    """
    src_path = os.path.join(
        get_package_share_directory('linorobot2_navigation'), 'config', 'navigation.yaml'
    )
    with open(src_path, 'r') as f:
        params = yaml.safe_load(f)

    initial_pose = params['amcl']['ros__parameters']['initial_pose']
    pose = _initial_pose(context)
    initial_pose['x'], initial_pose['y'], initial_pose['yaw'] = pose

    tmp = tempfile.NamedTemporaryFile(
        mode='w', prefix='linorobot2_navigation_', suffix='.yaml', delete=False
    )
    yaml.safe_dump(params, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'rviz', 'linorobot2_navigation.rviz']
    )

    default_map_path = _resolve_map_path(MAP_NAME)

    lidar_safety_params_path = PathJoinSubstitution(
        [FindPackageShare('social_nav_safety'), 'config', 'lidar_safety_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Use simulation time (/clock)'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

       DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        DeclareLaunchArgument(
            name='initial_pose_x',
            # Empty = automatic (see _initial_pose): 0.5 in simulation, start_pose file on the
            # real robot. The simulation value must match spawn_x of gazebo.launch.py.
            default_value='',
            description='Initial robot X position'
        ),

        DeclareLaunchArgument(
            name='initial_pose_y',
            default_value='',
            description='Initial robot Y position'
        ),

        DeclareLaunchArgument(
            name='initial_pose_yaw',
            default_value='',
            description='Initial robot yaw'
        ),

        DeclareLaunchArgument(
            name='lidar_safety',
            default_value='true',
            description=(
                'Start lidar_safety_node1, the last safety layer between '
                'collision_monitor and the motor driver. Must stay true unless '
                'you change '
                'collision_monitor.cmd_vel_out_topic in navigation.yaml back to '
                '"cmd_vel" -- it publishes "cmd_vel_raw" so that this node '
                'sits in between; disabling it without changing the YAML means '
                'the robot receives NO velocity command.'
            )
        ),

        DeclareLaunchArgument(
            name='safety_impl',
            default_value='cpp',
            description=(
                "Implementation of lidar_safety_node1: 'cpp' "
                "(social_nav_safety_cpp, default) or 'py' (social_nav_safety, "
                "reference implementation). Both share the parameter file and are "
                "proven step-for-step identical by the golden replay test "
                "(social_nav_safety_cpp/test/golden/safety_trace.txt)."
            )
        ),

        OpaqueFunction(
            function=lambda context: [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_path),
                    launch_arguments={
                        'map': LaunchConfiguration("map"),
                        'use_sim_time': LaunchConfiguration("sim"),
                        # Temporary parameter file with the initial pose written in.
                        'params_file': _params_with_initial_pose(context),
                    }.items()
                )
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        ),

        # Lidar safety layer: the last physical guard, independent of costmap, controller
        # and perception. The two nodes below are mutually exclusive (safety_impl): both
        # are named 'lidar_safety_node1' and publish /cmd_vel.
        Node(
            package='social_nav_safety_cpp',
            executable='lidar_safety_node1',
            name='lidar_safety_node1',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('lidar_safety'),
                "'.lower() in ('true', '1', 'yes') and '",
                LaunchConfiguration('safety_impl'), "' == 'cpp'",
            ])),
            parameters=[
                lidar_safety_params_path,
                {'use_sim_time': LaunchConfiguration('sim')},
            ]
        ),
        Node(
            package='social_nav_safety',
            executable='lidar_safety_node1',
            name='lidar_safety_node1',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('lidar_safety'),
                "'.lower() in ('true', '1', 'yes') and '",
                LaunchConfiguration('safety_impl'), "' != 'cpp'",
            ])),
            parameters=[
                lidar_safety_params_path,
                {'use_sim_time': LaunchConfiguration('sim')},
            ]
        )
    ])
