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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = True

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    robot_base = os.getenv('LINOROBOT2_BASE')
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "urdf/robots", f"{robot_base}.urdf.xacro"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui', 
            default_value='true',
            description='Enable Gazebo Client'
        ),
        
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),

        DeclareLaunchArgument(
            name='odom_topic',
            default_value='/odom',
            description='EKF out odometry topic'
        ),

        DeclareLaunchArgument(
            name='world_name',
            # Default: the social navigation world (one walking actor, two standing people).
            default_value='dymap',
            description='Gazebo world name — loads <world_name>.sdf from the linorobot2_gazebo worlds directory'
        ),

        DeclareLaunchArgument(
            name='world_path',
            default_value=[
                FindPackageShare('linorobot2_gazebo'),
                '/worlds/',
                LaunchConfiguration('world_name'),
                '.sdf'
            ],
            description='Full path to the Gazebo world SDF file (overrides world_name when set)'
        ),

        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='0.0',
            description='Robot spawn position in X axis'
        ),

        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='0.0',
            description='Robot spawn position in Y axis'
        ),

        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),
            
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='0.0',
            description='Robot spawn heading'
        ),
        
        # Add the workspace models/ directory to GZ_SIM_RESOURCE_PATH: the world uses
        # model:// meshes (walk.dae, Nurse.obj, Nurse_Col.obj) shipped with this package,
        # so no Fuel download or network access is needed. The package hook is not
        # sourced by colcon, so it is set here; the existing value is kept.
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(
                get_package_share_directory('linorobot2_gazebo'), 'models'
            ) + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
# Force the NVIDIA EGL/GLX vendor: with animated actors gz-sim otherwise falls
# back to Mesa EGL, which has no driver for a proprietary NVIDIA GPU and hangs
# while loading the world. REQUIRES the NVIDIA driver; remove these two
# variables on machines without an NVIDIA GPU.
        SetEnvironmentVariable(
            name='__EGL_VENDOR_LIBRARY_FILENAMES',
            value='/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
        ),
        SetEnvironmentVariable(
            name='__GLX_VENDOR_LIBRARY_NAME',
            value='nvidia'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                # --headless-rendering: required on hybrid Intel + NVIDIA laptops, where EGL
                # device enumeration can hang in gz-sim-sensors-system. It only affects how the
                # sensor rendering backend is created, not physics.
                'gz_args': [' -r -s --headless-rendering ', LaunchConfiguration('world_path')]
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(LaunchConfiguration('gui')),
            launch_arguments={
                'gz_args': [' -g']
            }.items()
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-entity', 'linorobot2', 
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),
            ]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                # Ground-truth poses (GroundTruthPosePublisher1 plugin in dymap.sdf) for
                # social_nav_metrics. A TFMessage, but NOT published on /tf.
                "/social_nav/ground_truth@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                # /camera/points is not bridged: the pipeline does not use it (the detector
                # back-projects depth images itself), it is the most CPU-intensive topic
                # (~9 million points/s), and gz-sim stamps it with the optical frame although
                # the data is in body convention. To inspect depth, display the depth image.
            ],
            remappings=[
                ('/camera/camera_info', '/camera/color/camera_info'),
                ('/camera/image', '/camera/color/image_raw'),
                ('/camera/depth_image', '/camera/depth/image_rect_raw'),
            ]
        ),

        # /camera/depth/camera_info for RViz's DepthCloud display, which derives the
        # camera_info topic from the depth topic name. Gazebo has one RGB-D sensor, so
        # the depth intrinsics equal the colour ones. A separate node is needed because
        # one parameter_bridge cannot remap a topic to two names.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="depth_camera_info_bridge",
            arguments=[
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            remappings=[
                ('/camera/camera_info', '/camera/depth/camera_info'),
            ]
        ),

        Node(
            package='linorobot2_gazebo',
            executable='command_timeout',
            name='command_timeout'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
                'urdf': LaunchConfiguration('urdf')
            }.items()
        )
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940
