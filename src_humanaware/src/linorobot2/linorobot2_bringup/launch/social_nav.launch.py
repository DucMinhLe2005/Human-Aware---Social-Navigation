# Starts the social navigation front end: person perception (YOLO pose + depth)
# and human tracking (Hungarian + Kalman, lidar/camera fusion), which feed the
# AGHPM social costmap layer and the human-aware controller.
#
# Set sim:=true in Gazebo so every node uses sim time (/clock); otherwise TF
# lookups compare sim-time stamps with wall-clock "now" and always fail.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    human_tracker_params_path = PathJoinSubstitution(
        [FindPackageShare('social_nav_tracking'), 'config', 'human_tracker_params.yaml']
    )

    # Person detector model. Measured on an Intel i3-6100U (640x480 images),
    # yolo26n-pose in ONNX took 33.3 ms per frame and was the fastest ONNX pose
    # model; a bbox-only model was not faster, and the keypoints are the only
    # source of the person's body heading used by the asymmetric social zone.
    # The ONNX model has a fixed input size (256x320), so `inference_resize`
    # below has little effect on it.
    yolo_model_path = PathJoinSubstitution(
        [FindPackageShare('social_nav_perception'), 'weights', 'yolo26n-pose.onnx']
    )

    realsense_launch_path = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )

    # yolo_pose_node1 subscribes to fixed topic names:
    #   /camera/color/image_raw, /camera/depth/image_rect_raw, /camera/color/camera_info
    # Gazebo (ros_gz_bridge) and the RealSense driver (launched with an empty
    # camera_namespace, see below) both publish these names, so the default
    # prefix "/camera" works for both. Change it only for a different namespace.
    camera_prefix = LaunchConfiguration('camera_prefix')

    # On the real camera, depth must be ALIGNED to the colour image: keypoints are
    # pixel coordinates in the colour image and are back-projected with the colour
    # intrinsics. Enabling alignment creates a separate topic
    # (aligned_depth_to_color/image_raw), so that topic is used with camera:=true.
    # In Gazebo there is a single sensor and depth is already aligned.
    depth_suffix = PythonExpression([
        "'/aligned_depth_to_color/image_raw' if '",
        LaunchConfiguration('camera'),
        "'.lower() in ('true', '1', 'yes') else '/depth/image_rect_raw'",
    ])

    camera_remappings = [
        ('/camera/color/image_raw', [camera_prefix, '/color/image_raw']),
        ('/camera/depth/image_rect_raw', [camera_prefix, depth_suffix]),
        ('/camera/color/camera_info', [camera_prefix, '/color/camera_info']),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim',
            default_value='false',
            description=(
                'True when running in Gazebo (use sim time /clock). Required in '
                'simulation, otherwise the tracker keeps failing TF lookups.'
            )
        ),
        DeclareLaunchArgument(
            name='camera',
            default_value='false',
            description=(
                'True to start realsense2_camera (real robot, D435). '
                'Keep false in Gazebo, where images come from ros_gz_bridge.'
            )
        ),
        DeclareLaunchArgument(
            name='debug_image',
            default_value='false',
            description=(
                'True to publish the image with drawn skeletons on '
                '/perception/image/pose_estimation_result_multiple_3d_ori. '
                'Off by default because drawing and encoding every frame costs CPU.'
            )
        ),
        DeclareLaunchArgument(
            name='tracker_impl',
            default_value='cpp',
            description=(
                "Implementation of human_tracker_node1: 'cpp' "
                "(social_nav_tracking_cpp, default) or 'py' "
                "(social_nav_tracking, reference implementation). Both use the "
                "same parameter file and node name and must not run together."
            )
        ),
        DeclareLaunchArgument(
            name='camera_prefix',
            default_value='/camera',
            description=(
                'Camera topic prefix. The default /camera is correct for both '
                'simulation and the real robot; change it only when running the '
                'RealSense driver with a different camera_namespace.'
            )
        ),
        DeclareLaunchArgument(
            name='camera_reset',
            default_value='true',
            description=(
                'Hardware-reset the D435 before streaming (only with '
                'camera:=true). Adds ~3 s but recovers the camera from a stuck '
                'state after an unclean shutdown, in which the driver keeps '
                'printing "xioctl(VIDIOC_S_FMT) failed, errno=5" and publishes '
                'no images.'
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
            condition=IfCondition(LaunchConfiguration('camera')),
            launch_arguments={
                # Required: without alignment depth pixels do not match the colour
                # image and the 3D back-projection is wrong.
                'align_depth.enable': 'true',
                'pointcloud.enable': 'false',  # not needed, saves CPU
                # Empty namespace: by default the driver nests its topics as
                # /camera/camera/..., while Gazebo publishes /camera/... . With an
                # empty namespace the real topic names match the simulation ones.
                # Frame names derive from camera_name ('camera'), not from the
                # namespace, so TF frames are unchanged.
                'camera_namespace': '',
                # Hardware reset before streaming (see the camera_reset argument).
                'initial_reset': LaunchConfiguration('camera_reset'),
            }.items(),
        ),
        Node(
            package='social_nav_perception',
            executable='yolo_pose_node1',
            name='yolo_pose_node1',
            output='screen',
            parameters=[{
                'model_path': yolo_model_path,
                'detection_period_sec': 0.3,
                'inference_resize': [256, 192],
                # ONNX Runtime threads per inference. Without a limit it starts one
                # thread per core and can starve the Nav2 container. One inference
                # takes ~10-15 ms at ~3.3 inferences/s, so 2 threads are plenty.
                # 2 in simulation, 1 on the real robot (dual-core NUC).
                'onnx_num_threads': ParameterValue(PythonExpression([
                    "2 if '", LaunchConfiguration('sim'), "'.lower() == 'true' else 1"
                ]), value_type=int),
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('debug_image'), value_type=bool
                ),
                'use_sim_time': LaunchConfiguration('sim'),
            }],
            remappings=camera_remappings,
        ),
        # The two tracker nodes below are mutually exclusive (tracker_impl): both
        # are named 'human_tracker_node1' and publish /planning/tracked_humans.
        Node(
            package='social_nav_tracking_cpp',
            executable='human_tracker_node1',
            name='human_tracker_node1',
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('tracker_impl'), "' == 'cpp'"])
            ),
            parameters=[
                human_tracker_params_path,
                {'use_sim_time': LaunchConfiguration('sim')},
            ],
        ),
        Node(
            package='social_nav_tracking',
            executable='human_tracker_node1',
            name='human_tracker_node1',
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('tracker_impl'), "' != 'cpp'"])
            ),
            parameters=[
                human_tracker_params_path,
                {'use_sim_time': LaunchConfiguration('sim')},
            ],
        ),
    ])
