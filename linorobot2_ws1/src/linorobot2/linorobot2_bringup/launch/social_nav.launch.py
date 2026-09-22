# social_nav Phase 6: khoi dong perception (YOLO pose + depth) va tracking
# (Hungarian+Kalman, fusion lidar-camera) cho social-cost layer (Phase 4)
# va controller (Phase 5). File MOI, khong sua sau vao cac launch file loi
# da chay on dinh -- chi include co dieu kien qua bringup.launch.py voi
# launch arg 'social_nav' (mac dinh false, phai bat tuong minh).
#
# Fix 2026-09-08 (phat hien khi chay Phase 9 test that trong Gazebo):
# thieu use_sim_time -> ca 2 node mac dinh dung wall-clock trong khi toan bo
# simulation chay theo /clock (sim time), khien human_tracker_node1 loi lien
# tuc "Cannot transform LiDAR laser -> odom" (TF buffer duoc stamp theo sim
# time nhung node tra cuu bang wall-clock "now" -- lech hang chuc nam, khong
# bao gio khop). Them arg 'sim' (mac dinh false, giong quy uoc cua
# navigation.launch.py/gazebo.launch.py) de truyen use_sim_time dung xuong
# ca 2 node.
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

    # Model nhan dien nguoi. Doi sang ONNX 2026-09-12 theo yeu cau nguoi dung.
    #
    # So do TRUC TIEP tren chinh NUC nay (i3-6100U, anh 640x480, 15 lan/lan do):
    #   yolo26n-pose_int8  [OpenVINO]  71.5 ms   <- gia tri CU o day, CHAM NHAT
    #   yolo26n-pose       [OpenVINO]  25.9 ms
    #   yolo26n-pose       [ONNX]      33.3 ms   <- dang dung
    #   yolo11n-pose       [ONNX]      35.0 ms
    #   yolov8n (bbox)     [ONNX]      37.1 ms
    #   yolov8n-pose       [ONNX]      38.9 ms
    #
    # Ba dieu rut ra, deu nguoc voi truc giac thong thuong:
    # 1. Ban INT8 cu KHONG nhanh hon ma CHAM GAP 2.8 LAN. Hai ly do cong lai: no duoc
    #    xuat o 640x640 (gap 5.2 lan so diem anh cua ban 256x320), va i3-6100U la
    #    Skylake chi co avx2, KHONG co VNNI -- thieu phan cung do thi OpenVINO phai
    #    mo phong phep nhan INT8, dat hon ca FP32.
    # 2. Trong nhom ONNX, chinh mang dang dung (yolo26n-pose) la NHANH NHAT; yolov8n
    #    va yolo11n deu cham hon.
    # 3. yolov8n chi co bbox (37.1 ms) con CHAM HON yolo26n-pose co du keypoint
    #    (33.3 ms) -- tren phan cung nay, bo pose di khong tiet kiem duoc gi. Dieu do
    #    quan trong vi keypoint la NGUON DUY NHAT cho ra huong nhin cua nguoi (yaw),
    #    ma AGHPM dung Gaussian bat doi xung (sigma_front 0.50 vs sigma_back 0.30).
    #    Bo pose thi vung xa hoi thanh hinh tron deu, mat dung phan "human-aware".
    #
    # LUU Y: `inference_resize` ben duoi gan nhu VO TAC DUNG voi ca ONNX lan OpenVINO
    # IR -- hai dinh dang nay co kich thuoc dau vao CO DINH (day la 256x320), nen
    # ultralytics luon co anh ve dung kich thuoc do bat ke ta dat gi.
    yolo_model_path = PathJoinSubstitution(
        [FindPackageShare('social_nav_perception'), 'weights', 'yolo26n-pose.onnx']
    )

    realsense_launch_path = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )

    # yolo_pose_node1.py subscribe ten topic CO DINH trong code:
    #   /camera/color/image_raw, /camera/depth/image_rect_raw, /camera/color/camera_info
    # Trong Gazebo, ros_gz_bridge publish dung y het cac ten do -> camera_prefix
    # mac dinh "/camera" thanh remap dong nhat (vo hai).
    # Tren xe that TRUOC 2026-09-16, realsense2_camera nam trong namespace long nhau
    # (camera_namespace=camera, camera_name=camera) nen topic that la
    #   /camera/camera/color/image_raw ...
    # va moi lenh phai keo theo camera_prefix:=/camera/camera.
    # NAY KHONG CON NUA: launch truyen camera_namespace='' xuong rs_launch.py (xem
    # khoi IncludeLaunchDescription ben duoi), nen topic that da trung khit ben sim
    # va gia tri mac dinh '/camera' dung cho CA HAI. Giu lai tham so nay chi de
    # phong truong hop co y chay RealSense voi namespace khac.
    camera_prefix = LaunchConfiguration('camera_prefix')

    # Depth PHAI duoc can ve khung anh MAU, vi yolo_pose_node1 lay toa do pixel
    # tren anh mau roi back-project bang noi tham so cua camera MAU. Tra do sau
    # tai dung toa do do chi dung neu hai anh trung khung.
    #
    # LOI CU (sua 2026-09-12): launch nay bat 'align_depth.enable: true' -- dung --
    # nhung roi remap sang '<prefix>/depth/image_rect_raw', tuc topic CHUA CAN.
    # Bat can chinh chi SINH RA MOT TOPIC MOI ('aligned_depth_to_color/image_raw');
    # no khong doi noi dung cua 'depth/image_rect_raw'. Nen cong can chinh lam xong
    # roi bo di, va node doc do sau cua D435 o khung cam bien depth (goc nhin lech,
    # FOV rong hon anh mau) tai toa do pixel cua anh mau -> vi tri 3D cua nguoi sai
    # cho, hoac tra ra do sau vo nghia nen khong publish duoc ket qua nao.
    #
    # Trong Gazebo thi vo hinh: ros_gz_bridge chi co MOT cam bien nen depth vong
    # da trung khung voi anh mau. Day la loi chi xe that moi dinh.
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
                'True neu chay trong Gazebo (dung sim time /clock) -- BAT '
                'BUOC true khi test social_nav trong Gazebo, neu khong '
                'human_tracker_node1 se loi TF lien tuc vi lech wall-clock '
                'voi sim time.'
            )
        ),
        DeclareLaunchArgument(
            name='camera',
            default_value='false',
            description=(
                'True de tu khoi dong realsense2_camera (xe that, D435). '
                'De false khi chay Gazebo vi anh da co san tu ros_gz_bridge.'
            )
        ),
        DeclareLaunchArgument(
            name='debug_image',
            default_value='false',
            description=(
                'True de yolo_pose_node1 cong bo anh da ve khung/khop nguoi len '
                '/perception/image/pose_estimation_result_multiple_3d_ori (xem '
                'bang display "Camera Pose" trong linorobot2_navigation.rviz). '
                'MAC DINH TAT vi ve + ma hoa anh moi khung hinh ton them CPU, '
                'ma CPU dang la nut co chai. Chi bat khi can nhin YOLO thay gi.'
            )
        ),
        DeclareLaunchArgument(
            name='tracker_impl',
            default_value='cpp',
            description=(
                "Ban cai dat cua human_tracker_node1: 'cpp' "
                "(social_nav_tracking_cpp, mac dinh) hoac 'py' "
                "(social_nav_tracking, ban goc de doi chieu). Hai ban dung "
                "CHUNG file tham so va CHUNG ten node, chay lan luot chu khong "
                "chay song song. Do duoc 2026-09-14, cung kich ban 25 s o nhip "
                "thoi gian that: ban py an 99.3% mot nhan va chi cong bo 40/100 "
                "chu ky; ban cpp an 0.9% va cong bo du 100/100."
            )
        ),
        DeclareLaunchArgument(
            name='camera_prefix',
            default_value='/camera',
            description=(
                'Tien to topic camera. Tu 2026-09-16 gia tri mac dinh /camera DUNG '
                'CHO CA sim lan xe that, vi launch da truyen camera_namespace RONG '
                'xuong realsense2_camera de bo tang namespace long. KHONG con phai '
                'dat camera_prefix:=/camera/camera nua -- neu van truyen gia tri do '
                'thi topic thanh /camera/camera/... va YOLO mat anh ngay. Chi dat lai '
                'khi co y chay RealSense voi camera_namespace khac.'
            )
        ),
        DeclareLaunchArgument(
            name='camera_reset',
            default_value='true',
            description=(
                'Reset cung D435 truoc khi mo stream (chi co tac dung khi '
                'camera:=true). De true tru khi dang can khoi dong that nhanh: '
                'no them ~3 s nhung tranh duoc trang thai ket sau mot lan tat '
                'khong sach, luc do driver in "xioctl(VIDIOC_S_FMT) failed, '
                'errno=5" lap vo tan ma node van song va khong topic anh nao ra.'
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
            condition=IfCondition(LaunchConfiguration('camera')),
            launch_arguments={
                # BAT BUOC: khong bat thi depth khong khop pixel voi anh mau,
                # back-project ra toa do 3D sai (ghi chu tu ban goc).
                'align_depth.enable': 'true',
                'pointcloud.enable': 'false',  # NUC CPU-only, khong can pointcloud
                # Them 2026-09-16: BO TANG NAMESPACE LONG.
                #
                # realsense2_camera mac dinh chay voi camera_namespace=camera VA
                # camera_name=camera, ma node nay dung topic rieng tu (~/), nen moi
                # topic bi long hai tang: /camera/camera/color/image_raw...
                # Trong khi Gazebo publish thang /camera/color/image_raw.
                #
                # Hai he qua cua viec de lech ten:
                #  1. Moi lenh chay xe that phai keo theo camera_prefix:=/camera/camera,
                #     quen mot cho la YOLO mat anh.
                #  2. Display DepthCloud "Camera Depth" trong linorobot2_navigation.rviz
                #     tro CUNG vao /camera/depth/image_rect_raw (ten ben sim) nen tren xe
                #     that no khong tim thay gi -> khong ve duoc dam may do sau bam tren
                #     mo hinh xe. DepthCloud con KHONG cho chon topic camera_info, no tu
                #     suy ra tu ten anh depth (/camera/depth/image_rect_raw ->
                #     /camera/depth/camera_info), nen chi doi rieng ten anh cung khong du.
                #
                # De namespace rong thi topic that thanh /camera/... trung khit ben sim,
                # va RealSense VON DA publish san ca depth/image_rect_raw lan
                # depth/camera_info -- khong can them bridge nao nhu ben Gazebo.
                #
                # AN TOAN VE TF: ten khung suy tu camera_name (giu nguyen 'camera') chu
                # khong phai namespace, va tf_prefix mac dinh rong. Nen camera_link,
                # camera_color_optical_frame, camera_depth_optical_frame GIU NGUYEN --
                # phan tra cuu TF cua yolo_pose_node1 khong bi dung toi.
                'camera_namespace': '',
                # Them 2026-09-16: reset cung D435 truoc khi mo stream.
                #
                # Thieu dong nay thi camera de KET trang thai va khong tu thoat ra:
                # neu lan chay truoc khong tat sach (hai driver cung mo mot thiet bi,
                # hoac node bi kill giua luc dang stream) thi lan sau driver in
                #   xioctl(VIDIOC_S_FMT) failed, errno=5 Input/output error
                #   The device has been disconnected! / No RealSense devices were found!
                # lap vo tan. Node VAN SONG, topic van ton tai nhung khong mot khung
                # anh nao duoc phat -- va vi image_callback khong bao gio chay nen
                # cung KHONG co dong log loi nao tu phia ta. Da dinh dung the ngay
                # 2026-09-16. Trieu chung nhin thay (tracked_humans rong) nam rat xa
                # nguyen nhan (USB I/O).
                #
                # Ban goc my_amr_perception/launch/human_detection_test.launch.py
                # co san 'initial_reset': 'true'; buoc port sang day lam roi mat.
                # Gia phai tra: them ~3 s khoi dong. Dang.
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
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('debug_image'), value_type=bool
                ),
                'use_sim_time': LaunchConfiguration('sim'),
            }],
            remappings=camera_remappings,
        ),
        # Hai node duoi day LOAI TRU NHAU qua tracker_impl -- ca hai deu ten
        # 'human_tracker_node1' va deu publish /planning/tracked_humans, nen
        # chay dong thoi la hai nguon ghi de len nhau.
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
