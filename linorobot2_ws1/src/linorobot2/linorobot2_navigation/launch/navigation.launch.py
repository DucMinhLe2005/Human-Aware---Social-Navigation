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


# Ten map lay tu BIEN MOI TRUONG, khong con la hang so phai sua tay.
#
# Truoc 2026-09-16 day la `MAP_NAME='dymap_slam'` va quy trinh deploy bat phai vao
# sua dong nay thanh 'map_nav_v2' moi lan day code xuong NUC (xem deploy_to_nuc.sh).
# Mot buoc thu cong nam trong MA NGUON nhu vay vua de quen, vua lam ban tren may dev
# va ban tren xe that troi khac nhau ma khong cong cu nao phat hien duoc.
#   mo phong : khong dat gi ca              -> 'dymap_slam'
#   xe that  : export LINOROBOT2_MAP=map_nav_v2  (dat san trong env_social_nav.sh)
MAP_NAME = os.environ.get('LINOROBOT2_MAP', 'dymap_slam')


def _resolve_map_path(map_name: str) -> str:
    """Tra ve duong dan .yaml cua map, chap nhan CA HAI cach bay file.

    May dev de map trong thu muc con : maps/<ten>/<ten>.yaml
    NUC de map phang ngay trong maps/: maps/<ten>.yaml
    Truoc day ham nay chi ghep kieu thu muc con, nen dat ten map kieu NUC se tro
    vao duong dan khong ton tai va map_server chet luc nap map -- keo theo ca
    lifecycle_manager huy bringup, mot trieu chung rat xa nguyen nhan that.
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

    # Khong thay o ca hai cho: van tra ve kieu thu muc con de thong bao loi cua
    # map_server chi dung duong dan dang ky vong, thay vi that bai o cho kho hieu.
    return nested

def _params_with_initial_pose(context):
    """Ghi 3 arg initial_pose_x/y/yaw xuong dung khoa amcl trong navigation.yaml.

    Ly do phai lam vong nay thay vi truyen thang: nav2_bringup/bringup_launch.py
    (Jazzy) KHONG khai bao cac arg ten 'initial_pose_x/y/yaw', nen truoc day 3 arg
    o duoi bi nuot im lang -- amcl khong bao gio co pose ban dau, khong publish
    TF map->odom, global_costmap het 20s cho TF roi lifecycle_manager huy toan bo
    bringup. Xem khoi comment 'SUA LOI CHAN DUNG CA STACK' trong navigation.yaml.

    Khong dung nav2_common RewrittenYaml vi no thay khoa theo TEN o moi cap do;
    khoa 'x'/'y'/'yaw' trung ten voi nhieu cho khac trong file. O day sua dung
    duong dan amcl -> ros__parameters -> initial_pose nen khong dung cham gi khac.
    """
    src_path = os.path.join(
        get_package_share_directory('linorobot2_navigation'), 'config', 'navigation.yaml'
    )
    with open(src_path, 'r') as f:
        params = yaml.safe_load(f)

    initial_pose = params['amcl']['ros__parameters']['initial_pose']
    initial_pose['x'] = float(LaunchConfiguration('initial_pose_x').perform(context))
    initial_pose['y'] = float(LaunchConfiguration('initial_pose_y').perform(context))
    initial_pose['yaw'] = float(LaunchConfiguration('initial_pose_yaw').perform(context))

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
            description='Enable use_sime_time to true'
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
            # GIU 0.5 (2026-09-11). Da co mot lan doi thanh 0.0 voi ly do "gazebo.launch.py
            # spawn xe o x=0.0" -- SAI, vi quy trinh chay that luon truyen spawn_x:=0.5:
            #   ros2 launch linorobot2_gazebo gazebo.launch.py world_name:=dymap spawn_x:=0.5
            # Doi thanh 0.0 lam amcl khoi dong lech 0.5m so voi cho xe thuc su dung.
            # HAI GIA TRI NAY PHAI DI THEO CAP: spawn_x cua gazebo.launch.py va
            # initial_pose_x o day. Doi mot ben thi phai doi ben kia.
            default_value='0.5',
            description='Initial robot X position'
        ),

        DeclareLaunchArgument(
            name='initial_pose_y',
            default_value='0.0',
            description='Initial robot Y position'
        ),

        DeclareLaunchArgument(
            name='initial_pose_yaw',
            default_value='0.0',
            description='Initial robot yaw'
        ),

        DeclareLaunchArgument(
            name='lidar_safety',
            default_value='true',
            description=(
                'Phase 8 (social_nav): bat node lidar_safety_node1 lam lop '
                'phong thu vat ly cuoi cung giua collision_monitor va micro-ROS '
                'agent. BAT BUOC de True tru khi ban tu doi lai '
                'collision_monitor.cmd_vel_out_topic trong navigation.yaml ve '
                '"cmd_vel" -- hien tai da doi sang "cmd_vel_raw" de node nay '
                'chen vao, neu tat ma khong sua lai yaml thi robot se KHONG '
                'nhan duoc lenh cmd_vel nao ca.'
            )
        ),

        DeclareLaunchArgument(
            name='safety_impl',
            default_value='cpp',
            description=(
                "Ban cai dat cua lidar_safety_node1: 'cpp' "
                "(social_nav_safety_cpp, mac dinh) hoac 'py' (social_nav_safety, "
                "ban goc de doi chieu). Hai ban dung CHUNG file tham so va da "
                "duoc chung minh trung khit tung buoc bang bo test phat lai du "
                "lieu vang (social_nav_safety_cpp/test/golden/safety_trace.txt). "
                "Chenh lech CPU o day NHO (do 2026-09-14, nhip thoi gian that: "
                "py 3.3% mot nhan, cpp 1.0%) -- khac han truong hop tracker, vi "
                "ban Python o day dung rclpy.spin mot luong chu khong phai "
                "MultiThreadedExecutor."
            )
        ),

        OpaqueFunction(
            function=lambda context: [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_path),
                    launch_arguments={
                        'map': LaunchConfiguration("map"),
                        'use_sim_time': LaunchConfiguration("sim"),
                        # File tam da duoc ghi de initial_pose theo 3 arg o tren.
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

        # Phase 8 (social_nav): lop phong thu vat ly cuoi cung, doc lap voi
        # costmap/controller/perception -- xem comment DeclareLaunchArgument
        # 'lidar_safety' o tren va navigation.yaml (collision_monitor).
        # Hai node duoi day LOAI TRU NHAU qua safety_impl -- ca hai deu ten
        # 'lidar_safety_node1' va deu publish /cmd_vel, nen chay dong thoi la
        # hai nguon lenh danh nhau tren dung cai topic dieu khien dong co.
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
