from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            output='screen',

            parameters=[{
                # Drop every beam hitting the robot body within 15 cm of the lidar (the body
                # radius is ~0.19 m, so points below 0.15 m are always inside the robot; a body
                # part at r = 0.128 m otherwise blocked the safety layer's forward corridor).
                # Replaced by NaN so AMCL and the costmaps treat them as invalid readings, not
                # obstacles. Real robot only (extra:=true); Gazebo filters with the URDF min_range.
                'filter3': {
                    'name': 'remove_robot_body_range',
                    'type': 'laser_filters/LaserScanRangeFilter',
                    'params': {
                        'use_message_range_limits': False,
                        'lower_threshold': 0.15,
                        'upper_threshold': 100000.0,
                        'lower_replacement_value': float('nan'),
                        'upper_replacement_value': float('nan'),
                    }
                },
                'filter1': {
                    'name': 'remove_left_bar',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 1.4835,
                        'upper_angle': 1.6581,
                        'replace_with_nan': True,
                    }
                },

                'filter2': {
                    'name': 'remove_right_bar',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -1.6581,
                        'upper_angle': -1.4835,
                        'replace_with_nan': True,
                    }
                }
            }],

            remappings=[
                ('scan', '/scan_raw'),
                ('scan_filtered', '/scan'),
            ]
        )

    ])