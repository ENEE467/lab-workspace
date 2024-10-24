import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, FindExecutable, PythonExpression

import sys

def generate_launch_description():
    desc = LaunchDescription()

    str_camera_name = 'logitech_webcam'
    for i in sys.argv:
        if i.find('camera_name') > -1:
            str_camera_name = i[i.find('=')+1:]
    

    show_output_var = LaunchConfiguration('show_output_var', default='False')
    show_output = DeclareLaunchArgument(
        name='show_output', default_value=show_output_var
    )

    launch_calibration_var = LaunchConfiguration('launch_calibration_var', default='False')
    launch_calibration = DeclareLaunchArgument(
        name='launch_calibration', default_value=launch_calibration_var
    )

    aruco_tracker_var = LaunchConfiguration('aruco_tracker_var', default='False')
    aruco_tracker = DeclareLaunchArgument(
        name='aruco_tracker', default_value=aruco_tracker_var
    )

    camera_name_var = LaunchConfiguration('camera_name_var', default='logitech_webcam')
    camera_name = DeclareLaunchArgument(
        name='camera_name', default_value=camera_name_var
    )

    camera_device_var = LaunchConfiguration('camera_device_var', default='/dev/video0')
    camera_device = DeclareLaunchArgument(
        name='camera_device', default_value=camera_device_var
    )

    marker_size_var = LaunchConfiguration('marker_size_var', default='0.15')
    marker_size = DeclareLaunchArgument(
        name='marker_size', default_value=marker_size_var
    )

    marker_dict_var = LaunchConfiguration('marker_dict_var', default='4X4_50')
    marker_dict = DeclareLaunchArgument(
        name='marker_dict', default_value=marker_dict_var
    )

    desc.add_action(show_output)
    desc.add_action(launch_calibration)
    desc.add_action(aruco_tracker)
    desc.add_action(camera_device)
    desc.add_action(camera_name)
    desc.add_action(marker_size)
    desc.add_action(marker_dict)

    width = LaunchConfiguration('width', default='1280')
    height = LaunchConfiguration('height', default='720')

    # Start USB Camera Connection
    start_usb_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=camera_name_var,
        parameters=[
                {
                    'video_device': camera_device_var,
                    'image_width': width,
                    'image_height': height,
                    'pixel_format': 'mjpeg2rgb',
                    'frame_id': 'tracking_cam',
                    'camera_name': camera_name_var,
                    'camera_info_url': f'package://lab8/calibration/camera/{str_camera_name}.yaml',
                }
            ],
        output='screen',
    )

    desc.add_action(start_usb_camera)

    # Start Aruco Detection Node
    start_aruco_detect = Node(
        package="aruco_opencv",
        executable="aruco_tracker",
        parameters=[
            {
                'cam_base_topic': 'image_raw',
                'marker_size': marker_size_var,
                'marker_dict': marker_dict_var
            }
        ],
        output='screen'
    )

    desc.add_action(start_aruco_detect)

    configure_lifecycle = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/aruco_tracker', 'configure'],
        output='screen'
    )

    activate_lifecycle = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/aruco_tracker', 'activate'],
        output='screen'
    )

    delay_configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_aruco_detect,
            on_start=[configure_lifecycle],
        )
    )

    desc.add_action(delay_configure)

    delay_activate = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=configure_lifecycle,
            on_exit=[activate_lifecycle],
        )
    )

    desc.add_action(delay_activate)

    show_rqt_image = ExecuteProcess(
        condition = IfCondition(
            PythonExpression(show_output_var)
        ),
        cmd=[[
            'ros2 run rqt_image_view rqt_image_view /aruco_tracker/debug'
        ]],
        shell = True
    )

    desc.add_action(show_rqt_image)

    return desc