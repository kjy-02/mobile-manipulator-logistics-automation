import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Launch Arguments
    declare_launch_camera = DeclareLaunchArgument(
        'launch_camera',
        default_value='false',
        description='Launch RealSense camera node (rs_launch.py with depth alignment)'
    )

    default_model_path = PathJoinSubstitution([
        FindPackageShare('use_yolo'),
        'box_yolov8n.onnx'
    ])

    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Path to YOLOv8 ONNX model (default: box_yolov8n.onnx)'
    )

    launch_camera = LaunchConfiguration('launch_camera')
    model_path = LaunchConfiguration('model_path')

    # 2. RealSense2 Camera Launch Include (Optional)
    realsense_launch_path = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py'
    ])

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_sync': 'true',
        }.items(),
        condition=IfCondition(launch_camera)
    )

    # 3. YOLOv8 Depth 연동 3D 위치 검출 노드
    yolo_detector_node = Node(
        package='use_yolo',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': model_path
        }]
    )

    # 4. xArm5 6단계 비전 연동 Pick & Drop 제어 노드
    xarm_pick_drop_node = Node(
        package='use_yolo',
        executable='xarm_vision_pick_drop',
        name='xarm_vision_pick_drop',
        output='screen'
    )

    return LaunchDescription([
        declare_launch_camera,
        declare_model_path,
        realsense_node,
        yolo_detector_node,
        xarm_pick_drop_node
    ])
