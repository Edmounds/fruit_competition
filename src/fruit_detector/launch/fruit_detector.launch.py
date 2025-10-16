from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """启动水果检测节点 - 单摄像头两阶段检测
    
    模型A：成熟度检测 (ripe/unripe)
    模型C：种类检测 (辣椒/南瓜/洋葱/番茄)
    """
    # Get package directory
    package_dir = get_package_share_directory('fruit_detector')
    model_path_a = os.path.join(package_dir, 'model', 'A.onnx')
    model_path_c = os.path.join(package_dir, 'model', 'C.onnx')
    
    # Launch arguments
    image_topic = LaunchConfiguration('image_topic', default='/camera/color/image_raw')
    ripeness_confidence = LaunchConfiguration('ripeness_confidence', default='0.75')
    classifier_confidence = LaunchConfiguration('classifier_confidence', default='0.75')
    iou_threshold = LaunchConfiguration('iou_threshold', default='0.3')
    image_center_x = LaunchConfiguration('image_center_x', default='320')
    image_center_y = LaunchConfiguration('image_center_y', default='240')
    enable_on_start = LaunchConfiguration('enable_on_start', default='true')
    
    # Declare launch arguments
    declare_image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Topic name for the camera image input'
    )
    
    declare_ripeness_confidence = DeclareLaunchArgument(
        'ripeness_confidence',
        default_value='0.6',
        description='Confidence threshold for ripeness detection (Model A)'
    )
    
    declare_classifier_confidence = DeclareLaunchArgument(
        'classifier_confidence',
        default_value='0.75',
        description='Confidence threshold for fruit classification (Model C)'
    )
    
    declare_iou_threshold = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.3',
        description='IoU threshold for checking overlap between detections'
    )
    
    declare_image_center_x = DeclareLaunchArgument(
        'image_center_x',
        default_value='320',
        description='Image center X coordinate (for offset calculation)'
    )
    
    declare_image_center_y = DeclareLaunchArgument(
        'image_center_y',
        default_value='240',
        description='Image center Y coordinate (for offset calculation)'
    )
    
    declare_enable_on_start = DeclareLaunchArgument(
        'enable_on_start',
        default_value='true',
        description='Enable detection on startup'
    )
    
    # Create node
    fruit_detector_node = Node(
        package='fruit_detector',
        executable='fruit_detector_node',
        name='fruit_detector',
        parameters=[{
            'image_topic': image_topic,
            'model_path_a': model_path_a,
            'model_path_c': model_path_c,
            'ripeness_confidence': ripeness_confidence,
            'classifier_confidence': classifier_confidence,
            'iou_threshold': iou_threshold,
            'image_center_x': image_center_x,
            'image_center_y': image_center_y,
            'enable_on_start': enable_on_start
        }],
        output='screen'
    )
    
    return LaunchDescription([
        declare_image_topic,
        declare_ripeness_confidence,
        declare_classifier_confidence,
        declare_iou_threshold,
        declare_image_center_x,
        declare_image_center_y,
        declare_enable_on_start,
        fruit_detector_node
    ]) 