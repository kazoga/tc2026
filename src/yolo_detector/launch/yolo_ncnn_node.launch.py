from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('yolo_detector')

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/usb_cam/image_raw',
        description='購読する画像トピック',
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([pkg_share, 'models', 'best_ncnn_model']),
        description='NCNNモデルディレクトリのパス',
    )
    detection_interval_arg = DeclareLaunchArgument(
        'detection_interval',
        default_value='0.5',
        description='推論を行うインターバル（秒）',
    )
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.5', description='検出の信頼度閾値'
    )
    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='yolo_detector/detections',
        description='Detection2DArrayの出力トピック',
    )
    annotated_image_topic_arg = DeclareLaunchArgument(
        'annotated_image_topic',
        default_value='yolo_detector/detection_image',
        description='検出重畳画像の出力トピック',
    )
    enabled_topic_arg = DeclareLaunchArgument(
        'enabled_topic',
        default_value='',
        description='推論有効化フラグの入力トピック',
    )
    enabled_value_arg = DeclareLaunchArgument(
        'enabled_value', default_value='1', description='推論を有効化するInt32値'
    )
    start_enabled_arg = DeclareLaunchArgument(
        'start_enabled',
        default_value='true',
        description='起動直後に推論を有効にするか',
    )

    yolo_ncnn_node = Node(
        package='yolo_detector',
        executable='yolo_ncnn_node',
        name='yolo_ncnn_node',
        output='screen',
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'detection_interval': LaunchConfiguration('detection_interval'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'annotated_image_topic': LaunchConfiguration('annotated_image_topic'),
                'enabled_topic': LaunchConfiguration('enabled_topic'),
                'enabled_value': LaunchConfiguration('enabled_value'),
                'start_enabled': LaunchConfiguration('start_enabled'),
            }
        ],
    )

    return LaunchDescription(
        [
            image_topic_arg,
            model_path_arg,
            detection_interval_arg,
            confidence_threshold_arg,
            detections_topic_arg,
            annotated_image_topic_arg,
            enabled_topic_arg,
            enabled_value_arg,
            start_enabled_arg,
            yolo_ncnn_node,
        ]
    )
