from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    yolo_pkg_share = FindPackageShare('yolo_detector')
    road_pkg_share = FindPackageShare('road_blockage_detector')

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/usb_cam/image_raw',
        description='購読する画像トピック',
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([yolo_pkg_share, 'models', 'best_ncnn_model']),
        description='経路封鎖看板検出用NCNNモデルディレクトリ',
    )
    detection_interval_arg = DeclareLaunchArgument(
        'detection_interval',
        default_value='0.5',
        description='推論を行うインターバル（秒）',
    )
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='検出の信頼度閾値',
    )
    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='/perception/road_blockage/detections',
        description='Detection2DArrayの出力トピック',
    )
    annotated_image_topic_arg = DeclareLaunchArgument(
        'annotated_image_topic',
        default_value='/perception/road_blockage/detection_image',
        description='検出重畳画像の出力トピック',
    )
    detector_param_file_arg = DeclareLaunchArgument(
        'detector_param_file',
        default_value=PathJoinSubstitution([road_pkg_share, 'params', 'default.yaml']),
        description='road_blockage_detectorノードのパラメータファイル',
    )

    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_ncnn_node',
        name='yolo_detector_road_blockage',
        output='screen',
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'detection_interval': LaunchConfiguration('detection_interval'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'annotated_image_topic': LaunchConfiguration('annotated_image_topic'),
            }
        ],
    )

    road_blockage_node = Node(
        package='road_blockage_detector',
        executable='road_blockage_detector',
        name='road_blockage_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('detector_param_file'),
            {
                'detections_topic': LaunchConfiguration('detections_topic'),
                'image_topic': LaunchConfiguration('image_topic'),
            },
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
            detector_param_file_arg,
            yolo_node,
            road_blockage_node,
        ]
    )
