from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    yolo_pkg_share = FindPackageShare('yolo_detector')
    signal_pkg_share = FindPackageShare('traffic_signal_recognizer')

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/usb_cam/image_raw',
        description='購読する画像トピック',
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([yolo_pkg_share, 'models', 'traffic_signal_best.pt']),
        description='信号認識用PyTorchモデルのパス',
    )
    detection_interval_arg = DeclareLaunchArgument(
        'detection_interval',
        default_value='0.2',
        description='推論を行うインターバル（秒）',
    )
    image_size_arg = DeclareLaunchArgument(
        'image_size',
        default_value='320',
        description='推論時の入力画像サイズ',
    )
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.8',
        description='検出の信頼度閾値',
    )
    recog_flag_topic_arg = DeclareLaunchArgument(
        'recog_flag_topic',
        default_value='/recog_flag',
        description='信号認識開始フラグ入力トピック',
    )
    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='/perception/traffic_signal/detections',
        description='Detection2DArrayの出力トピック',
    )
    annotated_image_topic_arg = DeclareLaunchArgument(
        'annotated_image_topic',
        default_value='/perception/traffic_signal/image_det',
        description='YOLO検出重畳画像の出力トピック',
    )
    recognizer_param_file_arg = DeclareLaunchArgument(
        'recognizer_param_file',
        default_value=PathJoinSubstitution([signal_pkg_share, 'params', 'default.yaml']),
        description='traffic_signal_recognizerノードのパラメータファイル',
    )

    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_node',
        name='yolo_detector_traffic_signal',
        output='screen',
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'detection_interval': LaunchConfiguration('detection_interval'),
                'image_size': LaunchConfiguration('image_size'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'annotated_image_topic': LaunchConfiguration('annotated_image_topic'),
                'enabled_topic': LaunchConfiguration('recog_flag_topic'),
                'enabled_value': 1,
                'start_enabled': False,
            }
        ],
    )

    recognizer_node = Node(
        package='traffic_signal_recognizer',
        executable='traffic_signal_recognizer',
        name='traffic_signal_recognizer',
        output='screen',
        parameters=[
            LaunchConfiguration('recognizer_param_file'),
            {
                'recog_flag_topic': LaunchConfiguration('recog_flag_topic'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'image_topic': LaunchConfiguration('image_topic'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            },
        ],
    )

    return LaunchDescription(
        [
            image_topic_arg,
            model_path_arg,
            detection_interval_arg,
            image_size_arg,
            confidence_threshold_arg,
            recog_flag_topic_arg,
            detections_topic_arg,
            annotated_image_topic_arg,
            recognizer_param_file_arg,
            yolo_node,
            recognizer_node,
        ]
    )
