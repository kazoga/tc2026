# traffic_signal_recognizer パッケージ README

## 概要
`traffic_signal_recognizer` は、`yolo_detector` が publish する信号検出結果
`vision_msgs/msg/Detection2DArray` を入力として、信号横断可否を `/sig_recog` へ publish する
ROS 2 パッケージです。YOLO モデルのロードと画像推論は行わず、GO/STOP 判定と判定重畳画像の
生成を担当します。

## 主な機能
- `/recog_flag==1` の間だけ信号判定を有効化。
- YOLO の red / green 検出結果から、既定では 3 回連続 green で GO と判定。
- `/sig_recog` に `1=GO`, `2=STOP` を publish。
- `/perception/traffic_signal/decision_image` に判定状態を重畳した画像を publish。
- `traffic_signal_perception.launch.py` により、信号用 `yolo_detector` と判定ノードをまとめて起動可能。

## ノード構成
| ファイル | 役割 |
| --- | --- |
| `traffic_signal_recognizer_node.py` | ROS 2 parameter、subscriber、publisher、画像変換、描画を担当。 |
| `signal_recognition_core.py` | ROS 非依存の GO/STOP 判定ロジックを担当。 |

## 起動方法
### 判定ノード単体
すでに `/perception/traffic_signal/detections` と `/usb_cam/image_raw` が publish されている場合に使用します。

```bash
ros2 launch traffic_signal_recognizer traffic_signal_recognizer.launch.py
```

### yolo_detector と合わせた信号認識
信号用 PyTorch YOLO と `traffic_signal_recognizer` を同時に起動します。

```bash
ros2 launch traffic_signal_recognizer traffic_signal_perception.launch.py \
  image_topic:=/usb_cam/image_raw
```

`traffic_signal_perception.launch.py` では、YOLO 推論と判定ノードの両方が `/recog_flag` を参照します。
動作確認時は別ターミナルで以下を publish し、認識を有効化します。

```bash
ros2 topic pub /recog_flag std_msgs/msg/Int32 "{data: 1}" -r 1
```

## 認識系パッケージのみでの動作確認
`route_follower` や実機カメラを起動せず、`yolo_detector` の静止画配信と信号認識 launch だけで確認します。

1. 静止画を `/usb_cam/image_raw` へ publish します。

```bash
ros2 launch yolo_detector camera_simulator_node.launch.py \
  frame_image_path:=<image_path> \
  frame_width:=640 \
  frame_height:=480 \
  frame_ratio:=10.0
```

2. 別ターミナルで信号認識一式を起動します。

```bash
ros2 launch traffic_signal_recognizer traffic_signal_perception.launch.py \
  image_topic:=/usb_cam/image_raw
```

`traffic_signal_perception.launch.py` の既定 `model_path` は
`share/yolo_detector/models/traffic_signal_best.pt` です。付属モデルを使う場合は
`model_path` の指定を省略できます。別モデルを指定する場合、相対パスは実行時の
カレントディレクトリ基準で解釈されます。

3. 別ターミナルで `/recog_flag` を有効化します。

```bash
ros2 topic pub /recog_flag std_msgs/msg/Int32 "{data: 1}" -r 1
```

4. 出力を確認します。

```bash
ros2 topic echo /sig_recog
ros2 topic hz /perception/traffic_signal/detections
ros2 topic hz /perception/traffic_signal/decision_image
```

`/perception/traffic_signal/detection_image` は YOLO 生検出の確認用画像、
`/perception/traffic_signal/decision_image` は GO/STOP 判定後の確認用画像です。

## ROS インタフェース
### Subscribe
| パラメータ | 既定値 | 型 | 説明 |
| --- | --- | --- | --- |
| `recog_flag_topic` | `/recog_flag` | `std_msgs/msg/Int32` | `1` の間だけ信号認識を有効化。 |
| `detections_topic` | `/perception/traffic_signal/detections` | `vision_msgs/msg/Detection2DArray` | YOLO 信号検出結果。 |
| `image_topic` | `/usb_cam/image_raw` | `sensor_msgs/msg/Image` | 判定重畳画像の元画像。 |

### Publish
| パラメータ | 既定値 | 型 | 説明 |
| --- | --- | --- | --- |
| `sig_recog_topic` | `/sig_recog` | `std_msgs/msg/Int32` | 信号判定結果。既定値は `1=GO`, `2=STOP`。 |
| `decision_image_topic` | `/perception/traffic_signal/decision_image` | `sensor_msgs/msg/Image` | 判定状態を重畳した画像。 |

## パラメータ
| 名称 | 既定値 | 説明 |
| --- | --- | --- |
| `confidence_threshold` | `0.8` | 判定対象とする最小 confidence。 |
| `judge_count` | `3` | GO 判定に必要な連続 green 回数。 |
| `go_status` | `1` | GO として publish する値。 |
| `stop_status` | `2` | STOP として publish する値。 |
| `unknown_class_id` | `99` | 未検出時に内部的に扱う class id。 |
| `green_class_ids` | `[1]` | green とみなす class id。 |
| `red_class_ids` | `[0]` | red とみなす class id。 |
| `green_class_names` | `['green']` | green とみなす class name。 |
| `red_class_names` | `['red']` | red とみなす class name。 |
| `hold_go` | `false` | 一度 GO 判定した後に GO を保持するか。 |
| `publish_stop_when_disabled` | `false` | 無効化時に STOP を publish するか。 |
| `publish_image_when_disabled` | `true` | 無効時も入力画像を判定重畳画像 topic へ publish するか。 |

## 開発・テスト
ROS 非依存ロジックは `tests/test_signal_recognition_core.py` で確認します。

```bash
cd <ros2_ws>
pytest src/traffic_signal_recognizer/tests
colcon build --symlink-install --packages-select traffic_signal_recognizer
colcon build --packages-select traffic_signal_recognizer
```

## トラブルシューティング
- `/sig_recog` が publish されない場合は、`/recog_flag` が `1` で publish されているか確認してください。
- `/perception/traffic_signal/detections` が流れない場合は、`traffic_signal_perception.launch.py` の `model_path` と `/recog_flag` を確認してください。
- GO 判定が出にくい場合は、モデルの class id / class name と `green_class_ids`、`red_class_ids`、`confidence_threshold` の対応を確認してください。
