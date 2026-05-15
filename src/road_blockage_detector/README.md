# road_blockage_detector パッケージ README

## 概要
`road_blockage_detector` は、`yolo_detector` が publish する経路封鎖看板の検出結果、
カメラ画像、自己位置を入力として、経路封鎖の有無を `/road_blocked` へ publish する
ROS 2 パッケージです。YOLO モデルのロードと画像推論は行わず、検出結果の意味判定と
判定重畳画像の生成を担当します。

## 主な機能
- `/perception/road_blockage/detections` から経路封鎖看板候補を抽出。
- score、class id、bbox サイズ条件に基づいて有効検知をフィルタ。
- 判定期間内の検知割合から仮封鎖を判断し、`/road_blocked` を publish。
- `/amcl_pose` を用いて確定封鎖位置を記録し、同一地点付近の多重検知を抑止。
- `/perception/road_blockage/decision_image` に判定状態を重畳した画像を publish。
- `road_blockage_perception.launch.py` により、経路封鎖用 `yolo_detector` と判定ノードをまとめて起動可能。

## 起動方法
### 判定ノード単体
すでに `/perception/road_blockage/detections`、`/usb_cam/image_raw`、`/amcl_pose` が
publish されている場合に使用します。

```bash
ros2 launch road_blockage_detector road_blockage_detector.launch.py
```

### yolo_detector と合わせた経路封鎖検知
NCNN 版 YOLO と `road_blockage_detector` を同時に起動します。

```bash
ros2 launch road_blockage_detector road_blockage_perception.launch.py \
  image_topic:=/usb_cam/image_raw
```

PyTorch 版 YOLO で確認する場合は以下を使用します。

```bash
ros2 launch road_blockage_detector road_blockage_perception_yolo.launch.py \
  image_topic:=/usb_cam/image_raw
```

## 認識系パッケージのみでの動作確認
`robot_navigator` や実機カメラを起動せず、`yolo_detector` の静止画配信、疑似 `/amcl_pose`、
経路封鎖検知 launch だけで確認します。

1. 静止画を `/usb_cam/image_raw` へ publish します。

```bash
ros2 launch yolo_detector camera_simulator_node.launch.py \
  frame_image_path:=<image_path> \
  frame_width:=640 \
  frame_height:=480 \
  frame_ratio:=10.0
```

2. 別ターミナルで疑似 `/amcl_pose` を publish します。

```bash
ros2 topic pub /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}" -r 1
```

3. 別ターミナルで経路封鎖検知一式を起動します。

```bash
ros2 launch road_blockage_detector road_blockage_perception.launch.py \
  image_topic:=/usb_cam/image_raw
```

`road_blockage_perception.launch.py` の既定 `model_path` は
`share/yolo_detector/models/best_ncnn_model` です。PyTorch 版
`road_blockage_perception_yolo.launch.py` の既定値は `share/yolo_detector/models/best.pt` です。
付属モデルを使う場合は `model_path` の指定を省略できます。別モデルを指定する場合、
相対パスは実行時のカレントディレクトリ基準で解釈されます。

PyTorch モデルで確認する場合は `road_blockage_perception_yolo.launch.py` を使用します。

4. 出力を確認します。

```bash
ros2 topic echo /road_blocked
ros2 topic hz /perception/road_blockage/detections
ros2 topic hz /perception/road_blockage/decision_image
```

`/perception/road_blockage/detection_image` は YOLO 生検出の確認用画像、
`/perception/road_blockage/decision_image` は経路封鎖判定後の確認用画像です。

## ROS インタフェース
### Subscribe
| パラメータ | 既定値 | 型 | 説明 |
| --- | --- | --- | --- |
| `detections_topic` | `/perception/road_blockage/detections` | `vision_msgs/msg/Detection2DArray` | YOLO 経路封鎖検出結果。 |
| `image_topic` | `/usb_cam/image_raw` | `sensor_msgs/msg/Image` | 判定重畳画像の元画像。 |
| `amcl_pose_topic` | `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 自己位置。封鎖位置記録と多重検知抑止に使用。 |

### Publish
| パラメータ | 既定値 | 型 | 説明 |
| --- | --- | --- | --- |
| `road_blocked_topic` | `/road_blocked` | `std_msgs/msg/Bool` | 経路封鎖の仮判定状態。 |
| `decision_image_topic` | `/perception/road_blockage/decision_image` | `sensor_msgs/msg/Image` | 判定状態を重畳した画像。 |

## パラメータ
| 名称 | 既定値 | 説明 |
| --- | --- | --- |
| `target_class_id` | `0` | 経路封鎖看板に対応する class id。 |
| `score_threshold` | `0.5` | 判定対象とする最小 score。 |
| `bbox_width_min` | `-1.0` | bbox 幅の下限 [pixel]。負値なら無効。 |
| `bbox_width_max` | `-1.0` | bbox 幅の上限 [pixel]。負値なら無効。 |
| `bbox_height_min` | `-1.0` | bbox 高さの下限 [pixel]。負値なら無効。 |
| `bbox_height_max` | `-1.0` | bbox 高さの上限 [pixel]。負値なら無効。 |
| `bbox_bottom_max` | `-1.0` | bbox 下端位置の上限 [pixel]。負値なら無効。 |
| `decision_duration` | `3.0` | 検知割合を計算する判定期間 [秒]。 |
| `decision_frame_ratio` | `50.0` | 仮封鎖とみなす判定期間内の検知割合 [%]。 |
| `confirmation_duration` | `10.0` | 仮封鎖が継続した場合に封鎖確定とみなす時間 [秒]。 |
| `multi_detection_suppression_range` | `10.0` | 確定封鎖位置付近の多重検知を抑止する距離 [m]。 |
| `publish_decision_image` | `true` | 判定重畳画像を publish するか。 |

## 判定フロー
1. `Detection2DArray` を受信し、最新 `/amcl_pose` が取得できていることを確認します。
2. 各検出について class id、score、bbox 条件を確認し、有効検知数を計数します。
3. `decision_duration` 内の履歴から、有効検知があった秒バケットの割合を算出します。
4. 割合が `decision_frame_ratio` 以上の場合、`/road_blocked=True` を publish します。
5. 仮封鎖が `confirmation_duration` 以上継続した場合、現在位置を封鎖位置として記録します。
6. 閾値未満に戻った場合は `/road_blocked=False` を publish し、誤検知として解除します。

## 開発・ビルド
```bash
cd <ros2_ws>
colcon build --symlink-install --packages-select road_blockage_detector
colcon build --packages-select road_blockage_detector
```

## トラブルシューティング
- `/road_blocked` が publish されない場合は、`/amcl_pose` が流れているか確認してください。未受信時は判定処理をスキップします。
- `/perception/road_blockage/detections` が流れない場合は、統合 launch の `model_path` と `image_topic` を確認してください。
- 検知しているのに封鎖判定にならない場合は、`target_class_id`、`score_threshold`、bbox 閾値、`decision_frame_ratio` の設定を確認してください。
