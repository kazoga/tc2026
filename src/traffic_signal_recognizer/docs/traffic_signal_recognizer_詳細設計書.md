# traffic_signal_recognizer 詳細設計書

## 1. 目的とスコープ

`traffic_signal_recognizer` は、tc2025 まで ROS 1 ノード `tc2023_signal_detector.py` が
担っていた信号横断可否判定を ROS 2 に移行するパッケージである。

YOLO モデルのロードと画像推論は `yolo_detector` に任せ、本パッケージは
`vision_msgs/msg/Detection2DArray` から GO/STOP を判定する。ROS 1 資産をそのまま持ち込むのではなく、
外部 interface と判定仕様を維持したうえで、ROS 通信と判定ロジックを分離する。

## 2. tc2025 互換 interface

tc2025 では、ROS 2 側の `route_follower` と ROS 1 側の信号認識ノードが `ros1_bridge` を介して
連携していた。tc2026 では ROS 2 に一本化するが、route stack との接続契約は維持する。

| Topic | Type | 方向 | 用途 |
| --- | --- | --- | --- |
| `/recog_flag` | `std_msgs/msg/Int32` | Subscribe | `1` の間だけ信号認識を有効化する。 |
| `/sig_recog` | `std_msgs/msg/Int32` | Publish | 信号認識結果。`1=GO`, `2=STOP` を既定とする。 |
| `/sig_det_imgs` | `sensor_msgs/msg/Image` | Publish | 信号認識結果を確認する画像。 |

`route_follower` は signal stop ウェイポイントで `/recog_flag=1` を publish し、`/sig_recog==1`
を受信すると停止解除可能と判断する。本ノードはこの運用契約を ROS 2 内で満たす。

## 3. ノード構成

| ファイル | 役割 |
| --- | --- |
| `traffic_signal_recognizer_node.py` | ROS 2 の parameter、subscriber、publisher、画像変換、描画を担当する。 |
| `signal_recognition_core.py` | ROS 非依存の GO/STOP 判定ロジックを担当する。 |

`traffic_signal_recognizer_node.py` は thin node とし、判定状態や連続 green 判定は
`TrafficSignalRecognitionCore` に閉じ込める。

## 4. 外部 I/F

### 4.1 Subscribe

| パラメータ | 既定値 | 型 | 用途 |
| --- | --- | --- | --- |
| `recog_flag_topic` | `/recog_flag` | `std_msgs/msg/Int32` | 信号認識の有効化フラグ。 |
| `detections_topic` | `/perception/traffic_signal/detections` | `vision_msgs/msg/Detection2DArray` | YOLO 信号検出結果。 |
| `image_topic` | `/usb_cam/image_raw` | `sensor_msgs/msg/Image` | `/sig_det_imgs` 生成用の元画像。 |

### 4.2 Publish

| パラメータ | 既定値 | 型 | 用途 |
| --- | --- | --- | --- |
| `sig_recog_topic` | `/sig_recog` | `std_msgs/msg/Int32` | GO/STOP 判定結果。 |
| `sig_det_image_topic` | `/sig_det_imgs` | `sensor_msgs/msg/Image` | 判定枠と検出矩形を重畳した画像。 |

`/recog_flag != 1` の間は判定履歴を reset し、推論結果を受けても `/sig_recog` を publish しない。
`publish_image_when_disabled=true` の場合、無効時も入力画像を `/sig_det_imgs` へ流し、GUI の信号監視画像を維持する。

## 5. パラメータ

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
| `class_names` | `['red', 'green']` | `Detection2D.results.id` から class name を復元するための対応表。 |
| `hold_go` | `false` | 一度 GO 判定した後に GO を保持するか。 |
| `publish_stop_when_disabled` | `false` | 無効化時に STOP を publish するか。 |
| `publish_image_when_disabled` | `true` | 無効時も入力画像を `/sig_det_imgs` へ publish するか。 |

## 6. 判定仕様

1. `Detection2DArray` 内の各 `Detection2D` から、最も score が高い `results` を取り出す。
2. `confidence_threshold` 未満の候補は破棄する。
3. `green_class_ids` / `green_class_names` または `red_class_ids` / `red_class_names` に一致する候補だけを既知信号として扱う。
4. 1 フレーム内の既知信号候補のうち、最も score が高い候補を採用する。
5. 採用候補が green の場合は `True`、red または未検出の場合は `False` を履歴へ追加する。
6. 直近 `judge_count` 回がすべて green の場合に `go_status` を publish する。
7. それ以外は `stop_status` を publish する。

この仕様は、ROS 1 実装の「3 回連続 green で GO、それ以外は STOP」という運用を維持する。
ただし ROS 1 実装にあった DataFrame 依存や `rospy.wait_for_message` 中心のループ構造は採用しない。

## 7. 画像出力

`/sig_det_imgs` には以下を重畳する。

- GO の場合は緑、STOP の場合は赤の画像外枠。
- confidence 閾値以上の検出矩形。
- class name と score。
- `sig_recog=<value>` の判定値。

画像生成は運用確認用であり、制御判断の正本は `/sig_recog` とする。

## 8. 起動構成

| launch | 内容 |
| --- | --- |
| `traffic_signal_recognizer.launch.py` | 判定ノード単体を起動する。 |
| `traffic_signal_perception.launch.py` | 信号用 `yolo_detector` インスタンスと判定ノードをまとめて起動する。 |

`traffic_signal_perception.launch.py` では、信号用 `yolo_detector` も `/recog_flag` を購読し、
`/recog_flag==1` の間だけ推論を行う。これにより、信号停止地点以外で不要な推論を走らせない。

## 9. テスト方針

ROS 非依存の `TrafficSignalRecognitionCore` を pytest で優先的に確認する。

- green が `judge_count` 回連続した場合に GO となること。
- red または未検出では STOP を維持すること。
- confidence 閾値未満の候補を無視すること。
- `hold_go=true` の場合、一度 GO になった後に GO を保持すること。

ROS 通信、画像描画、実モデル推論は実機・rosbag・GUI に依存するため、通常の自動確認では
`colcon build` と core test までを対象とする。

## 10. 今後の検討事項

- 信号モデルの class id と class name の正式定義をモデル管理資料に記録する。
- 実機で `detection_interval` と `judge_count` の組み合わせを評価する。
- `/sig_det_imgs` の描画内容を robot_console の表示要件に合わせて調整する。
