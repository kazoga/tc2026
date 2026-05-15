# yolo_detector 詳細設計書

## 1. 目的とスコープ

`yolo_detector` は、カメラ画像を YOLO モデルへ入力し、検出結果を
`vision_msgs/msg/Detection2DArray` と重畳画像として publish する共通推論パッケージである。

tc2026 の認識系では、本パッケージは画像推論層に責務を限定する。経路封鎖看板であるか、
信号が GO/STOP のどちらか、といった意味判定は下流の `road_blockage_detector` と
`traffic_signal_recognizer` が担当する。

## 2. ノード構成

| ノード | 実装 | 用途 |
| --- | --- | --- |
| `yolo_node` | `yolo_detector/yolo_node.py` | PyTorch `.pt` モデルを用いた YOLO 推論。信号認識モデルなど、PyTorch モデルを直接使う用途を想定する。 |
| `yolo_ncnn_node` | `yolo_detector/yolo_ncnn_node.py` | NCNN 形式モデルを用いた YOLO 推論。経路封鎖看板など、CPU 推論を軽量化したい用途を想定する。 |
| `camera_simulator_node` | `yolo_detector/camera_simulator_node.py` | 静止画を `sensor_msgs/msg/Image` として publish する検証補助ノード。 |

`road_blockage_detector` の実行 entry point と旧実装ファイルは本パッケージから削除する。
経路封鎖判定ノード本体は `road_blockage_detector` パッケージから起動する。

## 3. 外部 I/F

### 3.1 Subscribe

| パラメータ | 既定値 | 型 | 用途 |
| --- | --- | --- | --- |
| `image_topic` | `/usb_cam/image_raw` | `sensor_msgs/msg/Image` | 推論対象画像。 |
| `enabled_topic` | 空文字 | `std_msgs/msg/Int32` | 空でない場合、推論有効化フラグとして購読する。 |

`enabled_topic` が空文字の場合は常時推論する。空でない場合は、受信値が `enabled_value` と一致する間だけ推論する。

### 3.2 Publish

| パラメータ | 既定値 | 型 | 用途 |
| --- | --- | --- | --- |
| `detections_topic` | `yolo_detector/detections` | `vision_msgs/msg/Detection2DArray` | YOLO 検出結果。 |
| `annotated_image_topic` | `yolo_detector/detection_image` | `sensor_msgs/msg/Image` | 検出矩形とスコアを重畳した画像。 |

検出結果の `header` は入力画像の `header` を引き継ぐ。`Detection2D.results[0].hypothesis.class_id`
には class id を文字列化して格納し、`Detection2D.results[0].hypothesis.score` には confidence
を格納する。

## 4. パラメータ

| 名称 | 対象 | 既定値 | 説明 |
| --- | --- | --- | --- |
| `model_path` | 共通 | 空文字 | モデルファイルまたはモデルディレクトリのパス。空の場合は各ノードの既定モデルを使用する。 |
| `image_topic` | 共通 | `/usb_cam/image_raw` | 入力画像 topic。 |
| `detection_interval` | 共通 | `0.5` | 推論 timer 周期 [秒]。 |
| `confidence_threshold` | 共通 | `0.5` | YOLO 推論時の confidence 閾値。 |
| `detections_topic` | 共通 | `yolo_detector/detections` | 検出結果出力 topic。 |
| `annotated_image_topic` | 共通 | `yolo_detector/detection_image` | 重畳画像出力 topic。 |
| `enabled_topic` | 共通 | 空文字 | 推論有効化フラグ topic。空文字なら購読しない。 |
| `enabled_value` | 共通 | `1` | 推論を有効化する `Int32` 値。 |
| `start_enabled` | 共通 | `true` | 起動直後に推論を有効とみなすか。 |
| `image_size` | `yolo_node` | `320` | PyTorch 推論時の入力画像サイズ。 |
| `class_names` | `yolo_ncnn_node` | `['item']` | NCNN 結果に class name が無い場合の fallback 名。 |

## 5. 用途別インスタンス

経路封鎖看板と信号認識は、モデル、推論頻度、起動条件、対象クラス、下流判定が異なる。
そのため、単一ノードで class を混在処理するのではなく、launch 上で用途別に
`yolo_detector` を複数インスタンス起動する。

| 用途 | 推奨ノード | 主な出力 topic | 下流 |
| --- | --- | --- | --- |
| 経路封鎖看板 | `yolo_ncnn_node` | `/perception/road_blockage/detections` | `road_blockage_detector` |
| 信号認識 | `yolo_node` | `/perception/traffic_signal/detections` | `traffic_signal_recognizer` |

信号認識インスタンスは `/recog_flag` により推論有効状態を制御する。これにより、
tc2025 で `ros1_bridge` 越しに使用していた「信号停止地点だけ推論する」運用契約を
ROS 2 内で維持する。

## 6. 起動構成

| launch | 内容 |
| --- | --- |
| `yolo_node.launch.py` | PyTorch 版 YOLO ノード単体を起動する。 |
| `yolo_ncnn_node.launch.py` | NCNN 版 YOLO ノード単体を起動する。 |

新規構成では、経路封鎖検知全体は `road_blockage_detector` パッケージの
`road_blockage_perception.launch.py` または `road_blockage_perception_yolo.launch.py`、
信号認識全体は `traffic_signal_recognizer` パッケージの `traffic_signal_perception.launch.py`
から起動する。

## 7. エラー処理とログ

- モデル読み込みに失敗した場合は `error` ログを出して例外を再送出する。
- 画像変換に失敗した場合は `error` ログを出し、そのフレームの処理をスキップする。
- 推論 timer は非ブロッキング lock で多重実行を抑止する。
- 推論無効時は timer callback の先頭で return し、最新画像の保持だけを継続する。

## 8. 今後の検討事項

- `Detection2D.results` に class name を含める独自 msg の必要性を検討する。
- 用途別インスタンスの推論周期と CPU 負荷を実機で測定し、既定値を調整する。
- `traffic_signal_best.pt` のモデル来歴、学習データ、クラス定義を別資料で管理する。
