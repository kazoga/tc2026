# traffic_signal_recognizer 詳細設計書

## 1. 目的とスコープ

`traffic_signal_recognizer` はROS 2の信号横断可否判定パッケージである。

YOLO モデルのロードと画像推論は `yolo_detector` に任せ、本パッケージは
`vision_msgs/msg/Detection2DArray` から GO/STOP を判定する。ROS通信と判定ロジックは分離する。

## 2. 外部 interface と topic 方針

認識ノードは画像を配信せず、`road_blockage_detector` と対になる形で認識結果のみを
`tc_perception_msgs/msg/PerceptionOverlay` として配信する。画像への重畳は表示側
（`robot_console` の `ConsoleCore`）が行う。

| Topic | Type | 方向 | 用途 |
| --- | --- | --- | --- |
| `/recog_flag` | `std_msgs/msg/Int32` | Subscribe | `1` の間だけ信号認識を有効化する。 |
| `/sig_recog` | `std_msgs/msg/Int32` | Publish | 信号認識結果。`1=GO`, `2=STOP` を既定とする。 |
| `/perception/traffic_signal/overlay` | `tc_perception_msgs/msg/PerceptionOverlay` | Publish | 検出矩形と判定結果。表示側が画像へ重畳するために使う。 |

`route_follower` は signal stop ウェイポイントで `/recog_flag=1` を publish し、`/sig_recog==1`
を受信すると停止解除可能と判断する。本ノードはこの運用契約を ROS 2 内で満たす。
フロントカメラは 1 台のみであり、認識ノードごとに重畳画像を配信すると同一フレームの
画像が複数系統流れる。判定ノードは認識データを送り、表示側は `/usb_cam/image_raw` を共用する。
YOLO自体の確認用detection_imageは別途配信される。

## 3. ノード構成

| ファイル | 役割 |
| --- | --- |
| `traffic_signal_recognizer_node.py` | ROS 2 の parameter、subscriber、publisher を担当する。 |
| `signal_recognition_core.py` | ROS 非依存の GO/STOP 判定ロジックを担当する。 |

`traffic_signal_recognizer_node.py` は thin node とし、判定状態や連続 green 判定は
`TrafficSignalRecognitionCore` に閉じ込める。

## 4. 外部 I/F

### 4.1 Subscribe

| パラメータ | 既定値 | 型 | 用途 |
| --- | --- | --- | --- |
| `recog_flag_topic` | `/recog_flag` | `std_msgs/msg/Int32` | 信号認識の有効化フラグ。 |
| `detections_topic` | `/perception/traffic_signal/detections` | `vision_msgs/msg/Detection2DArray` | YOLO 信号検出結果。 |

### 4.2 Publish

| パラメータ | 既定値 | 型 | 用途 |
| --- | --- | --- | --- |
| `sig_recog_topic` | `/sig_recog` | `std_msgs/msg/Int32` | GO/STOP 判定結果。 |
| `overlay_topic` | `/perception/traffic_signal/overlay` | `tc_perception_msgs/msg/PerceptionOverlay` | 検出矩形と判定結果。 |

`/recog_flag != 1` の間は判定履歴を reset し、推論結果を受けても `/sig_recog` と
`overlay_topic` のいずれも publish しない。表示側は鮮度低下により未受信として扱う。

`overlay_topic` は表示用であり取りこぼしを許容できるため BEST_EFFORT で配信し、
`road_blockage_detector` 側と QoS を揃える。制御経路へ渡す `/sig_recog` は既定
（RELIABLE）のままとする。

### 4.3 フレームの紐付け
`PerceptionOverlay.header` には、判定根拠となった元画像フレームの `stamp` / `frame_id` を
そのまま引き継ぐ。`yolo_detector` が `Detection2DArray.header` へ元画像の header を複製して
いるため、本ノードはそれを転記するだけでよい。

認識は生画像より低いレートで動作するため、生画像の全フレームに対して結果が対応する
とは限らない。表示側は直近の結果を保持して描画する前提で実装する（保持時間や
同期判定の扱いは `robot_console` 側の責務）。

`OverlayDetection.adopted` には、`confidence_threshold` 以上で判定に採用した検出かどうかを
入れる。採用外の検出も残すことで、判定に使われなかった検出を表示側で描き分けられる。

### 4.4 画像 topic の位置づけ
- `/perception/traffic_signal/detection_image` は `yolo_detector_traffic_signal` が publish する YOLO 生検出の確認用画像である。
- 本ノードは画像を publish しない。判定後の重畳表示は `robot_console` が生画像へ描画する。
- 制御判断の正本は `/sig_recog` とする。

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
| `class_names` | `['red', 'green']` | `Detection2D.results[].hypothesis.class_id` から class name を復元するための対応表。 |
| `hold_go` | `false` | 一度 GO 判定した後に GO を保持するか。 |
| `publish_stop_when_disabled` | `false` | 無効化時に STOP を publish するか。 |
| `overlay_topic` | `/perception/traffic_signal/overlay` | 認識結果の出力 topic。 |

## 6. 判定仕様

1. `Detection2DArray` 内の各 `Detection2D` から、最も score が高い `results` を取り出す。
2. `confidence_threshold` 未満の候補は破棄する。
3. `green_class_ids` / `green_class_names` または `red_class_ids` / `red_class_names` に一致する候補だけを既知信号として扱う。
4. 1 フレーム内の既知信号候補のうち、最も score が高い候補を採用する。
5. 採用候補が green の場合は `True`、red または未検出の場合は `False` を履歴へ追加する。
6. 直近 `judge_count` 回がすべて green の場合に `go_status` を publish する。
7. それ以外は `stop_status` を publish する。


## 7. 認識結果の出力

`/perception/traffic_signal/overlay` には以下を載せる。本ノードは画像を扱わない。

- 検出矩形（中心座標と大きさ）。
- class name と score。
- `confidence_threshold` 以上で判定に採用したかを表す `adopted`。
- `decision`（`sig_recog` と同じ値）と `decision_text`（`GO` / `STOP`）。

描画内容の決定は表示側の責務とする。`robot_console` は採用した検出を太線、採用外を
細線で描き分け、判定結果は画像へ焼き込まずチップとして表示する。走行中に運転者が
読むのは判定そのものであり、画像内の小さな文字では数 m 離れた位置から判読できない
ためである。

配信は運用確認用であり、制御判断の正本は `/sig_recog` とする。

## 8. 起動構成

| launch | 内容 |
| --- | --- |
| `traffic_signal_recognizer.launch.py` | 判定ノード単体を起動する。 |
| `traffic_signal_perception.launch.py` | 信号用 `yolo_detector` インスタンスと判定ノードをまとめて起動する。 |

`traffic_signal_perception.launch.py` では、信号用 `yolo_detector` も `/recog_flag` を購読し、
`/recog_flag==1` の間だけ推論を行う。これにより、信号停止地点以外で不要な推論を走らせない。

## 9. road_blockage_detector との対応関係
`traffic_signal_recognizer` と `road_blockage_detector` は、YOLO 推論層の後段で意味判定を行う対になるパッケージとして扱う。

| 項目 | traffic_signal_recognizer | road_blockage_detector |
| ---- | ---- | ---- |
| YOLO 入力 | `/perception/traffic_signal/detections` | `/perception/road_blockage/detections` |
| raw 画像入力 | なし（画像を扱わない） | なし（画像を扱わない） |
| 制御出力 | `/sig_recog` | `/road_blocked` |
| 認識結果 | `/perception/traffic_signal/overlay` | `/perception/road_blockage/overlay` |
| YOLO 生検出画像 | `/perception/traffic_signal/detection_image` | `/perception/road_blockage/detection_image` |

`/perception/*/detection_image` は YOLO 推論結果の確認用である。後段判定結果の確認は
`/perception/*/overlay` を `robot_console` が生画像へ重畳した表示で行う。

## 10. テスト方針

ROS 非依存の `TrafficSignalRecognitionCore` を pytest で優先的に確認する。

- green が `judge_count` 回連続した場合に GO となること。
- red または未検出では STOP を維持すること。
- confidence 閾値未満の候補を無視すること。
- `hold_go=true` の場合、一度 GO になった後に GO を保持すること。

ROS 通信、画像描画、実モデル推論は実機・rosbag・GUI に依存するため、通常の自動確認では
`colcon build` と core test までを対象とする。
