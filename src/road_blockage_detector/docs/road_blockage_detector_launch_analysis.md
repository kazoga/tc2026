# 経路封鎖検知の起動と時刻の前提

## 自己位置入力

`road_blockage_detector` は `/localization/pose_enu` の最新値をキャッシュし、
YOLOの `Detection2DArray` を受けたときに参照する。TFによる自己位置検索や
検知時刻への補間は行わない。

位置が未受信の場合は警告を出し、当該時刻の検知数を0として記録して、通常の封鎖判定をスキップする。
検知と位置のstamp差が3秒以上でも警告のみで、最新位置を用いた処理を継続する。
時刻差の警告がある状態を、同期済み・正しい検知位置として扱わない。

## 起動方法

| launch | 構成 |
|---|---|
| `road_blockage_detector.launch.py` | 判定ノードのみ |
| `road_blockage_perception.launch.py` | NCNNのYOLO推論と判定 |
| `road_blockage_perception_yolo.launch.py` | PyTorchのYOLO推論と判定 |

設定YAMLを指定して起動する。これらのlaunchは `use_sim_time` 引数を提供しない。
判定には入力メッセージのstampを使うため、検知と位置の時刻基準を一致させる必要がある。
launchの選択だけで入力の時刻不整合が補正されるわけではない。

トピックとパラメータは [README](../README.md)、
判定・多重検知抑止は [詳細設計](road_blockage_detector_詳細設計書.md)を参照する。
