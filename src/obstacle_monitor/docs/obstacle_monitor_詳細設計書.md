# obstacle_monitor 詳細設計書

## 1. 目的とスコープ

`obstacle_monitor` は `/scan`（LiDAR）から前方の閉塞状況と左右の回避余裕を求め、
`tc_route_msgs/ObstacleAvoidanceHint` として配信する。あわせて判定の根拠を
`/sensor_viewer` に可視化する。

本ノードは観測から回避ヒントを導くところまでを担う。回避経路の生成は `route_follower`、
速度の決定は `robot_navigator` の責務であり、本ノードは指令を出さない。
購読側の使い分けは、`robot_navigator` が距離（`front_clearance_m`）、`route_follower` が
閉塞（`front_blocked`）と左右幅を参照する。

## 2. 外部インタフェース

トピック名、型、QoS、パラメータの一覧は [README](../README.md) を正とする。
本書は一覧を再掲せず、値の意味と解釈の制約を定める。

## 3. 判定ロジック

### 3.1 点群の抽出

`/scan` の受信時に NaN/Inf と後方データを除外し、前方 ±90° の点群へ変換する。
`max_obstacle_distance_m` 以下の点を左右に分割し、`|y|` 昇順へ並べ替える。

### 3.2 左右の回避オフセット

ギャップ検出と外縁推定で左右のオフセットを算出し、`avoid_offset_min_m` を下限として適用する。

### 3.3 前方閉塞

前方距離は車幅帯内の x 座標のパーセンタイル（`front_clearance_percentile`）で求める。
`front_cone_half_deg` 内での距離分位点が `stop_dist_m` 以下のとき閉塞と判定する。

## 4. 制約

**点数が不足する場合は `front_blocked=false`、`front_clearance_m=inf` を返す。**
未観測と空き領域を区別する品質フラグは持たない。したがって購読側は、新鮮なメッセージを
受け取っていることをもって有効な障害物観測が成立しているとは判断できない。
入力途絶の検出は購読側で別に行う必要がある。

**`hint_range_m` を大きくしても検知範囲は広がらない。** 前方点の抽出上限は
`max_obstacle_distance_m` であり、`hint_range_m` は通知する距離の上限にすぎない。
`hint_range_m` < `max_obstacle_distance_m` の場合は警告を出して内部で補正する。

`/sensor_viewer` は点群・車幅・範囲・距離・回避幅・`active_target` を描画する。目標は
自己位置と yaw からローカル座標へ変換して描く。これは観測結果の可視化であり、
旋回中の車体外周・後方・低い段差の通行性を保証するものではない。

## 5. 確認方法

`tests/` で `_is_front_blocked` の境界挙動を確認する。有効な返りなし、全点無効、少数点、
車幅帯の内外を分けて検査する。

結合確認では `laser_scan_simulator` で `/scan` を供給し、`/obstacle_avoidance_hint` の
`front_blocked` と `left_offset_m` が障害物に応じて変化することを確認する。
ヒントの値だけでなく、下流の最終速度まで併せて確認する。

実 LiDAR の反射特性、低い段差、旋回中の車体外周は模擬入力では再現しない。
