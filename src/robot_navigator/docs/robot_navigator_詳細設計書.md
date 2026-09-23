# robot_navigator 詳細設計書

## 1. 目的とスコープ

`robot_navigator` は `/active_target` の目標姿勢へ車体を誘導する速度指令を生成する。
入力は自己位置、車輪速度、障害物距離であり、出力は `Twist` である。

`route_follower` が `/active_target` を生成し、本ノードが速度指令へ変換する。共通起動では
融合による減速と `drive_mode_manager` の mux を経て車輪ドライバへ渡る。

試験用の `robot_simulator` を同梱する。

## 2. 外部インタフェース

トピック名、型、QoS、パラメータの一覧は [README](../README.md) を正とする。
本書は一覧を再掲せず、制御則と制約を定める。

## 3. 制御ループ

制御周期は `control_rate_hz`（既定 20 Hz）とする。

1. odom・`/localization/pose_enu`・`/active_target` の受信状況を監視し、欠損時は
   `cmd_vel_topic` へゼロを出力して WARN を 5 秒周期で報告する。
2. 入力が揃うと `compute_time_optimal_cmd_vel()` を呼び、角度誤差の PID 制御で角速度を算出する。
3. 線速度は角度誤差と障害物距離に基づくスケールを適用し、`max_acc_v` に従って加速度を制限する。
4. 目標までの距離と、障害物距離から `min_obstacle_distance` を引いた距離について、
   `v × 遅延 + v² / (2 × max_decel_v)` の制動距離で前進速度を制限する。ここで `v` は
   実測速度と直前指令の絶対値の大きい方とする。停止が必要なら前進指令をゼロにする。
5. 指令を `cmd_vel_topic` へ publish し、`/direction_marker` で進行方向を可視化する。
6. CSV ログが有効な場合は速度、誤差、障害物距離を逐次書き出す。

## 4. 障害物距離の入力

`obstacle_distance_mode` は `hint`（既定）または `scan` を選ぶ。`hint` は
`ObstacleAvoidanceHint.front_clearance_m`、`scan` は前方帯の距離を使う。

**非有限な hint 距離は `None` へ変換され、距離制約を適用しない。** したがって新鮮な
メッセージを受け取っていても、有効な障害物観測が成立しているとは限らない。
`obstacle_monitor` が点数不足時に `inf` を返すためである。

`obstacle_timeout_sec` は単体の既定が 0 で監視無効、共通実機起動は 1 秒で監視する。

## 5. 制約

**障害物による制動は線速度にのみ適用する。** 目標への角度誤差が許容値を超えている場合、
前進指令がゼロでも旋回指令は残り得る。

**`route_follower` の待機・完了状態と実測速度ゼロは別である。** 目標再送が止まっても、
本ノードが保持している目標は解除されない。本ノードは `/follower_state` を購読せず、
`/active_target` の受信期限も監視しないため、follower 停止時の指令失効は保証されない。

`max_acc_w` はドライバとの能力照合に使う要求値であり、本ノード内で角指令の変化率を
制限する値ではない。

## 6. 入力途絶時の停止

`pose_timeout_sec`、`odom_timeout_sec` はそれぞれ既定 1.0 秒である。
`/localization/pose_enu` または odom が未受信、または受信間隔が閾値以上の場合、制御タイマーで
ゼロ速度を発行し、PID 積分・前回指令をクリアする。両入力の受信が復帰すると既存目標への
追従を再開する。

判定時刻は実機で `time.monotonic()`、`use_sim_time=true` では ROS 時計とする。
低速な物理計算や一時停止の wall 時間を模擬入力の欠測に数えない。

**ROS メッセージ自体の stamp の鮮度は、この監視では検証しない。** 融合ノード側で別に監視する。
古い内容の再配信、LiDAR の途絶、自己位置の品質低下、ノード自体の停止は別途対策が要る。

監視は ROS 非依存の `input_watchdog_core.py` が担当する。

## 7. 確認方法

制動と能力整合は `tests/test_braking.py`、契約の失効・欠落は `tests/test_motion_contract.py`
で確認する。入力途絶の境界と復帰は `tests/test_input_watchdog_core.py` で確認する。

結合確認では `robot_simulator` を起動し、単体構成では `/cmd_vel`、統合構成では
`/cmd_vel/autonomous`・`/drive_mode_status`・最終 `/cmd_vel`・`/direction_marker` の挙動と
CSV ログの内容を確認する。

同梱 `robot_simulator` の契約既定は最大線速度 2.0 m/s・角速度 3.0 rad/s、線加速度 0.7 m/s²・
線減速度 1.5 m/s²・角加速度 1.5 rad/s² である。これらは契約として配信される値であり、
シミュレータの速度更新に物理的な加減速を再現するものではない。

制動式は一定減速度を仮定した設計モデルであり、実路面・荷重での停止距離の実測値ではない。
