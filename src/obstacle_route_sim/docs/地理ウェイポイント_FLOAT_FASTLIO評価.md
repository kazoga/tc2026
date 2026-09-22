# 地理ウェイポイント追従・FLOAT・FAST-LIO 併用評価

## 1. 対象

本書は FAST-LIO 接続前の評価記録である。後続の実装・実測結果は
[FAST-LIO シミュレータ接続・検証](FASTLIOシミュレータ接続検証.md) を参照する。
LiDAR＋IMU による推定は追加済みだが、走行制御への GNSS 融合は未実装である。

2026-09-13、調整版 tc2026_corridor_1m の地形と物体を用い、市役所付近の
40.465 m・22 点を比較対象とする。全 2.224 km の完走検証とは区別する。
初回は既存の ENU CSV、追加試験は同じ点を WGS84 緯度・経度・真北基準方位へ変換した
CSV を route_planner に入力する。GUI での実測点採取は行っていない。

既存の route_planner → route_manager → route_follower → robot_navigator →
drive_cmd_mux → Gazebo DiffDrive を使用する。obstacle_monitor は仮想 LaserScan を入力する。
自己位置は Gazebo 真値からの模擬 GNSS → geo_pose_converter を経由し、真値そのものを
走行制御へ渡さない。真値は誤差評価・模擬センサ生成に使う。

地図原点を経路生成側と GNSS 側で明示的に一致させるため、評価ツールは projection.yaml を
生成して route_planner に渡す。従来の ENU CSV は原点変換を使わず動作したが、既定原点の
まま緯度経度正本の CSV を扱うことはできない。緯度・経度に加えて heading_deg が必要である。
right_is_open / left_is_open は元経路の 2 m を保持するが、その全幅を測量確認したものではない。
途中の一時停止・横断・再開・信号課題はこの試験に含めない。

## 2. 実装

- tools/gnss_environment_core.py: 建物平面輪郭までの距離、状態ヒステリシス、相関 bias を計算する。
- tools/gnss_simulator_node.py: 環境オプションに応じ NavSatFix、Imu、RtkStatus を配信する。
  基地局診断は NTRIP を持たない構成として `DISABLED` を配信する。
- tools/evaluate_terrain_trial.py: 建物 GNSS オプション、RTK 状態件数、projection 設定を追加する。
- tools/prepare_waypoint_evaluation.py: 同一ウェイポイントの FIX/FLOAT/障害物比較環境を別々に生成する。
- tests/test_gnss_environment.py: 距離境界、ヒステリシス、bias の蓄積と減衰、不正値を確認する。

既存ディレクトリに合わせ補助ツールを tools、記録を docs、テストを tests に配置する。
既定で環境劣化は無効であり、既存の FIX 模擬を維持する。

| parameter | default | 内容 |
| --- | ---: | --- |
| buildings_json | 空 | world.json。空なら環境劣化無効 |
| float_enter_m | 12 m | 建物までこの距離以下で FLOAT |
| float_exit_m | 15 m | FLOAT 後、この距離以上で FIX へ復帰 |
| float_sigma_m | 0.6 m | FLOAT 水平白色ノイズの各軸標準偏差 |
| float_bias_m | 1.2 m | FLOAT 時の東・北各軸の漸近 bias |
| float_heading_sigma_deg | 5 度 | FLOAT 時の方位標準偏差 |

bias は時定数 8 秒で増加し、FIX 復帰後も同じ時定数で減衰する。
水平共分散には白色ノイズと bias の二乗を含める。通常の水平 sigma は 0.02 m、
方位 sigma は 0.5 度、10 Hz、遅延 0.1 秒を使用する。
FLOAT は RtkStatus.STATE_RTK_FLOAT=3、FIX=4 として配信する。
NavSatStatus 自体には RTK FIX/FLOAT の区別がないため、RtkStatus を参照する。
建物の高さ・衛星配置・時刻・遮蔽・電波反射を計算する RF シミュレータではない。
今回は測量されていない誤差の仮説を注入する機能であり、実機 receiver の精度を保証しない。

## 3. 確認結果

ENU の同じ 40.465 m 区間で、通常 FIX は FINISHED・停止を確認した。
ゴール真値誤差 0.223 m、自己位置 XY RMSE 0.0868 m、最大経路ずれ 0.114 m、
経路ずれ RMSE 0.0333 m、左右 1 m 外の記録点 0/516、車体接触 0 であった。

FLOAT は 803 件配信され、自己位置 XY RMSE 1.759 m、最大経路ずれ 1.598 m、
経路ずれ RMSE 1.184 m、左右 1 m 外の記録点 580/737 だった。
85 秒期限時点でゴールへ未到達（残距離 7.79 m）、車体接触は 0 だった。
時間内未到達だけで恒久的に完走不能とは断定しない。1 m 以内の精度条件を満たさないことが
主要な失敗である。位置受信は続くので、既存の途絶 watchdog はこの劣化を止めない。

自己位置誤差は受信した自己位置と直近真値の比較で遅延の影響を含む。
経路ずれは記録した真値位置から概略経路全線分への最短距離である。
走行記録は約 0.1 秒間隔で、サンプル間の最大値までは保証しない。
結果は log/codex/waypoint_eval/{fixed,float}/result.json、trajectory.csv、path_metrics.json に保存する。

## 4. FAST-LIO を併用する構成案

GNSS の更新が来るたびに直接自己位置を置換する構成から、連続した LIO と
品質を判定した GNSS を融合する構成へ変更する。

| 経路 | 役割 |
| --- | --- |
| Mid-360 点群 + IMU → FAST-LIO | 高頻度で連続した局所 odometry を推定 |
| 車輪 odometry | 低速域の補助、LIO の監視。滑りと誤差相関を考慮 |
| GNSS FIX + dual heading | グローバル位置・方位の初期整合と低周波の drift 補正 |
| GNSS FLOAT | 共分散拡大と innovation 検査。大きな偏りは融合対象から除外 |
| 融合済み pose → /localization/pose_enu | 既存走行スタックへの単一の自己位置出力 |
| LIO 異常 + GNSS 不良 | 続行せず減速・停止する品質監視 |

連続な odom→base_link と、GNSS 補正を受ける map→odom を分ける。
/rtk_gps/fix と GNSS 由来 pose を同時に独立観測として二重融合しない。
LIO と同じ IMU を別推定器で独立観測として融合する場合も相関を検討する。
LIO 初期座標の yaw と ENU の yaw、アンテナの lever arm を校正する。
GNSS 復帰時は即座に位置を飛ばさず、残差検査と連続 FIX の確認を経て補正する。

[FAST-LIO 公式](https://github.com/hku-mars/FAST_LIO) は LiDAR / IMU の時刻同期と
点ごとの時刻を重要な前提とする。FAST-LIO 自体に GNSS 融合やグローバル位置保証があるとは
解釈しない。融合基盤として robot_localization の EKF と
[navsat_transform_node](https://docs.ros.org/en/jade/api/robot_localization/html/navsat_transform_node.html)
の構成を参照できるが、この参照は旧版概念説明であり Jazzy の具体設定は導入時に再確認する。
既存 geo_pose_converter を使う場合も、ENU pose を融合する adapter と品質 gate が必要である。

## 5. 現状の FAST-LIO 接続を阻むもの

ローカル src/FAST_LIO の mid360.yaml は lidar_type=1 で CustomMsg、/livox/lidar と
/livox/imu を要求する。現行 Gazebo 入力は /mid360/livox/lidar/points の PointCloud2 で、
IMU sensor はまだない。GNSS heading 用 Imu は角速度・加速度が無効と明示されており、
これを FAST-LIO の IMU として渡してはならない。

このローカル版は lidar_type=4 が MID360 handler で、上流のシミュレータ例の「4」を
そのまま流用できない。MID360 handler は line / reflectivity 等を要求する。
汎用 handler は XYZI の瞬時点群を扱うが点時刻をゼロにするため、実機の走査歪みの評価にならない。

先に、Gazebo IMU（重力・角速度・加速度・バイアス・ノイズ）、/clock と use_sim_time、
点群の field とセンサ姿勢、点時刻、Mid-360 の走査近似を整える必要がある。
FAST-LIO 出力は /Odometry、frame_id=camera_init、child_frame_id=body なので
map/odom/base_link と取付高 0.6 m の整合が必要である。
local laserMapping.cpp は covariance を publish 後に設定しており、融合利用前に
同一時刻の covariance が届くようレビュー・修正する必要がある。
今回は未追跡の FAST_LIO ソースを変更せず、融合・FAST-LIO 実走評価は行っていない。

## 6. 次の受け入れ条件

1. 緯度経度ウェイポイントを同じ projection で読み、固定経路の追従・停止を満たす。
2. 障害物を検知し、AVOIDING への遷移、接触なし、復帰、ゴール停止をそれぞれ確認する。
3. GNSS FLOAT でも LIO 併用時に経路ずれが許容値内であることを真値で比較する。
4. LIO 退化・点群欠測・IMU 異常と GNSS 不良の同時発生時に停止する。
5. 開けた場所で再 FIX した際、位置ジャンプや誤った回避指令を生じない。

全ルート、複数 seed、誤差強度、建物高さ、実機 rosbag で範囲を広げる。
今回の調整モデルは物体を人為的に移動・削除しているので、その通過成功は現地通過の証明にならない。

## 7. 緯度経度ウェイポイントと障害物の実行結果

| ケース | ゴール真値誤差 | 状態 | 接触 |
| --- | ---: | --- | ---: |
| 緯度経度正本、通常 FIX | 0.375 m | RUNNING → FINISHED、停止 | 0 |
| 緯度経度正本、0.6 × 0.6 × 0.8 m 障害物 | 0.313 m | RUNNING → AVOIDING → RUNNING → FINISHED、停止 | 0 |

障害物は waypoint 10 上に配置した。回避時の最大経路ずれは 0.927 m、
真値中心から障害物矩形までの最小記録距離は 0.538 m だった。
車体形状全体の連続最小離隔ではないが、車体 contact 通知 0 と併せて判定した。
回避時に経路を外れるのは意図された挙動である。歩行者や動く障害物は今回対象外である。
結果は log/codex/waypoint_llh_eval/{fixed,obstacle}/ に保存する。
２ケースは DDS domain 87 / 88 と個別 Gazebo partition に分離して同時実行したため、
wall 時間による速度性能の比較は行わない。ENU 比較の通常／FLOAT は逐次実行した。
GNSS の注入周期・遅延・bias 時定数は wall 時間基準のため、実時間係数による差も残る。

全ケースで起動した子プロセスを回収し、DDS domain 86 / 87 / 88 の node list は空であった。
関連 pytest は 54 passed。通常 install と symlink install が成功した。
資料は既存の全域デジタルツイン記録と GNSS 検証記録を参照し、詳細設計書 10.6 と README
に入口を追加する。コードの parameter、生成 CSV、projection、状態ログを対応付けて確認した。
FAST-LIO のコード変更・ビルド・ROS 実行は対象外で、5 章の前提不足を未確認事項として残す。

## 8. 再現手順

ROS Jazzy と検証用 overlay、Gazebo runtime を有効化して実行する。
既存の試験結果を上書きしない出力先を指定する。

```bash
python3 src/obstacle_route_sim/tools/prepare_waypoint_evaluation.py --source log/codex/tc2026_corridor_1m --output <新しい試験出力先>
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <新しい試験出力先>/fixed --gnss --timeout 120
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <新しい試験出力先>/float --gnss --building-gnss --timeout 120
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <新しい試験出力先>/obstacle --gnss --timeout 125
```

試験ツールは合格条件を満たさない場合に終了コード 1 を返す。プロセス異常の有無と
result.json の個別指標を読んで、タイムアウトと機能失敗を区別する。
今回の実測 FLOAT 結果は ENU CSV・85 秒の条件であり、上の再現例の LLH・120 秒とは
時間・入力形式が異なる。誤差の傾向と新しい結果は別の実験として評価する。

## 9. 改版履歴

- 2026-09-13: 初版。局所追従、固定障害物回避、FLOAT 誤差評価、FAST-LIO 接続前提を記録。
