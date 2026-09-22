# FAST-LIO シミュレータ接続・検証

## 1. 対象と到達点

2026-09-13、i-Cart mini の調整版地理モデル上で FAST-LIO を実際に起動し、
Gazebo が生成した LiDAR と IMU から /lio/odometry を推定する構成を追加した。
真値 odometry や GNSS を FAST-LIO へ入力していない。
約 40 m の既存 waypoint 走行中に LIO を並行実行し、時刻を合わせた真値で評価する。
現在の走行制御は GNSS 自己位置を用い、LIO の出力は比較計測用である。
GNSS 融合や LIO による単独走行制御を実装済みとは扱わない。

参照した資料は既存の地理 waypoint / FLOAT 評価、全域デジタルツイン記録、
ローカル FAST_LIO の preprocess.cpp、IMU_Processing.hpp、laserMapping.cpp である。

## 2. 構成・配置

| ファイル | 役割 |
| --- | --- |
| tools/prepare_fastlio_trial.py | 元 world を保持し、IMU と LIO 用 LiDAR 設定を持つ別 world を生成 |
| tools/lio_sensor_adapter_node.py | 整定待機、非有限点と範囲外点の除去。入力 stamp を保持 |
| params/fastlio_gazebo.yaml | このローカル FAST-LIO 版用の設定 |
| launch/fastlio_sim.launch.py | Gazebo、clock/センサ bridge、adapter、FAST-LIO の起動 |
| tools/evaluate_terrain_trial.py | 既存追従試験と LIO の並行起動、記録、独立した精度判定 |
| tools/lio_evaluation_core.py | 初回姿勢の座標整合と同時刻真値比較 |
| tools/plot_lio_evaluation.py | 誤差 JSON と PNG の生成 |
| tests/test_lio_simulator.py | IMU 配置、点群除外、整定、座標整合、失敗判定 |

パッケージ固有の開発・検証機能を tools、起動を launch、設定を params、記録を docs に置く。
既存 make_robot、実機 Mid-360 設定、FAST_LIO ソースは変更しない。
生成物は log/codex/fastlio_trial 配下で管理し、Git に追加しない。
実行依存は sensor_msgs_py と fast_lio を含む。FAST-LIO と Livox driver/SDK は
ワークスペースの submodule revision を使用する。再帰取得後、ルートから
`colcon build --symlink-install --packages-up-to fast_lio` で構築する。
SDK は `livox_sdk2_vendor` が install prefix に配置する。

## 3. センサと interface

| topic | 型 | 内容 |
| --- | --- | --- |
| /clock | rosgraph_msgs/Clock | Gazebo 時刻 |
| /scan | sensor_msgs/LaserScan | 独立した Top-URG 相当の 2D スキャン、20 Hz |
| /mid360/livox/lidar/points | sensor_msgs/PointCloud2 | Gazebo の 10 Hz 点群 |
| /sim/mid360/imu | sensor_msgs/Imu | Gazebo の 200 Hz IMU |
| /sim/lio/points | sensor_msgs/PointCloud2 | 有限な XYZI、有効距離 0.6〜70 m |
| /sim/lio/imu | sensor_msgs/Imu | 整定後 IMU。header stamp を維持 |
| /lio/odometry | nav_msgs/Odometry | FAST-LIO /Odometry の remap |

adapter の購読は SensorDataQoS、点群配信は reliable depth 10、IMU 配信は reliable depth 100。
FAST-LIO は use_sim_time=true。ソフトによる時刻合わせは無効にする。
初回センサ時刻から 3 秒間は配信しない。評価ツールは wall 8 秒と LIO 10 件を待って走行を始め、
追加 30 秒の期限内に初期化できなければ終了する。
点群内 timestamp を捏造せず、XYZI の瞬時点群として扱う。

IMU は LiDAR と同位置・同方向、モデル基準 x=0、y=0、z=0.6 m に配置する。
角速度ノイズ σ=0.0002 rad/s、加速度ノイズ σ=0.002 m/s²、ゼロ平均 Gaussian を各軸へ設定した。
Gazebo IMU の重力を含む specific force を使用し、静止整定後の z 約 9.8 m/s² を確認した。
GNSS heading 用 Imu は使用しない。取付誤差・温度 bias・時刻ずれはこの条件に含めていない。

## 4. FAST-LIO 設定

このローカル版は lidar_type=0 で汎用 XYZI handler を選ぶ。
lidar_type=4 は MID360 の line / reflectivity 等を要求する handler なので使わない。
点ごとの offset は全てゼロであり、走査歪みのある実機点群の再現ではない。
LiDAR/IMU 外部パラメータは同位置なので T=0、R=I、オンライン外部校正は無効である。
voxel は 0.25 m、繰返し最大 4 回、map cube 200 m、PCD 自動保存は無効にする。

LIO 専用 world の LiDAR を水平 1,800 × 垂直 32 サンプル、最大距離 70 m とする。
これは規則格子の模擬観測で、Mid-360 の非反復走査・材質別反射・雨・遮蔽欠測の完全再現ではない。
70 m 以内なら常に観測できる仮定であり、実機の最大検出距離や測距性能の保証ではない。

## 5. 試験結果と修正理由

1. 整定待機なしの初回は、落下中のほぼゼロ加速度で重力初期化され、推定位置が大きく発散した。
2. 3 秒整定と非有限点除去後は初期発散を解消したが、30 m / 360×16 の点群では
   長い壁沿いの前進量を過小評価した（XY RMSE 15.76 m、最大 31.83 m）。
3. 30 m / 1800×32 では改善したが、なお XY RMSE 11.97 m、最大 25.05 m で不合格だった。
4. 70 m / 1800×32 では、約 40.06 m の区間で XY RMSE 0.0609 m、最大 0.1209 m、
   終端 0.0767 m だった。401 件、54.5 秒の同時刻比較で試験基準を満たした。

遠方の形状を含めると改善したことから、近傍の壁面だけでは移動方向の拘束が弱い可能性がある。
厳密な退化解析・Hessian の評価は未実施であり、原因を一意に断定しない。
密度の変更も CPU 負荷・処理頻度に影響する。
GNSS による経路追従の goal_pass と、LIO の pass_lio を別々に記録する。
LIO 基準は XY RMSE <0.2 m、最大 <0.5 m、比較時間 10 秒以上、移動 10 m 以上である。
単に /lio/odometry が出たことや GNSS がゴールへ到達したことを LIO 合格にしない。

評価では記録真値を LIO 時刻へ線形補間する。初回 XY と yaw だけで座標を合わせ、
軌跡全体による最適化は行わない。この初期整合は評価用であり、真値を推定器へ戻さない。
真値は車軸モデル位置、LIO は IMU 位置である。roll/pitch による lever arm の厳密な
補正を行っていないため、この比較は主に平面走行の相対精度を評価する。
全 2.224 km、長時間 drift、実機 rosbag、動的物体、LIO 再初期化は未確認である。

## 6. 利用手順

ROS Jazzy、現在の obstacle_route_sim overlay、fast_lio、Gazebo runtime を有効にする。
IMU 未追加の waypoint 試験フォルダから別の出力先を作る。

```bash
python3 src/obstacle_route_sim/tools/prepare_fastlio_trial.py --source log/codex/waypoint_llh_eval/fixed --output <新しいLIO試験フォルダ>
ros2 launch obstacle_route_sim fastlio_sim.launch.py world:=<新しいLIO試験フォルダ>/trial.sdf domain_id:=86 noise_profile:=reference
```

launch は走行指令を出さず、仮想センサと推定器を起動する。長時間起動した場合は利用者が終了する。
別の走行ノードを接続する場合は同じ DDS domain を使い、実機 domain と混在させない。

期限付き waypoint 走行・評価は以下を使う。

```bash
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <新しいLIO試験フォルダ> --gnss --fastlio <fast_lioの実行ファイル> --timeout 150 --lio-noise-profile reference
python3 src/obstacle_route_sim/tools/plot_lio_evaluation.py <新しいLIO試験フォルダ>
```

FAST-LIO 設定は --fastlio-config で変更できる。評価ツールは設定を出力先に保存する。
/clock、raw センサ、adapter、LIO を起動してから既存走行スタックを起動する。
結果は result.json、lio_trajectory.csv、lio_result.json、lio_comparison.png で確認する。
基準未達は終了コード 1。失敗時も起動した process group を回収する。

## 7. GNSS 融合へ進む前の課題

LIO の frame_id=camera_init、child_frame_id=body を map/odom/base_link と整合させる必要がある。
GNSS FIX で初期方位・位置を求め、FLOAT を品質 gate で除外または弱く融合する。
今回の真値による初期整合を実機の初期化として使うことはできない。
ローカル FAST-LIO は covariance を publish 後に設定するため、融合時には同一時刻の
covariance を出力するよう修正・検証する必要がある。今回そのソース変更は行っていない。
IMU や点群の途絶・時刻逆行・退化の検知、悪化時停止、再 FIX 時の連続補正も別途必要である。

## 8. 検証・改版

- 2026-09-13: 初版。センサ追加、入力 adapter、launch、時刻整合による精度判定を実装。
- 関連 pytest 59 件成功。obstacle_route_sim の通常／symlink ビルド成功。
- 既存詳細設計書に 10.7 を追加し、README に起動・検証の入口を記載する。
- ビルド時は既存 install の fast_lio / livox_ros_driver2 を利用する旨の警告が出るが成功する。
  今回はこれらのソースを変更しておらず再ビルドは対象外である。
- launch を domain 89、独立 GZ_PARTITION で 20 秒起動し、点群受信、IMU 初期化、
  map kdtree 初期化を確認した。終了時の adapter の二重 shutdown を try_shutdown で修正し、
  再試験では Gazebo、bridge、adapter、FAST-LIO の全 4 プロセスが正常終了した。
  記録は log/codex/fastlio_trial/launch_runtime_verified.txt に保存する。
- 最終確認は pytest src/obstacle_route_sim/tests src/robot_navigator/tests
  src/geo_pose_converter/tests が 59 件成功し、対象パッケージの通常／symlink ビルドも成功した。

## 9. GNSS FLOAT 中の LIO 独立推定

最終設定（70 m・1800×32、IMU、入力整形）で建物近傍 FLOAT を同時実行した。
FLOAT 805 件、LIO 551 件、raw IMU 15,826 件を記録した。
LIO は比較時間 75.4 秒、移動約 38.41 m、XY RMSE 0.0613 m、最大 0.1148 m、
終端 0.0960 m で LIO 基準を満たした。
GNSS 制御の route_follower は FINISHED になったが、真値のゴール誤差は 2.379 m で、
goal_pass=false と判定された。全体終了コード 1 はこの走行精度未達を正しく示す。
LIO に GPS は入力されておらず、LIO が良好でも走行側へ融合しなければ改善しないことを示す。
この結果を GPS+LIO 融合走行成功と解釈しない。
記録は log/codex/fastlio_trial/float/ に保存する。

## 10. URG 2D スキャンと障害物回避

Top-URG は Mid-360 の投影ではなく、別の Gazebo GPU LiDAR である。
車軸基準 (0.075, 0, 0.300) m、roll/pitch/yaw=0 とし、前後位置 75 mm は仮定である。
水平約 270 度、1,080 点、20 Hz、距離 0.2〜30 m、垂直走査なしとする。
機種の正確な型番が未確定なので、これを実機の仕様保証とは扱わない。
追加した fastlio_sim.launch.py に不足していた /scan の LaserScan bridge を補った。
既存評価ツールの /scan → obstacle_monitor → obstacle_avoidance_hint → route_follower
という接続を維持し、FAST-LIO と同時に回避試験できるようにする。
launch 単体はセンサ・推定器のみを起動し、回避走行には評価ツール等の走行ノードが必要である。

評価ツールは scan の frame、角度、距離、点数と hint / front_blocked 件数を result.json に保存する。
最初の front_blocked を受けた直近スキャンを urg_blocked_scan.json に保存する。
これは非同期購読であり hint とスキャンの同一時刻を保証しない。
tools/plot_urg_evaluation.py は実受信点と真値回避軌跡を urg_evaluation.png に出力する。
開発・検証用可視化なので tools に配置する。

```bash
python3 src/obstacle_route_sim/tools/prepare_fastlio_trial.py --source log/codex/waypoint_llh_eval/obstacle --output <新しいURG試験フォルダ>
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <新しいURG試験フォルダ> --gnss --fastlio <fast_lioの実行ファイル> --timeout 150 --lio-noise-profile reference
python3 src/obstacle_route_sim/tools/plot_urg_evaluation.py <新しいURG試験フォルダ>
```

高さ 0.8 m・幅奥行 0.6 m の静止箱を既存 waypoint 上に置いて評価する。
この試験は高さ 0.3 m の走査面に交差する物体の回避を扱う。
低い段差・穴・走査面より高い張り出しは検出できない場合がある。
材質別反射、雨、ガラス、測距ノイズ、点ごとの走査時刻・歪みは未再現である。
obstacle_monitor はセンサ座標を用い、取付前後位置の厳密な補正は未実施である。
接触判定の対象は従来どおり車体・支柱・アンテナで、車輪・キャスタは含まない。

2026-09-13 の同時走行試験（log/codex/fastlio_trial/urg_obstacle）では、
スキャン 2,071 件、hint 1,963 件、front_blocked 699 件を記録した。
初回の前方余裕は 0.985 m、RUNNING → AVOIDING → RUNNING → FINISHED へ遷移し、
ゴール誤差 0.336 m、監視対象の接触 0 件、停止を確認し goal_pass=true となった。
初回障害スキャンに近い真値記録は wall 37.886 秒、AVOIDING は wall 75.230 秒であり、
回避開始まで約 37 秒かかった。即時回避・運用上妥当な待機時間を保証する試験ではない。
LIO も並行して XY RMSE 0.0317 m、最大 0.2137 m、pass_lio=true となった。
本試験の自己位置制御は GNSS FIX であり、LIO 融合走行ではない。
関連 pytest は 60 件成功し、通常／symlink ビルドも成功した。
インストール済み launch でも domain 89 の直接購読で /scan を受信し、
frame=icart_mini/base_link/top_urg、1,080 点を確認した。
記録は launch_scan_sample_verified.json と launch_scan_verified.txt に保存する。
