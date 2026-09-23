# obstacle_route_sim 詳細設計書

## 1. 文書目的・対象範囲

本書は、tc2025 の `lidar_obstacle_sim` を tc2026 ワークスペースへ移植した
`obstacle_route_sim` パッケージの設計を記録する。対象は Ubuntu 24.04、ROS 2 Jazzy、
Gazebo Harmonic、`ros_gz` を前提とする障害物付き経路追従検証環境である。

Gazebo Classic、`gazebo_ros_pkgs`、`gazebo_msgs/srv/SpawnEntity`、Classic plugin は
移植対象から除外し、Harmonic の SDF system plugin と `ros_gz_bridge` で置き換える。

## 2. 背景・要求・スコープ

本パッケージは、道路コース、差動二輪ロボット、2D LiDAR、3D LiDAR、odometry、
fake localization pose、waypoint CSV 生成、pylon 障害物配置を提供し、既存の
`route_planner`、`route_manager`、`route_follower`、`robot_navigator`、
`obstacle_monitor`、`drive_mode_manager` を Gazebo 上で接続できる状態にする。

自己位置推定精度や Livox Mid-360 の非反復走査特性を厳密に再現することは非スコープとする。
`fake_pose_enu.py` は Gazebo 上の robot モデル真値 pose を `/localization/pose_enu` として配信し、
localization 誤差がない前提で route stack を評価する。

## 3. 全体構成・アーキテクチャ

`sim_obstacle_route.launch.py` は Gazebo 単体確認用の起動入口である。launch は道路種別と道幅から world template を選び、
robot include と seed 指定 pylon include を追加した一時 world を生成し、`ros_gz_sim` で起動する。
`gazebo_obstacle_route_stack.launch.py` は docs の 10.3 構成に合わせた統合確認用 launch であり、
Gazebo、bridge、fake localization pose、TF、`route_planner`、`route_manager`、`route_follower`、
`obstacle_monitor`、`robot_navigator`、`ps3_joy_sim_node`、`manual_teleop_node`、
`drive_cmd_mux_node`、任意の drive status GUI を同時起動する。

主要構成は以下とする。

| 要素 | 役割 |
| --- | --- |
| `worlds/templates/*.world` | Gazebo Harmonic の base world |
| `models/robot/model.sdf` | DiffDrive、2D LiDAR、3D LiDAR を持つ差動二輪ロボット |
| `models/pylon/model.sdf` | 静的 pylon 障害物 |
| `scripts/road_geometry.py` | 道路中心線と幅キーの共通定義 |
| `scripts/generate_pylon_world.py` | 起動前 world 生成 |
| `scripts/generate_waypoints.py` | `route_planner` 互換 waypoint CSV 生成 |
| `tools/generate_route_assets.py` | 各 world 用 route CSV と `route_planner` / `route_manager` params 生成 |
| `launch/gazebo_obstacle_route_stack.launch.py` | Gazebo と route stack の統合確認 launch |
| `scripts/fake_pose_enu.py` | Gazebo 真値 pose から `/localization/pose_enu` 互換 pose を生成 |
| `scripts/odom_tf_broadcaster.py` | bridged odom から `odom -> base_link` TF を publish |

## 4. 外部インタフェース仕様

既存 route stack との接続を優先し、ROS 側 topic は互換名を基本とする。

| Topic | Type | 方向 | 内容 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 2 -> Gazebo | DiffDrive 速度指令 |
| `/ypspur_ros/odom` | `nav_msgs/msg/Odometry` | Gazebo -> ROS 2 | Gazebo wheel odometry |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo -> ROS 2 | 2D LiDAR |
| `/mid360/livox/lidar/points` | `sensor_msgs/msg/PointCloud2` | Gazebo -> ROS 2 | Gazebo native GPU LiDAR 点群 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> ROS 2 | simulation time |
| `/gazebo/dynamic_pose_info` | `tf2_msgs/msg/TFMessage` | Gazebo -> ROS 2 | Gazebo dynamic entity pose info |
| `/localization/pose_enu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | ROS 2 | Gazebo 真値に基づく fake localization pose pose |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | ROS 2 | frame tree |

bridge は topic 所有者を明確にするため方向指定を使う。`/cmd_vel` は ROS 2 -> Gazebo のみ、
`/clock`、`/ypspur_ros/odom`、`/scan`、`/mid360/livox/lidar/points`、
`/world/<world_name>/dynamic_pose/info` は Gazebo -> ROS 2 のみとする。
Gazebo dynamic pose info は ROS 側で `/gazebo/dynamic_pose_info` へ remap し、`fake_pose_enu.py` が
先頭の robot model pose を抽出して `/localization/pose_enu` を publish する。3D LiDAR は Gazebo の sensor base topic ではなく `/mid360/livox/lidar/points` を
`gz.msgs.PointCloudPacked` として bridge する。

`robot_navigator` を統合する場合は、既存方針どおり `robot_navigator` の出力を
`/cmd_vel/autonomous` とし、`drive_cmd_mux_node` が最終 `/cmd_vel` を publish する。

## 5. パラメータ・launch 設計

`sim_obstacle_route.launch.py` は以下の launch 引数を持つ。

| 引数 | 既定値 | 内容 |
| --- | --- | --- |
| `road_type` | `crank` | `straight`, `crank`, `scurve` |
| `road_width` | `5.0` | `straight` は 2/3/5m、`crank` と `scurve` は 3/5m |
| `enable_pylons` | `true` | pylon include 生成の有無 |
| `pylon_seed` | `0` | pylon 配置の再現 seed |
| `use_sim_time` | `true` | ROS ノードの simulation time 使用 |
| `spawn_robot` | `true` | robot include 生成の有無 |
| `robot_x`, `robot_y`, `robot_z` | `1.0`, `0.0`, `0.16` | robot 初期 pose |
| `generated_world_dir` | `/tmp/obstacle_route_sim` | 生成 world 出力先 |
| `start_gazebo_gui` | `true` | Gazebo GUI 起動の有無 |

`keep_generated_world` と `use_compat_remap` は宣言されるが、挙動の切替には使われない。
トピック名は `/scan`、`/cmd_vel`、`/ypspur_ros/odom` へ直接bridgeする。

`gazebo_obstacle_route_stack.launch.py` は `sim_obstacle_route.launch.py` の引数に加え、以下を持つ。

| 引数 | 既定値 | 内容 |
| --- | --- | --- |
| `start_drive_status_gui` | `true` | drive status GUI 起動の有無 |
| `stack_use_sim_time` | `false` | route stack 側 ROS ノードの simulation time 使用 |
| `route_data_dir` | `/tmp/obstacle_route_sim/routes` | 生成 waypoint CSV と route config 出力先 |
| `enable_route_blocker` | `true` | 経路中心線上の決定的 blocker pylon 生成の有無 |
| `route_blocker_distance` | `8.0` | blocker pylon を置く経路始点からの距離 [m] |

統合確認 launch では、Gazebo/bridge/fake localization pose は simulation time を扱う一方、route stack 側は
初期 route request timer の停止を避けるため既定で wall time とする。

`tools/generate_route_assets.py` は、tc2025 の `generate_waypoints.py` 相当の waypoint 生成を
route stack で直接選択できる資産へ展開する開発ツールである。既定では以下の 7 world 分を生成する。

| route id | road_type | road_width | waypoints | start_label | goal_label |
| --- | --- | --- | --- | --- | --- |
| `straight_w2` | `straight` | `2.0` | 21 | `0` | `20` |
| `straight_w3` | `straight` | `3.0` | 21 | `0` | `20` |
| `straight_w5` | `straight` | `5.0` | 21 | `0` | `20` |
| `scurve_w3` | `scurve` | `3.0` | 28 | `0` | `27` |
| `scurve_w5` | `scurve` | `5.0` | 28 | `0` | `27` |
| `crank_w3` | `crank` | `3.0` | 33 | `0` | `32` |
| `crank_w5` | `crank` | `5.0` | 33 | `0` | `32` |

生成物は `route_planner/routes/obstacle_route_sim/<route_id>/`、
`route_planner/params/obstacle_route_<route_id>.yaml`、
`route_manager/params/obstacle_route_<route_id>.yaml` に配置する。
`route_planner` 側の path は `route_planner` package share からの相対 path とし、
リポジトリ固有の絶対 path を params に持たせない。`robot_console` は各 package の `params` を
起動カードの候補として自動検出するため、`obstacle_route_<route_id>.yaml` を
`route_planner` と `route_manager` の両方で選択すれば、該当 world の start-to-goal route を使用できる。

## 6. データモデル・処理フロー

道路中心線は `road_geometry.py` を正とし、道路モデル生成、waypoint 生成、pylon 配置で共用する。
waypoint CSV は `route_planner` の現行 parser が扱う列に合わせ、simulation では
`latitude` と `longitude` を空欄、`node=-1` とする。統合確認 launch は起動時に
`route_data_dir` 配下へ Gazebo 道路中心線と一致する `route_config.yaml` と CSV を生成し、
`route_planner` と `route_manager` へ一時 parameter YAML として渡す。

pylon は Gazebo 起動後に service spawn せず、起動前に world へ `<include>` を追加する。
同一 seed では同一配置となり、原点 5m 以内には配置しない。

ランダム pylon は障害物回避イベントを必ず発火させるものではないため、統合確認 launch は既定で
`enable_route_blocker=true` とし、経路中心線上の `route_blocker_distance` に pylon を 1 本追加する。
これにより `/obstacle_avoidance_hint.front_blocked`、`route_follower` の `AVOIDING` 遷移、
障害物通過後の `RUNNING` 復帰を再現性を持って確認できる。

## 7. TF・タイミング設計

frame 構成は以下とする。

```text
map
└── odom
    └── base_link
        ├── laser
        └── mid360_frame
```

`map -> odom` は初期段階では static zero transform とする。`odom -> base_link` は
`/ypspur_ros/odom` を `odom_tf_broadcaster.py` が TF 化する。`/localization/pose_enu` は TF ではなく
Gazebo dynamic pose info の先頭 robot model pose から生成し、covariance は 0 とする。
`base_link -> laser` と `base_link -> mid360_frame` は SDF link pose と同じ値を static TF として publish する。

## 8. エラー処理・ログ・診断

`fake_pose_enu.py` と `odom_tf_broadcaster.py` は起動時に購読・配信 topic を info ログへ出す。
`fake_pose_enu.py` は Gazebo dynamic pose info 内に対象 index が見つからない場合、初回と
一定周期で warn ログを出す。world 生成時に base world が不正な場合は例外を出して launch を失敗させる。
Gazebo や bridge の runtime エラーは各プロセスの標準出力で確認する。

Gazebo native 3D LiDAR の PointCloud2 が出ない場合は、bridge topic と型を最初に確認する。
GPU LiDAR の sensor base topic は scan 相当であり、点群は `.../points` topic に
`gz.msgs.PointCloudPacked` として出る。したがって bridge は
`/mid360/livox/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked` とする。
world template では `<render_engine>ogre2</render_engine>` を明示し、headless/GPU 権限差で
レンダリング backend が変わっても sensor 出力が不安定になりにくい構成とする。

## 9. 依存関係・ビルド設定

本パッケージは `ament_cmake` とし、Python script は `install(PROGRAMS)` で
`lib/obstacle_route_sim` へ配置する。Gazebo 連携は `ros_gz_sim` と `ros_gz_bridge` に依存する。
Classic 系依存は追加しない。

## 10. テスト計画・受け入れ条件

自動テストでは ROS/Gazebo 実行に依存しない `road_geometry.py`、waypoint CSV 生成、
pylon world 生成を確認する。

受け入れ条件は以下である。

- `pytest src/obstacle_route_sim/tests` が成功する。
- `/usr/bin/python3 -m pytest src/obstacle_route_sim/tests src/robot_console/tools/tests src/route_planner/tests` が成功する。
- `colcon build --symlink-install --packages-select obstacle_route_sim` が成功する。
- `colcon build --packages-select obstacle_route_sim` が成功する。
- `colcon build --symlink-install --packages-select obstacle_route_sim route_planner route_manager route_follower robot_navigator obstacle_monitor` が成功する。
- `colcon build --packages-select obstacle_route_sim route_planner route_manager route_follower robot_navigator obstacle_monitor` が成功する。
- ローカル ROS 実行確認時に Gazebo world が起動し、`/scan`、`/mid360/livox/lidar/points`、
  `/ypspur_ros/odom`、`/localization/pose_enu`、TF が確認できる。
- `gazebo_obstacle_route_stack.launch.py` で `/active_route`、`/active_target`、`/cmd_vel/autonomous`、
  `/cmd_vel`、`/drive_mode_status` が確認でき、`/cmd_vel` の publisher が `drive_cmd_mux_node` のみである。
- GUI 確認時に robot model 上の門型支柱・梁の上へ Mid-360 が配置され、センサだけが浮いて見えない。

### 10.4 地理地形とGNSS/LIO融合

terrain3dの生成・車体前提は[試験環境](terrain3d_icart_mini検証.md)、
地理入力は[全域モデル](つくば2026全域デジタルツイン.md)、
LiDAR/IMU接続は[FAST-LIO入力](FASTLIOシミュレータ接続検証.md)を参照する。
GNSS単独・FAST-LIO比較・融合走行は異なる構成で、--fusion指定時に融合を制御へ使う。
通常の実機／模擬共通起動はicart_bringupが担当する。

### 10.5 地図とセンサ誤差

--record-lio-mapで登録済み点群を分割保存する。
[地図評価](全域FASTLIO点群地図評価.md)は推定地図の整合、
[誤差モデル](FASTLIOセンサ誤差モデル評価.md)はLIOへの雑音・欠測・bias注入を説明する。
共通デジタルツインは歩行者の模擬と開始地点のFIX区域も設定できる。

## 11. 制約

基本のsim_obstacle_routeは真値由来poseによる経路試験であり、融合精度の試験ではない。
模擬Mid-360は規則格子点群を使い、非反復走査・点内時刻・材質反射を完全には再現しない。
シナリオとセンサ設定は使用するlaunch・trial・sessionで確認する。
