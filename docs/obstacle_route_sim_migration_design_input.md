# `obstacle_route_sim` 移植設計インプット資料

## 1. 目的

既存の `lidar_obstacle_sim` パッケージを、Ubuntu 24.04 + ROS 2 Jazzy 環境へ移植する。

移植後のパッケージ名は **`obstacle_route_sim`** とする。

本パッケージは、単なる LiDAR 障害物シミュレータではなく、以下を含む **障害物付き経路追従検証用 Gazebo シミュレーションパッケージ**として再構成する。

- 道路コース生成
- Gazebo world 起動
- ロボットモデル
- 2D LiDAR
- 3D LiDAR
- 差動二輪駆動
- odometry
- fake localization pose
- waypoint CSV 生成
- パイロン障害物配置
- TF
- launch
- 既存経路追従・障害物回避ノードとの接続

Ubuntu 24.04 + ROS 2 Jazzy では Gazebo Harmonic を前提とし、`ros_gz` 系パッケージによる ROS 2 / Gazebo 連携構成へ移行する。

---

## 2. 移植方針

既存実装は Gazebo Classic と `gazebo_ros_pkgs` を前提としているため、Ubuntu 24.04 + ROS 2 Jazzy ではそのまま延命しない。

移植後は以下を前提とする。

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
ros_gz_sim
ros_gz_bridge
ros_gz_interfaces
```

既存の Gazebo Classic 依存は廃止する。

廃止対象の代表例は以下。

```text
gazebo
gazebo_ros
gazebo_ros_pkgs
gazebo_msgs/srv/SpawnEntity
spawn_entity.py
libgazebo_ros_init.so
libgazebo_ros_factory.so
libgazebo_ros_ray_sensor.so
libgazebo_ros_diff_drive.so
GAZEBO_MODEL_PATH
GAZEBO_PLUGIN_PATH
```

代わりに、Gazebo Harmonic / Gazebo Sim と `ros_gz` 系パッケージを用いる。

---

## 3. パッケージ名

移植後のパッケージ名は以下とする。

```text
obstacle_route_sim
```

理由:

- LiDAR だけでなく、経路・道路・障害物・ロボット・センサを含むため
- 障害物回避と経路追従の検証環境であることが明確
- `lidar_obstacle_sim` より機能範囲を正しく表現できる
- 将来的に LiDAR 以外のセンサや複数ルート検証にも拡張しやすい

---

## 4. 移植対象範囲

本移植では、既存パッケージ内の全機能を対象とする。

| 機能 | 移植対象 | 方針 |
|---|---|---|
| Gazebo world 起動 | 対象 | Gazebo Harmonic 用 launch へ置換 |
| 道路モデル | 対象 | SDF を Harmonic で動作確認・必要修正 |
| ロボットモデル | 対象 | Classic plugin を Harmonic 用 system / bridge 構成へ置換 |
| 2D LiDAR | 対象 | Gazebo topic → ROS 2 `/scan` 相当へ bridge |
| 3D LiDAR | 対象 | Gazebo point cloud → ROS 2 PointCloud2 へ bridge |
| 差動二輪駆動 | 対象 | Gazebo Sim DiffDrive 相当へ置換 |
| odom | 対象 | Gazebo odom → ROS 2 odom へ bridge |
| fake localization pose | 対象 | 基本流用、topic/frame は移植先構成に合わせる |
| waypoint CSV 生成 | 対象 | 基本流用、道路定義共通化 |
| パイロン生成 | 対象 | 起動前 world 生成方式へ変更 |
| TF | 対象 | frame 構成を整理し直す |
| launch | 対象 | 新規作成 |
| package / install 設定 | 対象 | Jazzy / ros_gz 依存へ更新 |
| docs | 対象 | 移植後設計を明文化 |

---

## 5. 既存実装の扱い

既存実装を完全に忠実移植するのではなく、明らかな不整合や移植先に合わない構成は修正する。

修正前提とする主な観点は以下。

```text
1. Gazebo Classic 依存の廃止
2. SDF link pose と static TF の不一致疑いの解消
3. road_width 表記と world/model ファイル名の不整合解消
4. 道路中心線定義の重複解消
5. pylon spawn service 依存の廃止
6. topic / frame / namespace の移植先構成への適合
7. fake localization pose が map = odom の簡易実装であることの明記
8. センサ frame_id と TF の整合性確認
```

---

## 6. トピック名・frame 名の方針

トピック名、frame_id、namespace、remap ルールは、移植先である tc2026 側のシステム構成に合わせて決定する。

既存 tc2025 のトピック名は、移植時の絶対条件ではなく、互換用 remap の参考値として扱う。

既存で使われていた代表的な topic は以下。

```text
/cmd_vel
/scan
/mid360/livox/lidar
/ypspur_ros/odom
/localization/pose_enu
/tf
/tf_static
```

移植後は、必要に応じて以下のような内部標準名と互換 remap を分離する。

```text
内部標準例:
  /sim/cmd_vel
  /sim/odom
  /sim/scan
  /sim/mid360/points

互換 remap 例:
  /cmd_vel
  /ypspur_ros/odom
  /scan
  /mid360/livox/lidar
```

設計上は、**既存 topic 名を固定仕様にしない**。

ただし、既存の経路追従・障害物検知ノードを無改修で検証する段階では、launch の remap により既存 topic 名へ合わせることを許容する。

---

## 7. 推奨ディレクトリ構成

移植後の構成案は以下。

```text
obstacle_route_sim/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── sim_obstacle_route.launch.py
│   └── bridge_gz.launch.py
├── scripts/
│   ├── fake_pose_enu.py
│   ├── generate_waypoints.py
│   ├── generate_pylon_world.py
│   ├── road_generator.py
│   └── road_geometry.py
├── models/
│   ├── robot/
│   │   ├── model.sdf
│   │   └── model.config
│   ├── pylon/
│   │   ├── model.sdf
│   │   └── model.config
│   ├── road_straight_100m_w2/
│   ├── road_straight_100m_w3/
│   ├── road_straight_100m_w5/
│   ├── road_crank_50m_w3/
│   ├── road_crank_50m_w5/
│   ├── road_scurve_100m_w3/
│   └── road_scurve_100m_w5/
├── worlds/
│   ├── templates/
│   │   ├── road_straight_w2.world
│   │   ├── road_straight_w3.world
│   │   ├── road_straight_w5.world
│   │   ├── road_crank_w3.world
│   │   ├── road_crank_w5.world
│   │   ├── road_scurve_w3.world
│   │   └── road_scurve_w5.world
│   └── generated/
│       └── .gitkeep
└── docs/
    ├── design.md
    └── jazzy_migration.md
```

`worlds/generated/` は、基本的には Git 管理対象外とし、`.gitkeep` のみ置く。

生成 world は `/tmp/obstacle_route_sim` へ出力する運用でもよいが、デバッグ性を考慮して launch 引数で出力先を指定可能にする。

---

## 8. `package.xml` 方針

Classic 依存を削除する。

削除候補:

```xml
<exec_depend>gazebo_ros</exec_depend>
<exec_depend>gazebo_ros_pkgs</exec_depend>
<exec_depend>gazebo_msgs</exec_depend>
```

追加・維持候補:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>

<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>tf2_msgs</exec_depend>
<exec_depend>rosgraph_msgs</exec_depend>

<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>

<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
<exec_depend>ros_gz_interfaces</exec_depend>
```

---

## 9. `CMakeLists.txt` 方針

基本方針は既存と同様に、launch、worlds、models、scripts を install する。

ただし、パッケージ名変更に伴い project 名を変更する。

```cmake
project(obstacle_route_sim)
```

install 対象は以下。

```cmake
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fake_pose_enu.py
    scripts/generate_waypoints.py
    scripts/generate_pylon_world.py
    scripts/road_generator.py
  DESTINATION lib/${PROJECT_NAME}
)
```

`road_geometry.py` を共通モジュールとして使う場合は、`PROGRAMS` ではなく通常ファイルとして install するか、Python package 化を検討する。

初期移植では、簡単さを優先して `scripts/` 配下に置き、各スクリプトから import 可能な構成を確認する。

---

## 10. launch 設計

メイン launch は以下とする。

```text
launch/sim_obstacle_route.launch.py
```

主な launch 引数:

```text
road_type:=straight|crank|scurve
road_width:=2.0|3.0|5.0
enable_pylons:=true|false
pylon_seed:=0
use_sim_time:=true
spawn_robot:=true
robot_x:=1.4
robot_y:=0.0
robot_z:=0.5
generated_world_dir:=/tmp/obstacle_route_sim
keep_generated_world:=true
use_compat_remap:=true
```

launch の役割:

```text
1. road_type / road_width から world template を選択
2. enable_pylons=true の場合、generate_pylon_world.py で一時 world を生成
3. Gazebo Harmonic を ros_gz_sim で起動
4. robot model を spawn
5. ros_gz_bridge を起動
6. fake_pose_enu.py を起動
7. static TF を起動
8. 必要に応じて互換 remap を適用
```

---

## 11. Gazebo resource path

Gazebo Classic の `GAZEBO_MODEL_PATH` は使用しない。

Gazebo Harmonic では、モデル・world 解決用に `GZ_SIM_RESOURCE_PATH` を設定する。

launch 内では以下を追加する。

```text
GZ_SIM_RESOURCE_PATH に obstacle_route_sim の share ディレクトリと models ディレクトリを追加する
```

例:

```text
<install-prefix>/share/obstacle_route_sim
<install-prefix>/share/obstacle_route_sim/models
```

---

## 12. world 設計

world は Gazebo Harmonic 用 SDF として整備する。

基本構成:

```xml
<sdf version="1.9">
  <world name="default">
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics"/>

    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"/>

    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"/>

    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors"/>

    <include>
      <uri>model://road_crank_50m_w5</uri>
    </include>
  </world>
</sdf>
```

`enable_pylons=true` の場合は、上記 world に pylon の `<include>` を追加した一時 world を生成する。

---

## 13. 道路モデル設計

既存の道路は維持する。

対象道路:

```text
straight:
  100m 直線
  width: 2m / 3m / 5m

crank:
  50m → 左折50m → 右折50m
  width: 3m / 5m

scurve:
  100m S字
  width: 3m / 5m
```

ただし、道路中心線定義は以下へ共通化する。

```text
scripts/road_geometry.py
```

共通化対象:

```text
get_polyline_points(road_type)
build_segments(points)
interpolate_pose_on_polyline(distance)
compute_total_length(points)
```

これにより、以下のズレを防ぐ。

```text
road_generator.py
generate_waypoints.py
generate_pylon_world.py
```

---

## 14. waypoint 生成設計

既存の `generate_waypoints.py` は基本的に流用する。

ただし、道路中心線は `road_geometry.py` から取得する。

仕様:

```text
中心線に沿って 5m 間隔で waypoint を生成
開始点と終了点は道路端から 1m 内側
25度以上折れ曲がる交点には waypoint を追加
yaw は quaternion に変換
right_is_open / left_is_open は道幅の半分を設定
```

出力 CSV の基本列は維持する。

```text
label
x
y
z
q1
q2
q3
q4
right_is_open
left_is_open
line_is_stop
signal_is_stop
isnot_skipnum
```

ただし、移植先 tc2026 側の route CSV 仕様が異なる場合は、そちらに合わせる。

---

## 15. パイロン生成設計

パイロン生成は案Aを採用する。

すなわち、Gazebo 起動後に spawn service で生成するのではなく、Gazebo 起動前に pylon を含む world を生成する。

新スクリプト:

```text
scripts/generate_pylon_world.py
```

役割:

```text
1. road_type / road_width / seed を受け取る
2. road_geometry.py から中心線を取得
3. 既存 random_pylon_spawner.py の配置ロジックを再利用
4. pylon 配置 pose を生成
5. base world template に pylon <include> を追加
6. generated world を出力
```

主な引数:

```text
--road straight|crank|scurve
--width 2.0|3.0|5.0
--seed 0
--base-world path
--output path
--min-longitudinal-spacing 5.0
--longitudinal-margin 1.0
--min-lateral-gap 1.0
```

パイロン配置仕様:

```text
道路中心線に沿ってランダム配置
各配置地点に1〜3本
center / spread / cluster 配置
道幅方向に最低1mの通過可能隙間を残す
原点周辺には配置しない
S字は配置密度を調整する
seed 指定により再現性を確保する
```

生成される pylon include 例:

```xml
<include>
  <uri>model://pylon</uri>
  <name>crank_pylon_001</name>
  <pose>12.300 1.200 0.000 0 0 0</pose>
</include>
```

---

## 16. ロボットモデル設計

既存 `simple_robot` は Gazebo Classic plugin 依存を含むため、移植後は新しいモデルとして再構成する。

推奨モデルディレクトリ:

```text
models/robot/
```

維持する概念:

```text
差動二輪
base_link
left_wheel_link
right_wheel_link
rear_caster_link
laser
mid360_frame
UTM-30LX相当 2D LiDAR
Mid-360相当 3D LiDAR
```

見直す項目:

```text
SDF link pose
センサ frame
static TF
DiffDrive plugin
LiDAR sensor plugin
odom 出力
topic 名
```

特に、SDF 上の link pose と ROS TF は一致させる。

SDF を正とし、TF をそれに合わせる方針とする。

---

## 17. 差動駆動設計

既存の `libgazebo_ros_diff_drive.so` は使用しない。

Gazebo Harmonic の DiffDrive system 相当を使用し、Gazebo topic と ROS 2 topic を `ros_gz_bridge` で接続する。

概念構成:

```text
ROS 2 cmd_vel
  ↓ ros_gz_bridge
Gazebo DiffDrive
  ↓
Gazebo odom
  ↓ ros_gz_bridge
ROS 2 odom
```

ROS 2 側 topic 名は移植先構成に合わせる。

互換例:

```text
/cmd_vel
/ypspur_ros/odom
```

内部標準例:

```text
/sim/cmd_vel
/sim/odom
```

---

## 18. 2D LiDAR 設計

既存の `/scan` 相当を維持可能とする。

Gazebo 側では lidar sensor を定義し、ROS 2 へ `sensor_msgs/msg/LaserScan` として bridge する。

既存仕様の参考値:

```text
視野: 約270度
角度: -135度 ～ +135度
range: 0.2m ～ 30m
samples: 1080
update_rate: 40Hz
```

ただし、移植後の最終値は、移植先ロボット構成・処理負荷・検証目的に合わせて調整してよい。

---

## 19. 3D LiDAR 設計

既存の Mid-360 相当センサを維持する。

Gazebo 側では 3D lidar / gpu lidar sensor を定義し、ROS 2 へ `sensor_msgs/msg/PointCloud2` として bridge する。

既存仕様の参考値:

```text
水平視野: 360度
垂直方向 samples: 24
range: 0.2m ～ 40m
update_rate: 10Hz
```

ただし、Gazebo の ray sensor は Livox Mid-360 実機の非反復スキャン特性を厳密再現するものではない。

本パッケージでは、3D LiDAR 点群処理ノードの動作確認用の近似センサとして扱う。

---

## 20. bridge 設計

`ros_gz_bridge` を用いて Gazebo topic と ROS 2 topic を接続する。

代表的な bridge 対象:

```text
/cmd_vel
/odom または /sim/odom
/scan
/mid360/points
/clock
```

方向:

```text
cmd_vel:
  ROS 2 → Gazebo

odom:
  Gazebo → ROS 2

scan:
  Gazebo → ROS 2

point cloud:
  Gazebo → ROS 2

clock:
  Gazebo → ROS 2
```

互換 topic を使う場合は launch 側で remap する。

---

## 21. fake localization pose 設計

既存 `fake_pose_enu.py` は基本流用する。

役割:

```text
odom を PoseWithCovarianceStamped に変換して publish する
```

ただし、これは本物の localization ではない。

設計書上では以下を明記する。

```text
fake_pose_enu は map = odom とみなす簡易ノードである。
Gazebo シミュレーションにおいて、既存ノードが /localization/pose_enu を要求する場合の互換用として使用する。
本格的な自己位置推定精度評価には使用しない。
```

---

## 22. TF 設計

基本 frame 構成:

```text
map
└── odom
    └── base_link
        ├── laser
        └── mid360_frame
```

方針:

```text
map -> odom:
  初期段階では static zero transform

odom -> base_link:
  Gazebo odom または odom 由来 TF

base_link -> laser:
  SDF link pose と一致

base_link -> mid360_frame:
  SDF link pose と一致
```

注意点:

```text
SDF link pose と static TF を二重管理しない。
不一致がある場合は修正する。
frame_id は移植先システム構成に合わせる。
```

---

## 23. clock / sim time

Gazebo simulation time を使用する。

方針:

```text
/clock を bridge する
ROS 2 ノードは use_sim_time:=true を基本とする
RViz、TF、rosbag、各ノードの時刻基準を揃える
```

---

## 24. シナリオ管理

道路種別、道幅、パイロン有無、seed を組み合わせてシナリオとして扱う。

例:

```yaml
scenario_name: crank_w5_pylon_seed1234
road_type: crank
road_width: 5.0
enable_pylons: true
pylon_seed: 1234
```

将来的には、YAML でシナリオ定義を読み込む方式も検討する。

初期移植では launch 引数で十分とする。

---

## 25. ファイル名ルール

`road_width` の表記揺れを防ぐため、内部キーを定義する。

```text
2.0 -> w2
3.0 -> w3
5.0 -> w5
```

world ファイル名例:

```text
road_straight_w2.world
road_straight_w3.world
road_straight_w5.world
road_crank_w3.world
road_crank_w5.world
road_scurve_w3.world
road_scurve_w5.world
```

model 名例:

```text
road_straight_100m_w2
road_straight_100m_w3
road_straight_100m_w5
road_crank_50m_w3
road_crank_50m_w5
road_scurve_100m_w3
road_scurve_100m_w5
```

---

## 26. 受け入れ条件

移植後、最低限以下を満たすこと。

```text
1. colcon build が成功する
2. Gazebo Harmonic で world が起動する
3. straight / crank / scurve を選択できる
4. road_width を選択できる
5. ロボットが spawn される
6. /cmd_vel 相当でロボットが走行する
7. odom 相当が publish される
8. 2D LaserScan 相当が publish される
9. 3D PointCloud2 相当が publish される
10. fake localization pose 相当が publish される
11. map / odom / base_link / laser / mid360_frame の TF が解決できる
12. waypoint CSV が生成できる
13. pylon あり world が生成できる
14. pylon seed により同一配置を再現できる
15. pylon なし world も起動できる
16. topic 名は移植先構成に合わせて remap 可能である
```

---

## 27. 検証コマンド例

例:

```bash
cd ~/tsukuba-challenge/tc2026/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/kazuki/tsukuba-challenge/.venv/bin/activate
colcon build --packages-select obstacle_route_sim
source install/setup.bash
```

起動例:

```bash
ros2 launch obstacle_route_sim sim_obstacle_route.launch.py \
  road_type:=crank \
  road_width:=5.0 \
  enable_pylons:=true \
  pylon_seed:=1234 \
  use_sim_time:=true
```

確認例:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /mid360/livox/lidar --once
ros2 topic echo /ypspur_ros/odom --once
ros2 topic echo /localization/pose_enu --once
```

TF 確認例:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link mid360_frame
```

走行確認例:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.1}}"
```

ただし、上記 topic 名は互換例であり、最終的には移植先システム構成に合わせる。

---

## 28. 移植作業順序

推奨順序:

```text
1. 新パッケージ obstacle_route_sim を作成
2. package.xml / CMakeLists.txt を Jazzy + ros_gz 前提で整備
3. road_geometry.py を作成し、道路中心線定義を共通化
4. road_generator.py を移植
5. generate_waypoints.py を移植
6. Gazebo Harmonic 用 world template を作成
7. robot model を Harmonic 用に移植
8. sim_obstacle_route.launch.py を作成
9. ros_gz_bridge 設定を追加
10. /cmd_vel 相当による走行確認
11. odom bridge 確認
12. 2D LiDAR bridge 確認
13. 3D LiDAR bridge 確認
14. fake_pose_enu.py を接続
15. TF 整合性確認
16. generate_pylon_world.py を作成
17. pylon あり world の再現生成確認
18. 既存経路追従ノードとの接続確認
19. docs/design.md を整備
```

---

## 29. 最終的な設計方針まとめ

`obstacle_route_sim` は、Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic を前提とした、障害物付き経路追従検証用シミュレーションパッケージとする。

既存 `lidar_obstacle_sim` の全機能を移植対象とするが、Gazebo Classic 依存は廃止し、Gazebo Harmonic + `ros_gz` 構成へ置き換える。

既存実装に含まれる topic、frame、ファイル名、道路定義重複、SDF/TF 不整合などは、移植時に修正する。

パイロン生成は、Gazebo 起動後の spawn 方式ではなく、起動前に pylon を含む world を生成する方式とする。

トピック名、frame_id、namespace、remap は、移植先 tc2026 のシステム構成に合わせて決定する。既存 tc2025 の topic 名は互換用 remap として扱う。

この方針により、既存の経路追従・障害物回避ノードを Jazzy 世代の Gazebo Harmonic 環境で検証できるようにする。
