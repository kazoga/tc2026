# obstacle_route_sim

つくば2026の全周デジタルツインは [maps/tsukuba2026](maps/tsukuba2026/README.md) に
固定地図・オフライン閲覧版・静止画像を同梱している。clone後の確認にはこの固定版を使う。

`obstacle_route_sim` は、Gazebo Harmonic 上で直線・S字・クランクの道路 world、差動二輪ロボット、2D LiDAR、Mid-360 相当 3D LiDAR、pylon 障害物を起動し、既存の route stack と接続して障害物回避・ルート復帰を検証するためのパッケージである。

詳細設計は `docs/obstacle_route_sim_詳細設計書.md` を参照する。

MID-360の取付角を変えて地面観測範囲を比較する方法は
[LiDAR取付角と地面観測](docs/LiDAR取付角と地面観測.md)を参照する。

## 対象構成

対応 world は以下とする。

| route id | road_type | road_width | waypoints | start_label | goal_label |
| --- | --- | --- | --- | --- | --- |
| `straight_w2` | `straight` | `2.0` | 21 | `0` | `20` |
| `straight_w3` | `straight` | `3.0` | 21 | `0` | `20` |
| `straight_w5` | `straight` | `5.0` | 21 | `0` | `20` |
| `scurve_w3` | `scurve` | `3.0` | 28 | `0` | `27` |
| `scurve_w5` | `scurve` | `5.0` | 28 | `0` | `27` |
| `crank_w3` | `crank` | `3.0` | 33 | `0` | `32` |
| `crank_w5` | `crank` | `5.0` | 33 | `0` | `32` |

生成済み route 資産は以下に配置する。

- `src/route_planner/routes/obstacle_route_sim/<route_id>/fixed/waypoints.csv`
- `src/route_planner/routes/obstacle_route_sim/<route_id>/route_config.yaml`
- `src/route_planner/params/obstacle_route_<route_id>.yaml`
- `src/route_manager/params/obstacle_route_<route_id>.yaml`

`robot_console` は `route_planner` と `route_manager` の `params` を起動カードの候補として自動検出するため、両方で同じ `obstacle_route_<route_id>.yaml` を選択する。

## ビルド

ワークスペースルートで ROS 2 Jazzy 環境を有効化してから実行する。

```bash
colcon build --packages-select obstacle_route_sim route_planner route_manager robot_console
source install/setup.bash
```

開発中に `--symlink-install` と通常 install を切り替える場合は、対象パッケージの `build/<package>/` と `install/<package>/` をクリーンしてから再ビルドする。

## waypoint と route/config の作成

単体の waypoint CSV だけを作る場合は `generate_waypoints.py` を使う。

```bash
python3 src/obstacle_route_sim/scripts/generate_waypoints.py \
  --road straight \
  --width 5.0 \
  --output <output_csv>
```

`robot_console` から選べる route/config 一式を作る場合は `generate_route_assets.py` を使う。既定では全 world 分を生成する。

```bash
python3 src/obstacle_route_sim/tools/generate_route_assets.py
```

一部だけ再生成する場合は `--spec road:width` を指定する。

```bash
python3 src/obstacle_route_sim/tools/generate_route_assets.py \
  --spec straight:5.0 \
  --spec scurve:5.0 \
  --spec crank:5.0
```

生成後は `route_planner` と `route_manager` を再ビルドし、install 配下へ反映する。

```bash
colcon build --packages-select route_planner route_manager
source install/setup.bash
```

## Gazebo 単体起動

Gazebo GUI 付きで world、robot、bridge、fake localization pose、TF を起動する。

```bash
ros2 launch obstacle_route_sim sim_obstacle_route.launch.py \
  road_type:=straight \
  road_width:=5.0 \
  enable_pylons:=false \
  start_gazebo_gui:=true
```

pylon を含める場合は `enable_pylons:=true` を指定する。配置は `pylon_seed` で再現できる。

```bash
ros2 launch obstacle_route_sim sim_obstacle_route.launch.py \
  road_type:=scurve \
  road_width:=5.0 \
  enable_pylons:=true \
  pylon_seed:=0 \
  start_gazebo_gui:=true
```

経路上に決定的な blocker pylon を置いて障害物回避を再現したい場合は、統合 launch の `enable_route_blocker:=true` を使う。

```bash
ros2 launch obstacle_route_sim gazebo_obstacle_route_stack.launch.py \
  road_type:=straight \
  road_width:=5.0 \
  enable_pylons:=false \
  enable_route_blocker:=true \
  route_blocker_distance:=8.0 \
  start_gazebo_gui:=true \
  start_drive_status_gui:=true
```

## robot_console からの結合動作確認

Gazebo は `sim_obstacle_route.launch.py` で起動し、route stack は `robot_console` から起動する。以下は `straight_w5` の例である。

1. Gazebo GUI を起動する。

```bash
ros2 launch obstacle_route_sim sim_obstacle_route.launch.py \
  road_type:=straight \
  road_width:=5.0 \
  enable_pylons:=false \
  start_gazebo_gui:=true
```

2. 別端末で `robot_console` を起動する。

```bash
ros2 launch robot_console robot_console.launch.py
```

3. `robot_console` の起動カードで以下を選択する。

| profile | 選択・入力 |
| --- | --- |
| Route Planner | `obstacle_route_straight_w5.yaml` |
| Route Manager | `obstacle_route_straight_w5.yaml`、Start Label `0`、Goal Label `20` |
| Route Follower | `default.yaml` |
| Obstacle Monitor | `default.yaml` |
| Drive Mode Manager | `default.yaml`、`start_gui=true`、`joy_input=joy_node` |
| Robot Navigator | `default.yaml`、`cmd_vel_topic=/cmd_vel/autonomous`、`odom_topic=/ypspur_ros/odom` |

4. `route_planner`、`route_manager`、`route_follower`、`obstacle_monitor`、`drive_mode_manager`、`robot_navigator` の順に起動する。
5. `manual_start` を ON にする。
6. `/route_state.current_label` が goal label に到達することを確認する。

S字・クランクでは、同じ手順で route id と goal label を置き換える。

| world | Route Planner / Route Manager params | Goal Label |
| --- | --- | --- |
| 直線 | `obstacle_route_straight_w5.yaml` | `20` |
| S字 | `obstacle_route_scurve_w5.yaml` | `27` |
| クランク | `obstacle_route_crank_w5.yaml` | `32` |

pylon ありで確認する場合は、手順 1 の Gazebo 起動時に `enable_pylons:=true` を指定し、route stack 起動時に `obstacle_monitor` も起動する。pylon を必ず経路上に置いて障害物回避・復帰を見る場合は、`gazebo_obstacle_route_stack.launch.py` の `enable_route_blocker:=true` を使う。

## robot_console GUI 自動操作による確認

実 GUI を automation hook で操作する評価ツールも利用できる。ローカルデスクトップまたは X11 転送が有効な環境で実行する。

Gazebo を起動した状態で、別端末から以下を実行する。

```bash
python3 src/robot_console/tools/qt_route_stack_eval.py \
  --route-planner-param obstacle_route_straight_w5.yaml \
  --route-manager-param obstacle_route_straight_w5.yaml \
  --start-label 0 \
  --goal-label 20 \
  --timeout-sec 170 \
  --post-goal-wait-sec 3 \
  --startup-wait-sec 3 \
  --stop-timeout-sec 25 \
  --no-simulator \
  --show-drive-status-gui \
  --launch-order route_planner,route_manager,route_follower,obstacle_monitor,drive_mode_manager,robot_navigator
```

S字とクランクは params と goal label を置き換える。

```bash
python3 src/robot_console/tools/qt_route_stack_eval.py \
  --route-planner-param obstacle_route_scurve_w5.yaml \
  --route-manager-param obstacle_route_scurve_w5.yaml \
  --start-label 0 \
  --goal-label 27 \
  --timeout-sec 240 \
  --no-simulator \
  --show-drive-status-gui

python3 src/robot_console/tools/qt_route_stack_eval.py \
  --route-planner-param obstacle_route_crank_w5.yaml \
  --route-manager-param obstacle_route_crank_w5.yaml \
  --start-label 0 \
  --goal-label 32 \
  --timeout-sec 300 \
  --no-simulator \
  --show-drive-status-gui
```

## 地形・センサ・融合の確認

同梱のつくば地図で操作UIを使う場合は[icart_bringup](../icart_bringup/README.md)から起動する。
基本の道路worldによる真値pose試験と、センサから推定する融合試験を区別する。

| 対象 | 説明 |
| --- | --- |
| [terrain3d / i-Cart mini](docs/terrain3d_icart_mini検証.md) | 地形生成・模擬車体・期限付き試験・真値軌跡 |
| [地理地図・GNSS](docs/地理地図_GNSSシミュレーション検証.md) | LLH、品質、欠測の入力 |
| [つくば全域モデル](docs/つくば2026全域デジタルツイン.md) | 入力データ・精度・人為的な通路確保 |
| [地理経路とFLOAT](docs/地理ウェイポイント_FLOAT_FASTLIO評価.md) | 経路・建物近傍誤差の比較 |
| [FAST-LIO接続](docs/FASTLIOシミュレータ接続検証.md) | IMU・点群adapter、比較と融合走行の選択 |
| [点群地図](docs/全域FASTLIO点群地図評価.md) | 分割保存、PCD、SDF表面との比較 |
| [センサ誤差モデル](docs/FASTLIOセンサ誤差モデル評価.md) | reference／field_assumed／conservativeの仮定 |
| [融合](../gnss_lio_fusion/docs/実装評価.md) | 品質・方位・車輪退避・閉ループ確認 |

`evaluate_terrain_trial.py --gnss --fusion --fastlio <実行ファイル>`で融合位置による走行を選ぶ。
`--building-gnss`、`--baseline-shift`、`--baseline-shift-after`で品質・基線長変化を試験できる。
センサ誤差は--lio-noise-profileと--lio-noise-seedで指定する。
fastlio_sim.launchの既定はfield_assumed、共通デジタルツインはconservativeである。

## 運用上の注意

Gazebo・UI・走行系のdomainと時計を揃え、同じprofileを二重に起動しない。
MotionLimitsを配信しないGazebo構成ではnavigatorのrequire_motion_limitsをfalseにする。
robot_navigator同梱のrobot_simulatorはMotionLimitsを配信するため、監視を有効にできる。
GPUデバイスの権限とGazebo依存を準備する。生成地図やログはGitに追加しない。
FINISHED、真値停止、接触、横ずれ、推定誤差を分け、模擬結果を実機の性能保証として扱わない。
