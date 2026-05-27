# obstacle_route_sim

`obstacle_route_sim` は、Gazebo Harmonic 上で直線・S字・クランクの道路 world、差動二輪ロボット、2D LiDAR、Mid-360 相当 3D LiDAR、pylon 障害物を起動し、既存の route stack と接続して障害物回避・ルート復帰を検証するためのパッケージである。

詳細設計は `docs/obstacle_route_sim_詳細設計書.md` を参照する。

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

Gazebo GUI 付きで world、robot、bridge、fake AMCL、TF を起動する。

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
python3 src/robot_console/tools/gui_route_stack_eval.py \
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
python3 src/robot_console/tools/gui_route_stack_eval.py \
  --route-planner-param obstacle_route_scurve_w5.yaml \
  --route-manager-param obstacle_route_scurve_w5.yaml \
  --start-label 0 \
  --goal-label 27 \
  --timeout-sec 240 \
  --no-simulator \
  --show-drive-status-gui

python3 src/robot_console/tools/gui_route_stack_eval.py \
  --route-planner-param obstacle_route_crank_w5.yaml \
  --route-manager-param obstacle_route_crank_w5.yaml \
  --start-label 0 \
  --goal-label 32 \
  --timeout-sec 300 \
  --no-simulator \
  --show-drive-status-gui
```

## 確認済み結果

2026-05-26 時点で、Gazebo GUI と robot_console 実 GUI 自動操作を使い、以下を確認済みである。

| world | 条件 | 結果 |
| --- | --- | --- |
| `straight_w5` | `enable_pylons:=false` | `/route_state.current_label='20'` 到達 |
| `scurve_w5` | `enable_pylons:=false` | `/route_state.current_label='27'` 到達 |
| `crank_w5` | `enable_pylons:=false` | `/route_state.current_label='32'` 到達 |
| `straight_w5` | `enable_route_blocker:=true` | `front_blocked=true`、`AVOIDING` 遷移、`RUNNING` 復帰を確認 |

`w2` / `w3` の route/config は生成済みだが、GUI 走行確認は代表ケースとして `w5` を実施している。

## 注意事項

- `robot_console` から起動する場合、Gazebo 側は先に `sim_obstacle_route.launch.py` で起動しておく。
- `robot_navigator` は `/cmd_vel/autonomous` に出力し、`drive_cmd_mux_node` が最終 `/cmd_vel` を publish する構成にする。
- Gazebo GUI 付き確認では GPU デバイス権限が必要になる場合がある。`render` / `video` group 追加後はログアウト・ログインしてから確認する。
- `LIBGL_ALWAYS_SOFTWARE=1` は Gazebo/Ogre2 の安定性を落とす場合があるため、既定では使用しない。
- 停止時に SIGINT 由来の `Traceback` が表示されることがある。profile が `STOPPED` に遷移し、新しい crash report が出ていなければ停止処理として扱う。
