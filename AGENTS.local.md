# AGENTS.local.md

このファイルは、ローカル環境で Codex が ROS 2 コマンドを実行する場合だけ参照する
追加指示である。共通ルールは `AGENTS.md` を正とし、本ファイルは `ros2 run`,
`ros2 launch`, `ros2 topic`, `ros2 service` などの実行確認手順を補足する。

---

# Local ROS 2 Run Policy

- ローカル環境で、ユーザーが ROS 2 実行確認を明示した場合のみ `ros2` コマンドを実行してよい。
- 実行前に、対象パッケージ、起動するノード、投入する仮想入力、監視する出力、
  終了条件を簡潔に説明する。
- 実行前に `source install/setup.bash` を行う。未ビルドまたは install が古い可能性がある場合は、
  先に必要パッケージを `colcon build --symlink-install --packages-select <package_name>` で確認する。
- 長時間起動するノード、GUI、シミュレーションは `timeout`、監視スクリプト、または明示的な停止処理を用意する。
- Codex が ROS 2 実行確認を行う場合は、原則としてワークスペース直下の `log/codex/`
  配下に実行ごとのログディレクトリを作成する。例：`log/codex/<YYYYMMDD_HHMMSS>/`。
- 実行確認時は、少なくとも `ROS_LOG_DIR` を `log/codex/<run_id>/ros` に設定する。
  `robot_console` 管理の子プロセス stdout/stderr を保存する場合は、
  `console_log_directory` 相当を `log/codex/<run_id>/robot_console` に設定する。
- 実行結果の報告では、実行したコマンド、投入した入力、確認した出力、成功/失敗、
  ログ出力先、未確認事項、停止処理の結果を明記する。
- 確認後は `ros2 node list`, `ros2 topic list`, `pgrep -af` などで、意図しないプロセスや
  ノードが残っていないことを確認する。

---

# Safety Rules

- `ypspur_ros2` を起動してはならない。
- `ypspur-coordinator` を起動してはならない。
- 実機のロボットを実際に動かす操作を行ってはならない。
- `/cmd_vel` を実機 driver に接続する構成を起動してはならない。
- 実施してよいのは、各ノードの単体動作確認と、シミュレーションによる結合動作確認に限る。
- `rtk_gps_um982`、実センサ、実カメラ、LiDAR、Gazebo、RViz、外部デバイスを使う確認は、
  ユーザーが明示した場合のみ行い、必要な前提と未確認範囲を報告する。

---

# Package-Level Run Check Examples

パッケージ単体の実行確認では、対象ノードに必要な入力 topic / service を仮想的に与え、
出力 topic / service 応答を確認する。以下は実施方法の例であり、実際のパラメータや
topic 名は対象パッケージの README、launch、詳細設計書を確認してから調整する。

## 共通の確認パターン

1. 対象パッケージをビルドする。

   ```bash
   colcon build --symlink-install --packages-select <package_name>
   source install/setup.bash
   ```

2. ノードを短時間起動する。

   ```bash
   timeout 20s ros2 launch <package_name> <name>.launch.py
   ```

3. 別端末または別プロセスで入力を publish / service call する。

   ```bash
   ros2 topic pub --once /input_topic <msg_type> "{...}"
   ros2 service call /service_name <srv_type> "{...}"
   ```

4. 出力を確認する。

   ```bash
   timeout 10s ros2 topic echo --once /output_topic <msg_type>
   ```

5. 終了後に残存プロセスを確認する。

   ```bash
   ros2 node list
   pgrep -af "ros2 launch|ros2 run|<node_name>" || true
   ```

## `route_planner`

`route_planner` は route CSV/YAML から経路を生成し、`/get_route` と `/update_route`
service を提供する。単体確認では、launch 後に `GetRoute` service を呼び、
返却 route の waypoint 数、start/goal label、成功可否を確認する。

```bash
source install/setup.bash
timeout 30s ros2 launch route_planner route_planner.launch.py \
  param_file:="$(ros2 pkg prefix route_planner)/share/route_planner/params/tsukuba.yaml"
```

別プロセスで以下を実行する。

```bash
source install/setup.bash
ros2 service call /get_route route_msgs/srv/GetRoute \
  "{start_label: '10', goal_label: '50', checkpoint_labels: []}"
```

## `route_manager`

`route_manager` は `route_planner` の service を入力として `/active_route`,
`/route_state`, `/manager_status`, `/mission_info` を publish する。
単体寄りに確認する場合でも、実用上は `route_planner` と組み合わせて確認する。

```bash
source install/setup.bash
ros2 launch route_planner route_planner.launch.py \
  param_file:="$(ros2 pkg prefix route_planner)/share/route_planner/params/tsukuba.yaml"
```

別プロセスで以下を起動する。

```bash
source install/setup.bash
ros2 launch route_manager route_manager.launch.py \
  param_file:="$(ros2 pkg prefix route_manager)/share/route_manager/params/tsukuba.yaml" \
  start_label:=10 \
  goal_label:=50
```

出力確認例:

```bash
source install/setup.bash
timeout 10s ros2 topic echo --once /route_state route_msgs/msg/RouteState
timeout 10s ros2 topic echo --once /active_route route_msgs/msg/Route
```

## `route_follower`

`route_follower` は `/active_route`, `/amcl_pose`, `/manual_start`, `/sig_recog`,
`/road_blocked`, `/obstacle_avoidance_hint` を入力として、`/active_target`,
`/follower_state`, `/recog_flag` を出力する。単体確認では、短い `Route` と
`PoseWithCovarianceStamped` を仮想入力として与え、`/active_target` と
`/follower_state` が出ることを確認する。

実 route を使う場合は、`route_planner` と `route_manager` を併用し、
`robot_simulator` またはテスト用 publisher で `/amcl_pose` を与える。

出力確認例:

```bash
source install/setup.bash
timeout 10s ros2 topic echo --once /active_target geometry_msgs/msg/PoseStamped
timeout 10s ros2 topic echo --once /follower_state route_msgs/msg/FollowerState
```

## `robot_navigator`

`robot_navigator` は `/active_target`, `/amcl_pose`, `/odom` などを入力として
`/cmd_vel` を publish する。単体確認では、仮想 pose と target を与え、
`/cmd_vel` が publish されることを確認する。実機 driver には接続しない。

```bash
source install/setup.bash
ros2 launch robot_navigator robot_navigator.launch.py
```

入力例:

```bash
source install/setup.bash
ros2 topic pub --once /active_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
ros2 topic pub --once /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
ros2 topic pub --once /odom nav_msgs/msg/Odometry \
  "{header: {frame_id: 'odom'}, child_frame_id: 'base_link', pose: {pose: {orientation: {w: 1.0}}}}"
```

出力確認例:

```bash
source install/setup.bash
timeout 10s ros2 topic echo --once /cmd_vel geometry_msgs/msg/Twist
```

## `robot_console`

`robot_console` は tkinter GUI を持つため、ローカル環境で `DISPLAY` が利用できる場合に
GUI 起動確認を行う。画面座標クリックは使わず、`UiMain` の automation hook を通して
`Combobox`, `Entry`, `Checkbutton`, `Button` 相当の操作を行う。

短時間起動確認:

```bash
source install/setup.bash
timeout 10s ros2 run robot_console robot_console
```

GUI 操作を伴う route stack 評価では、手書きの inline Python ではなく
`src/robot_console/tools/gui_route_stack_eval.py` を使う。純粋な CLI / SSH 環境など
`DISPLAY` が利用できない場合は、GUI あり評価は実施せず、
`src/robot_console/tools/headless_route_stack_eval.py` による headless 評価へ切り替える。

```bash
source install/setup.bash
python3 src/robot_console/tools/gui_route_stack_eval.py \
  --start-label 10 \
  --goal-label 30
```

確認後は `ros2 node list`, `ros2 topic list`, `pgrep -af` で残存ノード・
プロセスがないことを確認する。

---

# Integration Run Check Examples

結合動作確認は、ローカル環境で安全に実施できるシミュレーション手順として整理する。
現時点では、「既存 route stack 回帰用の簡易構成」のみを対象とする。今後、
GNSS/LiDAR 入力模擬込み構成や Gazebo 障害物回避・ルート復帰検証構成などを
実装・確認する場合は、本章へ同じ粒度で手順を追加する。

## 既存 route stack 回帰用の簡易構成

目的は、既存 route stack が `route_planner -> route_manager -> route_follower ->
robot_navigator -> robot_simulator` の接続で進行し、指定した goal label へ到達できることを
確認することである。この構成では `rtk_gps_um982` と `ypspur_ros2` は起動しない。

### 対象ノード

- `robot_console`
- `route_planner`
- `route_manager`
- `route_follower`
- `robot_navigator`
- `robot_navigator` 付属の `robot_simulator`

### 実施ツール

この構成の確認では、手書きの inline Python ではなく以下の正式ツールを使う。

- GUI なし、または `DISPLAY` が利用できない環境：
  `src/robot_console/tools/headless_route_stack_eval.py`
- GUI あり、ローカルデスクトップまたは X11 転送ありの環境：
  `src/robot_console/tools/gui_route_stack_eval.py`

GUI あり評価では `UiMain` を実際に生成し、画面座標クリックではなく automation hook
経由で `Combobox`, `Entry`, `Checkbutton`, `Button` 相当の操作を行う。

### 既定の評価条件

| 項目 | 値 |
| --- | --- |
| `route_planner` parameter | `tsukuba.yaml` |
| `route_manager` parameter | `tsukuba.yaml` |
| `route_manager` Start Label | `10` |
| `route_manager` Goal Label | `30` |
| `route_follower` parameter | `default.yaml` |
| `robot_navigator` parameter | `default.yaml` |
| `robot_navigator` Simulator | 有効 |
| 起動後入力 | `manual_start=True` |

`route_follower` の `default.yaml` は `start_immediately: false` のため、起動後に
`manual_start=True` を送る。

### 監視条件

- `/route_state` (`route_msgs/msg/RouteState`) を監視する。
- `/active_route` (`route_msgs/msg/Route`) を監視し、waypoint 数と start / goal label を確認する。
- `/follower_state` (`route_msgs/msg/FollowerState`) を監視する。
- `/cmd_vel` (`geometry_msgs/msg/Twist`) を監視し、走行中に出力されることを確認する。
- `/manual_start` (`std_msgs/msg/Bool`) を監視し、`True` が送信されたことを確認する。
- `/route_state.current_label == <goal_label>` になったら goal 到達とみなす。
- 到達後は 10 秒待機してから停止処理を行う。
- 待機には上限時間を設ける。既定では 180 秒を上限とする。
- 停止後に `ros2 node list`, `ros2 topic list`, `pgrep -af` で残存ノード・
  プロセスを確認する。

### ログ出力先

この統合評価を Codex が実行する場合は、実行前に `run_id` を決め、
`log/codex/<run_id>/` 配下へログを集約する。

```bash
run_id=$(date +%Y%m%d_%H%M%S)
mkdir -p "log/codex/${run_id}/ros" "log/codex/${run_id}/robot_console"
export ROS_LOG_DIR="$PWD/log/codex/${run_id}/ros"
```

評価ツールが `robot_console` を Python から直接生成する場合は、
`--console-log-directory log/codex/<run_id>/robot_console` を指定し、
`robot_console` 管理の子プロセス stdout/stderr も `log/codex/<run_id>/robot_console`
配下へ保存する。保存済み ROS ログは `ROS_LOG_DIR` を参照する。

### GUI なし実施例

純粋な CLI / SSH 環境、または `DISPLAY` が利用できない環境では headless 評価を使う。

```bash
source install/setup.bash
run_id=$(date +%Y%m%d_%H%M%S)
mkdir -p "log/codex/${run_id}/ros" "log/codex/${run_id}/robot_console"
export ROS_LOG_DIR="$PWD/log/codex/${run_id}/ros"
python3 src/robot_console/tools/headless_route_stack_eval.py \
  --start-label 10 \
  --goal-label 30 \
  --timeout-sec 180 \
  --post-goal-wait-sec 10 \
  --console-log-directory "log/codex/${run_id}/robot_console"
```

### GUI あり実施例

ローカルデスクトップまたは X11 転送ありの環境では GUI あり評価を使う。

```bash
source install/setup.bash
run_id=$(date +%Y%m%d_%H%M%S)
mkdir -p "log/codex/${run_id}/ros" "log/codex/${run_id}/robot_console"
export ROS_LOG_DIR="$PWD/log/codex/${run_id}/ros"
python3 src/robot_console/tools/gui_route_stack_eval.py \
  --start-label 10 \
  --goal-label 30 \
  --timeout-sec 180 \
  --post-goal-wait-sec 10 \
  --console-log-directory "log/codex/${run_id}/robot_console" \
  --verify-log-open-buttons
```

### 残存確認例

```bash
source install/setup.bash
ros2 node list
ros2 topic list
pgrep -af "robot_console|route_planner|route_manager|route_follower|robot_navigator|robot_simulator|ros2 launch" || true
```

### 成功条件

- `route_manager` が `/route_state` を publish する。
- `/active_route` の start label が `"10"`、goal label が `"30"` である。
- `/manual_start` に `True` が publish される。
- `/cmd_vel` が走行中に publish される。
- `/route_state.current_label` が `"10"` から進行し、最終的に `"30"` に到達する。
- `"30"` 到達後に 10 秒待機できる。
- 停止処理後に対象ノード・対象プロセスが残っていない。

### 報告例

```text
/active_route waypoints=21 start='10' goal='30' version=100
/route_state current_label='30' status=2 version=100
goal label '30' reached by /route_state
stop state: route_planner status=STOPPED pid=None sim_pid=None error=''
stop state: route_manager status=STOPPED pid=None sim_pid=None error=''
stop state: route_follower status=STOPPED pid=None sim_pid=None error=''
stop state: robot_navigator status=STOPPED pid=None sim_pid=None error=''
```
