# tc2026

ROS 2 Jazzy ワークスペース。

## パッケージ一覧

### センサ / アクチュエータドライバ
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/rtk_gps_um982`](src/rtk_gps_um982/README.md)          | Unicore UM982 RTK GNSS ドライバ。NavSatFix / Imu / RtkStatus を配信  |
| [`src/rtk_gps_um982_msgs`](src/rtk_gps_um982_msgs/)         | 上記用カスタム msg (`RtkStatus`)                                     |
| [`src/ypspur_ros2`](src/ypspur_ros2/README.md)              | yp-spur ベースの差動駆動ロボット制御。`/cmd_vel` で動かす            |

### 経路計画・追従
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/route_planner`](src/route_planner/README.md)          | YAML / CSV から経路を生成し `/get_route`・`/update_route` を提供。可変ブロックの再計画にも対応 |
| [`src/route_manager`](src/route_manager/README.md)          | `route_planner` のサービスを呼び出し `/active_route` を配信、滞留報告から再計画を統括する FSM |
| [`src/route_follower`](src/route_follower/README.md)        | `/active_route` を追従し、現在の目標 Pose を `/active_target` として配信。滞留検知で `/report_stuck` を発行 |
| [`src/route_msgs`](src/route_msgs/README.md)                | 経路・走行系で共有する msg / srv 定義 (`Route`, `RouteState`, `ReportStuck` ほか) |

### 走行制御・障害物
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/robot_navigator`](src/robot_navigator/README.md)      | `/active_target` を追従して `/cmd_vel` を出力する時間最適制御ノード。試験用 `robot_simulator` も同梱 |
| [`src/drive_mode_manager`](src/drive_mode_manager/README.md) | 自律走行指令と手動走行指令を切り替え、最終 `/cmd_vel` と走行モード状態を配信 |
| [`src/obstacle_monitor`](src/obstacle_monitor/README.md)    | `/scan` を解析して `/obstacle_avoidance_hint` を配信。`/sensor_viewer` への可視化も提供 |

### 認識・監視
| パッケージ                                                  | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/yolo_detector`](src/yolo_detector/README.md)          | USB カメラ画像を YOLO (PyTorch / NCNN) で物体検出し、検出画像・`Detection2DArray` を配信 |
| [`src/robot_console`](src/robot_console/README.md)          | 走行状態・障害物回避・経路進捗・ノード起動を一画面で監視する tkinter GUI ダッシュボード |

ワークスペース横断の仕様書は [`docs/`](docs/)、パッケージ固有の設計書は各パッケージ
配下の `docs/design.md` を参照。

## 必要環境

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- (パッケージごとの追加要件は各 README を参照)

## Codex ローカル実行設定

Codex app / CLI / IDE Extension で `ros2 run`, `ros2 launch`, `ros2 topic` などを含む
ローカル環境の動作確認を行う場合は、`~/.codex/config.toml` に以下を追記する。

```toml
approval_policy = "on-request"
sandbox_mode = "workspace-write"

[sandbox_workspace_write]
network_access = true
```

この設定は、Codex がワークスペース内で ROS 2 ノードや確認用コマンドを実行するためのもの。
実機 driver や実ロボットを動かす確認は、各手順で明示された場合を除き実行しない。

## Python 依存モジュール

Python パッケージ群で使用する pip 依存モジュールは、[`requirements.txt`](requirements.txt) にまとめている。
対象は `obstacle_monitor`, `robot_console`, `robot_navigator`, `route_follower`,
`route_manager`, `route_planner`, `yolo_detector` と、それらが利用する
`route_msgs`。`drive_mode_manager` の GUI 依存である `python3-pyqt5` は
pip ではなく apt / rosdep で導入する。

ROS 2 の環境を読み込んだうえで、ワークスペース直下で以下を実行する。

```bash
python3 -m pip install -r requirements.txt
```

## ビルド

```bash
git clone --recursive https://github.com/t-nakabayashi/tc2026.git ~/colcon_ws
# 既存 clone の場合
# git submodule update --init --recursive

cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
python3 -m pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
```

開発時は `--symlink-install` 付きのビルドを推奨する。Python ソース、launch、config、
route、map、waypoint などの install 対象ファイルが `install/` 配下へ symlink されるため、
既存ファイルの内容変更を再ビルドなしで反映しやすい。
新規ファイル追加、ファイル名変更、install 対象の変更を行った場合は再ビルドする。

選択的にビルドする場合:

```bash
colcon build --symlink-install --packages-select rtk_gps_um982_msgs rtk_gps_um982
colcon build --symlink-install --packages-select ypspur_ros2
```

## 起動例

各パッケージごとに代表的な launch を抜粋。詳細な引数・トピック名は各パッケージ README を参照。

### センサ / アクチュエータドライバ
```bash
# RTK GPS ドライバ
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py

# yp-spur ロボット制御 (別端末で ypspur-coordinator も起動)
ypspur-coordinator -d /dev/ttyACM0 -p ~/spur/my_robot.param
ros2 launch ypspur_ros2 ypspur_ros2.launch.py cmd_vel_topic:=/cmd_vel

# coordinator も launch から起動する場合
ros2 launch ypspur_ros2 ypspur_ros2.launch.py \
  start_coordinator:=true \
  coordinator_device:=/dev/ttyACM0 \
  coordinator_param:=<robot_param_file> \
  cmd_vel_topic:=/cmd_vel
```

### 経路計画・追従
```bash
# 経路生成サービス (route_planner)
ros2 launch route_planner route_planner.launch.py

# 経路管理 FSM (route_manager)
ros2 launch route_manager route_manager.launch.py \
  start_label:=START goal_label:=GOAL \
  checkpoint_labels:="P1,P2"

# 経路追従 (route_follower)
ros2 launch route_follower route_follower.launch.py \
  arrival_threshold:=0.6 \
  control_rate_hz:=20.0 \
  start_immediately:=true
```

### 走行制御・障害物
```bash
# /active_target を追従して /cmd_vel を出力
ros2 launch robot_navigator robot_navigator.launch.py \
  obstacle_hint_topic:=/obstacle_avoidance_hint cmd_vel_topic:=/cmd_vel

# 自律/手動の走行指令 mux と専用状態 GUI
ros2 launch drive_mode_manager drive_mode_manager.launch.py

# 障害物監視 (LiDAR 入力 → /obstacle_avoidance_hint)
ros2 launch obstacle_monitor obstacle_monitor.launch.py \
  scan_topic:=/scan hint_topic:=/obstacle_avoidance_hint
```

### 手動走行のみ

手動走行だけを行う場合は、`drive_mode_manager` が Joy 入力から最終 `/cmd_vel` を publish し、
`ypspur_ros2` が `/cmd_vel` を車体へ渡す構成にする。

coordinator を別端末で手動起動する場合:

```bash
# 端末 1: yp-spur coordinator
ypspur-coordinator -d /dev/ttyACM0 -p ~/spur/my_robot.param

# 端末 2: /cmd_vel を購読して車体へ速度指令を渡す
ros2 launch ypspur_ros2 ypspur_ros2.launch.py cmd_vel_topic:=/cmd_vel

# 端末 3: joy_node を起動し、Joy 入力から最終 /cmd_vel を publish
ros2 launch drive_mode_manager drive_mode_manager.launch.py
```

coordinator も `ypspur_ros2.launch.py` から起動する場合:

```bash
# 端末 1: coordinator と ypspur_node を起動
ros2 launch ypspur_ros2 ypspur_ros2.launch.py \
  start_coordinator:=true \
  coordinator_device:=/dev/ttyACM0 \
  coordinator_param:=<robot_param_file> \
  cmd_vel_topic:=/cmd_vel

# 端末 2: joy_node を起動し、Joy 入力から最終 /cmd_vel を publish
ros2 launch drive_mode_manager drive_mode_manager.launch.py
```

起動直後は `drive_mode_manager` の既定モードが `autonomous` のため、Joy 入力で L1 と PS button
を長押しして手動走行へ切り替える。GUI が不要な端末では
`ros2 launch drive_mode_manager drive_mode_manager.launch.py start_gui:=false` を使う。
開発用 Joy simulator を使う場合は `joy_input:=ps3_joy_sim` を追加する。

### 認識・GUI
```bash
# YOLO 物体検出 (NCNN 版を推奨)
ros2 launch yolo_detector yolo_ncnn_node.launch.py

# 運用 GUI ダッシュボード
ros2 launch robot_console robot_console.launch.py
```

## 外部依存 (submodule)

ビルド前に `git submodule update --init --recursive` が必要。

- `src/rtk_gps_um982/third_party/UM982-RTK-GPS-Library` (MIT)
- `src/ypspur_ros2/third_party/yp-spur` (MIT) — Issue #245 のパッチを CMake が自動適用

## ライセンス

各パッケージは MIT。本リポジトリ全体としてのライセンスは [`LICENSE`](LICENSE) を参照。
