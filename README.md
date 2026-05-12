# tc2026

ROS 2 Jazzy ワークスペース。

## パッケージ一覧

| パッケージ                                                 | 役割                                                                 |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| [`src/rtk_gps_um982`](src/rtk_gps_um982/README.md)         | Unicore UM982 RTK GNSS ドライバ。NavSatFix / Imu / RtkStatus を配信  |
| [`src/rtk_gps_um982_msgs`](src/rtk_gps_um982_msgs/)         | 上記用カスタム msg (`RtkStatus`)                                     |
| [`src/ypspur_ros2`](src/ypspur_ros2/README.md)              | yp-spur ベースの差動駆動ロボット制御。`/cmd_vel` で動かす            |

ワークスペース横断の仕様書は [`docs/`](docs/)、パッケージ固有の設計書は各パッケージ
配下の `docs/design.md` を参照。

## 必要環境

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- (パッケージごとの追加要件は各 README を参照)

## Python 依存モジュール

Python パッケージ群で使用する pip 依存モジュールは、[`requirements.txt`](requirements.txt) にまとめている。
対象は `obstacle_monitor`, `robot_console`, `robot_navigator`, `route_follower`,
`route_manager`, `route_planner`, `yolo_detector` と、それらが利用する
`route_msgs`。

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
colcon build
source install/setup.bash
```

選択的にビルドする場合:

```bash
colcon build --packages-select rtk_gps_um982_msgs rtk_gps_um982
colcon build --packages-select ypspur_ros2
```

## 起動例

```bash
# RTK GPS ドライバ
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py

# yp-spur ロボット制御 (別端末で ypspur-coordinator も起動)
ypspur-coordinator -d /dev/ttyACM0 -p ~/spur/my_robot.param
ros2 launch ypspur_ros2 ypspur_ros2.launch.py
```

## 外部依存 (submodule)

ビルド前に `git submodule update --init --recursive` が必要。

- `src/rtk_gps_um982/third_party/UM982-RTK-GPS-Library` (MIT)
- `src/ypspur_ros2/third_party/yp-spur` (MIT) — Issue #245 のパッチを CMake が自動適用

## ライセンス

各パッケージは MIT。本リポジトリ全体としてのライセンスは [`LICENSE`](LICENSE) を参照。
