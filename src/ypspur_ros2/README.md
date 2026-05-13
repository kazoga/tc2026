# ypspur_ros2

[openspur/yp-spur](https://github.com/openspur/yp-spur) を ROS 2 (Jazzy) でラップし、
`/cmd_vel` で差動駆動ロボットを制御するパッケージ。

## 主要トピック

| Topic       | Type                          | 方向 | 説明                                  |
| ----------- | ----------------------------- | ---- | ------------------------------------- |
| `cmd_vel`   | `geometry_msgs/msg/Twist`     | sub  | `YPSpur_vel(linear.x, angular.z)` を呼ぶ |
| `odom`      | `nav_msgs/msg/Odometry`       | pub  | `YPSpur_get_pos` / `YPSpur_get_vel` を 50Hz で配信 |

## Issue #245 への対処

`yp-spur` 本体は Linux kernel 6.x 環境で
[issue #245](https://github.com/openspur/yp-spur/issues/245) (tcflush が入力バッファも flush する問題)
の影響を受けます。**Ubuntu 22.04 / 24.04 では必ずパッチが必要** で、
本パッケージはこれを CMake からビルド用コピーへ **自動適用** します
([`third_party/patches/0001-fix-tcflush-kernel-6.x.patch`](third_party/patches/0001-fix-tcflush-kernel-6.x.patch))。
`third_party/yp-spur` の submodule 本体は変更しません。

`colcon build` 時に
```
-- Applying yp-spur patch (0001-fix-tcflush-kernel-6.x)
```
もしくは
```
-- yp-spur patch already applied (or not needed)
```
が表示されます。

## 必要環境

- Ubuntu 24.04 (22.04 でも可)
- ROS 2 Jazzy
- yp-spur 用パラメータファイル (`robot.param`) — 自分のロボット用のもの

## ビルド

```bash
cd ~/colcon_ws
git submodule update --init --recursive
colcon build --packages-select ypspur_ros2
source install/setup.bash
```

`ypspur-coordinator`, `ypspur-free`, `ypspur-interpreter` も install/ypspur_ros2/bin/
に配置されるので、sourceしたあと PATH 経由で利用可能です。

## 起動

### 1. ypspur-coordinator を起動 (別端末)

```bash
ypspur-coordinator -d /dev/ttyACM0 -p ~/spur/my_robot.param
```

ロボットによっては `--without-device-watchdog` 等のオプションが必要。詳細は yp-spur
本体ドキュメントを参照。

### 2. ROS ノードを起動

```bash
ros2 launch ypspur_ros2 ypspur_ros2.launch.py
```

### 3. 動作確認

```bash
# 前進 0.1 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10

# odometry を監視
ros2 topic echo /odom
```

cmd_vel が `cmd_vel_timeout_s` (既定 0.5 秒) 入らないとロボットは自動停止します。

## パラメータ

| パラメータ              | 型      | 既定値      | 説明                                                |
| ----------------------- | ------- | ----------- | --------------------------------------------------- |
| `cmd_vel_timeout_s`     | double  | `0.5`       | 自動停止までのタイムアウト                          |
| `odom_publish_hz`       | double  | `50.0`      | `/odom` 配信レート                                  |
| `odom_frame_id`         | string  | `odom`      | Odometry header.frame_id                            |
| `base_frame_id`         | string  | `base_link` | Odometry child_frame_id                             |
| `coordinate_system`     | int     | `2` (CS_GL) | 0=BS, 1=SP, 2=GL, 3=LC, 4=FS, 5=BL                  |
| `ipc.use_socket`        | bool    | `false`     | true なら TCP 経由、false なら local msgqueue       |
| `ipc.ip`                | string  | `127.0.0.1` | socket モード時のホスト                             |
| `ipc.port`              | int     | `54321`     | socket モード時のポート                             |
| `velocity_max.linear`   | double  | `1.0`       | 受信 linear.x のクリップ閾値 (m/s, ±対称)           |
| `velocity_max.angular`  | double  | `1.5`       | 受信 angular.z のクリップ閾値 (rad/s, ±対称)        |

## トラブルシューティング

### `YPSpur_init failed. Is ypspur-coordinator running?`

ypspur-coordinator が起動していないか、別ユーザで起動している (msgqueue が見えない)。
同じユーザで先に coordinator を起動してから ROS ノードを起動する。

### Coordinator がモータドライバを認識しない (Ubuntu 22.04+)

- `dmesg | grep ttyACM` でデバイスが見えているか確認
- ModemManager が `ttyACM*` を掴んでしまうことがある。`sudo systemctl stop ModemManager`
- パッチが適用されていない場合 (本パッケージ経由でビルドしていない場合) は Issue #245 の症状
  (接続拒否・異音・負荷で coordinator が落ちる) が出る

### `/odom` が出ているのに位置が動かない

- coordinator のパラメータファイルでホイール直径やトレッド幅が合っていない
- `coordinate_system` がロボットの初期化方法と合っていない

## 設計

詳細は [`docs/design.md`](docs/design.md) を参照。

## ライセンス

MIT (yp-spur 本体も MIT)
