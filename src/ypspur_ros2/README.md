# ypspur_ros2

[openspur/yp-spur](https://github.com/openspur/yp-spur) を ROS 2 (Jazzy) でラップし、
`/cmd_vel` で差動駆動ロボットを制御するパッケージ。

## 主要トピック

| Topic | Type | 方向 | 説明 |
| --- | --- | --- | --- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | sub | `YPSpur_vel(linear.x, angular.z)` を呼ぶ。既定では `drive_mode_manager` が publish する `/cmd_vel` と接続する |
| `odom` | `nav_msgs/msg/Odometry` | pub | `YPSpur_get_pos` / `YPSpur_get_vel` を 50Hz で配信 |

`ypspur_node` は相対 topic `cmd_vel` を購読し、相対 topic `odom` を publish します。launch の既定では `odom` を `/ypspur_ros/odom` へ remap します（`robot_navigator`、`obstacle_route_sim` の Gazebo bridge、`robot_simulator` がいずれもこの topic 名を前提とするため）。namespace なしで `drive_mode_manager.launch.py` と同時起動した場合、`drive_cmd_mux_node` の最終出力 `/cmd_vel` がそのまま `ypspur_node` に入ります。topic 名を変える場合は launch 引数 `cmd_vel_topic` と `odom_topic` を指定します。

## Issue #245 への対処

[Issue #245](https://github.com/openspur/yp-spur/issues/245) の公式修正
[PR #248](https://github.com/openspur/yp-spur/pull/248)（commit `d653dee`）を、
CMakeからビルド用コピーへ **自動適用** します。
[`third_party/patches/0001-fix-tcflush-kernel-6.x.patch`](third_party/patches/0001-fix-tcflush-kernel-6.x.patch)
は公式コミットのパッチそのものです。

送信直後の `tcflush(..., TCOFLUSH)` は未送信データを破棄してしまいます。
Kernel 6.6以降のCDC-ACMドライバがこの処理に対応したことで問題が表面化しました。
公式修正に合わせ、`encode_write()` と `encode_int_write()` の両方から呼び出しを削除し、
不要になった `serial_flush_out()` の定義・宣言も削除します。
`third_party/yp-spur` のsubmodule本体・固定リビジョンは変更しません。

`colcon build` 時に
```
-- Applying yp-spur patch (0001-fix-tcflush-kernel-6.x)
```
もしくは
```
-- yp-spur patch already applied
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

coordinator を別端末で手動起動している場合は、従来通り `ypspur_node` だけを起動します。

```bash
ros2 launch ypspur_ros2 ypspur_ros2.launch.py
```

`ypspur-coordinator` も launch から同時起動する場合は、`start_coordinator:=true` と robot parameter file を指定します。この場合、`ypspur_node` は coordinator 初期化待ちとして 2 秒遅延起動します。

```bash
ros2 launch ypspur_ros2 ypspur_ros2.launch.py \
  start_coordinator:=true \
  coordinator_device:=/dev/ttyACM0 \
  coordinator_param:=<robot_param_file>
```

`drive_mode_manager` と組み合わせる場合、既定では `drive_mode_manager` の `/cmd_vel` と `ypspur_node` の `cmd_vel` が接続されます。明示する場合は以下のようにします。

```bash
ros2 launch ypspur_ros2 ypspur_ros2.launch.py cmd_vel_topic:=/cmd_vel odom_topic:=/ypspur_ros/odom
```

### 3. 動作確認

```bash
# 前進 0.1 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10

# odometry を監視
ros2 topic echo /ypspur_ros/odom
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
| launch `cmd_vel_topic` | string | `cmd_vel` | `cmd_vel` の remap 先。`drive_mode_manager` と接続する場合は既定または `/cmd_vel` |
| launch `odom_topic` | string | `/ypspur_ros/odom` | `odom` の remap 先。robot_navigator 等が前提とする既定値 |
| launch `start_coordinator` | bool | `false` | true の場合、launch から `ypspur-coordinator` を同時起動 |
| launch `coordinator_device` | string | `/dev/ttyACM0` | `ypspur-coordinator -d` に渡す device path |
| launch `coordinator_param` | string | `""` | `ypspur-coordinator -p` に渡す robot parameter file path |

## トラブルシューティング

### `YPSpur_init failed. Is ypspur-coordinator running?`

ypspur-coordinator が起動していないか、別ユーザで起動している (msgqueue が見えない)。
同じユーザで先に coordinator を起動してから ROS ノードを起動する。

### Coordinator がモータドライバを認識しない (Ubuntu 22.04+)

- `dmesg | grep ttyACM` でデバイスが見えているか確認
- ModemManager が `ttyACM*` を掴んでしまうことがある。`sudo systemctl stop ModemManager`
- パッチが適用されていない場合 (本パッケージ経由でビルドしていない場合) は Issue #245 の症状
  (接続拒否・異音・負荷で coordinator が落ちる) が出る

### odom が出ているのに位置が動かない

- coordinator のパラメータファイルでホイール直径やトレッド幅が合っていない
- `coordinate_system` がロボットの初期化方法と合っていない

## 設計

詳細は [`docs/design.md`](docs/design.md) を参照。

## ライセンス

MIT (yp-spur 本体も MIT)

## このPCのi-Cart middle（2026-09-22）

提供された `ダウンロード/icart-middle.param` を内容変更せず
`config/icart-middle.param` に格納。colcon buildでshare配下にも配置される。
車輪単体の起動は次のコマンドで行う（車輪制御が有効になる）。

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch ypspur_ros2 ypspur_ros2.launch.py start_coordinator:=true
```

既定のデバイスはT-frogのUSB固定名、パラメータは同梱ファイル。
`icart_bringup/params/hardware.yaml` も同じファイルをpackage URIで参照する。
以前に生成済みのsessionは `wheel.coordinator_param` を
`package://ypspur_ros2/config/icart-middle.param` に更新するか再生成する。
走行時は既存のdrive_mode_managerを通し、停止監視を有効にする。

### 速度・加速度の初期化

車輪ノードは接続後にゼロ速度を送り、`velocity_max` と `acceleration_max` を
YP-Spurへ設定する。coordinatorのユーザー上限は初期値ゼロのため、この設定なしでは
`cmd_vel` を受信しても走行しない。加速度の既定値は直進0.3 m/s²、旋回0.6 rad/s²。
これらの上限は正の有限値が必要。

2026-09-22: 旋回の角加速度を0.6から1.5 rad/s²へ調整。
角速度上限1.0 rad/sは維持し、静止から上限に達する指令上の時間は約1.67秒から0.67秒へ短縮する。
実車パラメータのMAX_ACC_W=3.28 rad/s²以下。角加速度は起動時設定のため反映には再起動が必要。
