# rtk_gps_um982

Unicore UM982 デュアルアンテナ RTK GNSS 受信機を ROS 2 (Jazzy) で扱うドライバパッケージ。
[UM982-RTK-GPS-Library](https://github.com/t-nakabayashi/UM982-RTK-GPS-Library) を内部利用し、
シリアル接続 + NTRIP 受信を 1 ノードで完結させる。

## パブリッシュするトピック

| Topic                       | Type                                | 説明                              |
| --------------------------- | ----------------------------------- | --------------------------------- |
| `~/fix`                     | `sensor_msgs/NavSatFix`             | 緯度経度高度、HDOP 由来の共分散   |
| `~/heading`                 | `sensor_msgs/Imu`                   | デュアルアンテナの orientation (REP-103 ENU) |
| `~/rtk_status`              | `rtk_gps_um982_msgs/RtkStatus`      | RTK 種別・衛星数・baseline・RTCM 累計バイト等 |

デフォルト namespace は `rtk_gps` なので、外から見える名前は `/rtk_gps/fix` などになる。

## 必要環境

- Ubuntu 24.04
- ROS 2 Jazzy (`/opt/ros/jazzy` インストール済)
- Python 3.12
- UM982 受信機 (USB-Serial or 直結 UART)

## ビルド手順

```bash
cd ~/colcon_ws
# 初回 / 更新時に submodule を取得
git submodule update --init --recursive

# 必要なら apt で依存をインストール
sudo apt install python3-serial

colcon build --packages-select rtk_gps_um982_msgs rtk_gps_um982
source install/setup.bash
```

## 起動

### シンプル起動 (デフォルト)

```bash
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py
```

### 自前 YAML を渡す

```bash
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py \
    config:=/path/to/my_params.yaml
```

### ノード直接起動

```bash
ros2 run rtk_gps_um982 rtk_gps_um982_node \
    --ros-args -p serial.port:=/dev/ttyACM0 -p output_rate_hz:=20
```

## パラメータ

`config/default.yaml` がデフォルト。主なもの:

| パラメータ            | 型     | 既定値          | 説明                                                  |
| --------------------- | ------ | --------------- | ----------------------------------------------------- |
| `serial.port`         | string | `/dev/ttyUSB0`  | UM982 のシリアルデバイス                              |
| `serial.baud`         | int    | `115200`        | ボーレート                                            |
| `output_rate_hz`      | int    | `10`            | UM982 の出力レート                                    |
| `frame_id`            | string | `gps_link`      | 全 publish の `header.frame_id`                       |
| `stamp_source`        | string | `gnss_utc`      | `gnss_utc` / `ros_time` / `pps_edge`                  |
| `transport_delay_ms`  | int    | `0`             | `gnss_utc` 用の受信遅延補正 (負値で前倒し)            |
| `ntrip.enabled`       | bool   | `false`         | NTRIP クライアント有効化                              |
| `ntrip.host`          | string | `""`            | caster ホスト                                         |
| `ntrip.port`          | int    | `2101`          | caster ポート                                         |
| `ntrip.mountpoint`    | string | `""`            | mountpoint                                            |
| `ntrip.user`          | string | `""`            | ユーザ名                                              |
| `ntrip.password`      | string | `""`            | パスワード (平文。実運用は env 経由で渡すこと)        |
| `publish.navsatfix`   | bool   | `true`          | `~/fix` を publish するか                             |
| `publish.imu_heading` | bool   | `true`          | `~/heading` を publish するか                         |
| `publish.rtk_status`  | bool   | `true`          | `~/rtk_status` を publish するか                      |
| `min_fix_for_publish` | string | `standalone`    | `none`/`standalone`/`dgps`/`float`/`fix`              |
| `hdop_sigma`          | float  | `1.0`           | NavSatFix の position_covariance スケール             |

## 時刻同期

PPS は **Lidar 側にのみ** 配線する前提。GPS msg は GNSS UTC を `header.stamp` に
入れるため、Lidar packet (PPS 同期) と同じ epoch で比較できる。
詳細は [`docs/design.md`](docs/design.md) §13 と [`scripts/README.md`](scripts/README.md) を参照。

## トラブルシューティング

### `Permission denied: '/dev/ttyUSB0'`

dialout グループに入っていない。

```bash
sudo usermod -aG dialout $USER
# ログアウト/ログインし直す
```

### `Failed to start UM982Client` (ノードが即落ち)

- ケーブル / 給電を確認
- `dmesg | tail` で `ttyUSBx` が認識されているか確認
- `screen /dev/ttyUSB0 115200` 等で生 NMEA が流れているか確認

### NTRIP に繋がらない

- `~/rtk_status` の `rtcm_bytes_received` が増えていなければ caster 到達不可
- ホスト/ポート/mountpoint/credentials を確認
- ファイアウォール (caster は TCP 2101 が多い)

### RTK Fix にならない

- `~/rtk_status.correction_age_s` を見て補正が新しい (~1-3s) か確認
- アンテナの空が見えているか、マルチパス源 (建物近接) がないか
- 基準局までの距離 (10km 程度まで RTK Fix 期待、それ以上は厳しい)

## テスト

```bash
colcon test --packages-select rtk_gps_um982
colcon test-result --verbose
```

純関数テストのみ (`test/test_converters.py`、16 ケース)。実機テストは手動。

## 設計

詳細は [`docs/design.md`](docs/design.md) を参照。

## ライセンス

MIT
