# rtk_gps_um982 設計書

## 1. 目的・スコープ

Unicore UM982 受信機 (デュアルアンテナ RTK GNSS) のデータを ROS 2 Jazzy 上で
パブリッシュするドライバパッケージ。ベースライブラリ
[UM982-RTK-GPS-Library](https://github.com/t-nakabayashi/UM982-RTK-GPS-Library)
(Python・MIT) を内部で利用し、シリアル接続と NTRIP 接続を一つのノードで完結させる。

**スコープ内**
- UM982 とのシリアル通信 (115200 baud デフォルト)
- 位置 (lat/lon/alt)、デュアルアンテナ heading/pitch、RTK 状態の公開
- NTRIP 経由 RTCM3 補正の取り込み (ノード内クライアント)
- ROS パラメータによる設定 (シリアル, NTRIP, 出力レート, frame_id)

**スコープ外 (将来検討)**
- TF 配信 (`map → gps_link` 等)、`navsat_transform_node` 連携は利用側で構築する想定
- C++ 実装、lifecycle node 化
- 内部 RTCM 中継 (他ノードから RTCM を流す構成)

## 2. 全体構成

```
                 ┌────────────────────────────────────────┐
                 │             rtk_gps_um982_node          │
                 │  (rclpy.Node, Python, single process)   │
                 │                                         │
   /dev/ttyUSBx  │  ┌──────────────┐    ┌──────────────┐  │
   ─────────────►│  │ UM982Client  │◄──►│  Publisher   │  │── /rtk_gps/fix         (NavSatFix)
       serial    │  │ (lib)        │    │  Layer       │  │── /rtk_gps/heading     (Imu)
                 │  │  - reader    │    │              │  │── /rtk_gps/rtk_status  (RtkStatus)
                 │  │  - ntrip     │    └──────────────┘  │── /diagnostics         (任意・後追加)
   NTRIP caster  │  └──────────────┘                      │
   ─────────────►│       ▲                                │
       TCP/HTTP  │       │                                │
                 │       └── 別スレッドで RTCM 受信         │
                 └────────────────────────────────────────┘
```

ノード起動時に `UM982Client` を生成し、`set_position_callback()` で受信ごとに
パブリッシュする (タイマーポーリングではなくコールバック駆動。理由は §6)。

## 3. パッケージ構成

ROS 2 のお作法に従い、メッセージ定義は別パッケージに切る (ament_python では
`rosidl` を直接扱いにくいため、msgs は ament_cmake で構築する)。

```
src/
├── rtk_gps_um982/                 # ドライバ本体 (ament_python)
│   ├── package.xml
│   ├── setup.py / setup.cfg
│   ├── resource/rtk_gps_um982
│   ├── rtk_gps_um982/
│   │   ├── __init__.py
│   │   ├── driver_node.py         # メインノード実装
│   │   └── converters.py          # PositionData → ROS msg 変換
│   ├── launch/
│   │   └── rtk_gps_um982.launch.py
│   ├── config/
│   │   └── default.yaml           # パラメータ既定値
│   ├── docs/
│   │   └── design.md              # 本書
│   └── test/
│       └── test_converters.py     # 単体テスト (純関数だけ対象)
│
└── rtk_gps_um982_msgs/            # メッセージ定義 (ament_cmake)
    ├── package.xml
    ├── CMakeLists.txt
    └── msg/
        └── RtkStatus.msg
```

ベースライブラリ (`UM982-RTK-GPS-Library`) は **`pip install -e .`** で取り込むか
**git submodule** で取り込むかを実装フェーズで決定する (どちらにせよ
`package.xml` の `<exec_depend>python3-pyserial</exec_depend>` は必須)。

## 4. ノード設計

### 4.1 ノード名

- 既定: `rtk_gps_um982_node`
- namespace 既定: `rtk_gps` (launch ファイルで `PushRosNamespace` を当てる)

### 4.2 トピック (publisher)

| Topic                   | Type                              | 頻度        | 説明                                            |
| ----------------------- | --------------------------------- | ----------- | ----------------------------------------------- |
| `~/fix`                 | `sensor_msgs/NavSatFix`           | 受信ごと    | 緯度経度高度、共分散は HDOP / baseline std 由来 |
| `~/heading`             | `sensor_msgs/Imu`                 | 受信ごと    | デュアルアンテナ orientation (REP-103 ENU)      |
| `~/rtk_status`          | `rtk_gps_um982_msgs/RtkStatus`    | 受信ごと    | RTK 種別・衛星数・baseline・補正経過時間など    |

3 トピックすべてに同一 `header.stamp` (GNSS UTC をノード時計に変換) と
同一 `header.frame_id` を入れる。

### 4.3 frame_id 規約

- 既定 `gps_link` (パラメータ `frame_id` で変更可)
- 上位 robot_localization が `gps_link → base_link` の static TF を持っている前提

### 4.4 ROS パラメータ

| パラメータ            | 型     | 既定値          | 説明                                                  |
| --------------------- | ------ | --------------- | ----------------------------------------------------- |
| `serial.port`         | string | `/dev/ttyUSB0`  | UM982 のシリアルデバイス                              |
| `serial.baud`         | int    | `115200`        | ボーレート                                            |
| `output_rate_hz`      | int    | `10`            | UM982 の出力レート (start 時に `set_output_rate`)     |
| `frame_id`            | string | `gps_link`      | 全 publish の header.frame_id                         |
| `ntrip.enabled`       | bool   | `false`         | NTRIP クライアントを有効化                            |
| `ntrip.host`          | string | `""`            | NTRIP caster ホスト                                   |
| `ntrip.port`          | int    | `2101`          | NTRIP caster ポート                                   |
| `ntrip.mountpoint`    | string | `""`            | mountpoint                                            |
| `ntrip.user`          | string | `""`            | ユーザ名                                              |
| `ntrip.password`      | string | `""`            | パスワード (起動時警告: 平文。実運用は env で渡す)    |
| `publish.navsatfix`   | bool   | `true`          | `~/fix` を publish するか                             |
| `publish.imu_heading` | bool   | `true`          | `~/heading` を publish するか                         |
| `publish.rtk_status`  | bool   | `true`          | `~/rtk_status` を publish するか                      |
| `min_fix_for_publish` | string | `standalone`    | 公開する最低 fix 種別 (`none`/`standalone`/`float`/`fix`) |
| `stamp_source`        | string | `gnss_utc`      | `gnss_utc` / `ros_time` / `pps_edge` (§13)            |
| `transport_delay_ms`  | int    | `0`             | `gnss_utc` 使用時のシリアル→コールバック遅延補正        |

`config/default.yaml` で全てを宣言し、launch ファイルから差し替え可能にする。

## 5. メッセージ仕様

### 5.1 `sensor_msgs/NavSatFix`

| field                          | 値                                                                     |
| ------------------------------ | ---------------------------------------------------------------------- |
| `header.stamp`                 | GNSS UTC (既定)。詳細は §13 時刻同期                                   |
| `header.frame_id`              | パラメータ `frame_id`                                                  |
| `status.status`                | RTK 種別を以下にマップ: STANDALONE→FIX, FLOAT→GBAS_FIX, FIX→GBAS_FIX  |
| `status.service`               | `SERVICE_GPS \| SERVICE_GLONASS \| SERVICE_GALILEO \| SERVICE_COMPASS` |
| `latitude / longitude`         | `PositionData.lat / .lon`                                              |
| `altitude`                     | `PositionData.alt`                                                     |
| `position_covariance`          | 対角成分のみ。水平: `(HDOP × σ₀)²`、垂直: `(VDOP × σ₀)²`。σ₀ は実装時調整 |
| `position_covariance_type`     | `COVARIANCE_TYPE_DIAGONAL_KNOWN`                                       |

※ FLOAT と FIX を NavSatFix で区別できないので、厳密な状態は `~/rtk_status` を見てもらう。

### 5.2 `sensor_msgs/Imu` (heading 用)

| field                                      | 値                                                              |
| ------------------------------------------ | --------------------------------------------------------------- |
| `header`                                   | 共通                                                            |
| `orientation`                              | (yaw = π/2 − heading_rad, pitch, roll=0) の quaternion (ENU)    |
| `orientation_covariance`                   | baseline_std から導出した分散を yaw/pitch に充当                |
| `angular_velocity` / `linear_acceleration` | 全 0 + 共分散 `[-1, 0, 0, ...]` で「未提供」を明示              |

**Note:** UM982 の heading は真北基準の方位角 (時計回り)。REP-103 の ENU では yaw は
東基準で反時計回り。よって `yaw_enu = π/2 − heading_rad`。

### 5.3 `rtk_gps_um982_msgs/RtkStatus.msg`

```
std_msgs/Header header

# RTK 種別
uint8 RTK_NONE       = 0
uint8 RTK_STANDALONE = 1
uint8 RTK_FLOAT      = 2
uint8 RTK_FIX        = 3
uint8 rtk_state

uint8  num_satellites
float32 hdop

# heading/pitch (deg, 真北 CW)
float32 heading_deg
float32 pitch_deg

# baseline
float32 baseline_length_m
float32 baseline_std_m

# 補正
float32 correction_age_s
uint32  rtcm_bytes_received

# 生 fix の経度緯度高度 (参照用)
float64 latitude
float64 longitude
float32 altitude
```

## 6. スレッド・並行性

- `UM982Client` は内部で別スレッドが走り続ける (シリアル & NTRIP の受信)
- ROS への publish は **ライブラリのコールバック → rclpy 側** で行う
  - rclpy のパブリッシャはスレッドセーフ
  - ただしコールバックで重い処理はしないこと (msg 構築のみ)
- `MultiThreadedExecutor` は不要。`SingleThreadedExecutor` 既定で OK

## 7. エラー処理・診断

- シリアルオープン失敗: ノード起動を失敗させる (例外を吐いて exit)
- 起動後の通信途絶: WARN ログ + `last_received` を監視して 5 秒以上途絶で ERROR
- NTRIP 接続失敗: WARN ログ・指数バックオフで自動再接続 (ライブラリ機能に依存)
- `min_fix_for_publish` 未満の品質では publish しない (ログには debug で記録)

将来 `diagnostic_updater` で `/diagnostics` に出すのを推奨 (本フェーズは見送り)。

## 8. 依存関係

`package.xml` (`rtk_gps_um982`):

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>rtk_gps_um982_msgs</exec_depend>
<exec_depend>python3-pyserial</exec_depend>
<!-- UM982-RTK-GPS-Library は pip / submodule で取り込む -->
```

`package.xml` (`rtk_gps_um982_msgs`):

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<depend>std_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 9. ビルド・起動

```bash
cd ~/colcon_ws
colcon build --packages-select rtk_gps_um982_msgs rtk_gps_um982
source install/setup.bash
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py \
    serial_port:=/dev/ttyUSB0 ntrip_enabled:=true ntrip_host:=...
```

## 10. テスト方針

- 単体: `converters.py` の純関数 (PositionData → msg) を pytest で網羅
  - heading→quaternion 変換、共分散行列、fix 種別マップ、フィルタ判定
- 結合: 実機接続テスト (CI ではなく手動。手順をこの docs/ に追記予定)
- 静的: `ament_flake8` / `ament_pep257` を有効化

## 11. 受け入れ条件 (Definition of Done)

- [ ] UM982 を接続するだけで `/rtk_gps/fix` が出る
- [ ] NTRIP パラメータを与えると RTCM が流れ、`rtk_status.rtk_state` が `RTK_FIX` になる
- [ ] デュアルアンテナ運用で `/rtk_gps/heading` の yaw が物理的に正しい方向を示す
- [ ] `ros2 param set` でランタイムに `min_fix_for_publish` を切り替えられる
- [ ] `colcon test` がパスする

## 12. 将来拡張

- lifecycle node 化
- `diagnostic_updater` で `/diagnostics` 配信
- 受信 RTCM の中継トピック (`~/rtcm` を購読する側になる構成への切り替え)
- PC への PPS 直結 (§13 参照)、PTP grandmaster 経由でのサブμs 同期
- ROS 2 Humble 等への対応 (`python_setup_tools` 等の差分対応)

## 13. 時刻同期

### 13.1 全体方針

UM982 の PPS は **Lidar 側にのみ** 配線し、PC への PPS 配線は行わない構成を前提とする
(物理配線困難・USB-Serial 経由では精度が出ないため)。代わりに **GPS msg と Lidar packet を
両方 GPS UTC でスタンプ** することで、PC system clock の精度に依存せず両者を直接同期する。

```
                   ┌── PPS + NMEA ──► Lidar (内部時計を PPS 同期 → GPS UTC stamp)
   UM982 ──────────┤
                   └── Serial ──► rtk_gps_um982_node
                                       │
                                       ├─► msg.header.stamp = GNSS UTC
                                       │
                                       └─► (任意) gpsd ──► chrony (NMEA only refclock)
                                                              │
                                                              ▼
                                                       system clock
                                                       (±30〜50ms)
```

- **GPS msg と Lidar packet は同じ GPS UTC epoch** → `message_filters::sync` 等で直接同期可
- system clock は gpsd+chrony で NMEA only 同期 (PPS なし) → ±30〜50ms。GPS/Lidar 以外の
  センサ (カメラ等) も GPS 時刻に近づく
- chrony PPS 同期がなくても **GPS と Lidar の相対同期は μs 級** を維持

### 13.2 `stamp_source` パラメータ

| 値           | 内容                                                                       |
| ------------ | -------------------------------------------------------------------------- |
| `gnss_utc` (既定) | `PositionData.timestamp` (NMEA UTC, Unix epoch) を `header.stamp` に。Lidar との直接同期が目的のときはこれ |
| `ros_time`   | `node.get_clock().now()`。chrony PPS 同期済 (= system clock = GPS time) の環境用 |
| `pps_edge`   | 受信フレーム内タイムタグ + 直前 PPS エッジを再構築。実装はフェーズ 2 以降 (要 /dev/pps0) |

`gnss_utc` 使用時、シリアル受信からコールバックまでの遅延 (~5〜20ms 程度) を補正したい場合は
`transport_delay_ms` を負方向に与える (例: 受信に 10ms かかるなら `transport_delay_ms: -10`)。

### 13.3 system clock 側の設定 (推奨手順)

パッケージ同梱の `scripts/` に以下を置き、README で適用手順を説明する。実際の `/etc/` 配置は
パッケージインストールでは行わず手動。

```
scripts/
├── chrony-gpsd.conf.sample      # chrony.conf 追記分 (refclock SHM 0 ...)
├── gpsd.default.sample          # /etc/default/gpsd の例 (DEVICES=/dev/ttyUSB0)
└── README.md                    # 適用手順 (apt install / systemctl enable)
```

設定の骨子 (PPS なし、NMEA SHM のみ):

```
# /etc/chrony/chrony.conf 追記分
refclock SHM 0 refid GPS precision 1e-1 offset 0.0 delay 0.2
```

期待精度: ±30〜50ms (PPS あり時の ±1μs に比べると粗いが、ROS msg 同士の同期には十分)。

### 13.4 将来 PPS を引けるようになった場合

- PC が RPi/Jetson なら GPIO + pps-gpio overlay で `/dev/pps0` を作る
- 計算機を変える必要がなければ、別途 GPSDO 付き Stratum 1 NTP サーバを LAN に立てる
- Lidar が PTP grandmaster 対応なら PTP 経由で PC 時計同期

いずれの場合も `stamp_source: ros_time` に切り替えれば node の修正なしで対応可能。

---

参考:
- ベースライブラリ: https://github.com/t-nakabayashi/UM982-RTK-GPS-Library
- REP-103 (Coordinate Conventions): https://www.ros.org/reps/rep-0103.html
- REP-105 (Coordinate Frames): https://www.ros.org/reps/rep-0105.html
