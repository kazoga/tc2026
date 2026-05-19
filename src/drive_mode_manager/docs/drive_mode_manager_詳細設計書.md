# drive_mode_manager 詳細設計書

作成日: 2026-05-19

## 1. 文書目的・対象範囲

本書は、`drive_mode_manager` パッケージで実装する自律走行・手動走行切替機能、
手動速度指令生成、専用状態表示 GUI の詳細設計を定義する。

対象ノードは以下である。

| ノード | 実装予定ファイル | 役割 |
| --- | --- | --- |
| `manual_teleop_node` | `drive_mode_manager/manual_teleop_node.py` | `/joy` から手動 `/cmd_vel/manual` を生成する |
| `drive_cmd_mux_node` | `drive_mode_manager/drive_cmd_mux_node.py` | 自律 cmd と手動 cmd を排他的に選択し、最終 `/cmd_vel` を publish する |
| `drive_status_gui_node` | `drive_mode_manager/drive_status_gui_node.py` | 走行状態、出力 cmd、自律復帰カウントダウンを常時表示する |

本書は設計中仕様を確定するための文書であり、現時点ではパッケージ骨格と本設計書のみを
配置する。実装時は本書に従い、ROS 非依存ロジックを `drive_mode_core.py` と
`manual_teleop_core.py` に分離する。

参照した入力は、添付された自律走行・手動走行切替方式の検討資料、tc2025 ROS 1 資産の
`scripts/joystick_teleop.py`、`scripts/total_gui.py`、`launch/1_init_robot_tc2024_joystick.launch`、
および tc2026 の既存 `robot_navigator`, `ypspur_ros2`, `robot_console`, `route_msgs` の設計である。

## 2. 背景・要求・スコープ

tc2025 の `joystick_teleop.py` は、L1 相当のデッドマン入力が押されている間は手動速度指令を
`ypspur_ros/cmd_vel` へ出し、デッドマン OFF の間は外部 `cmd_vel` を ypspur 側へ通す構造であった。
この方式は単純で扱いやすい一方、自律・手動の運用状態、実出力ソース、自律復帰タイミングを
明示する interface が不足していた。

tc2026 では、`robot_navigator` が `/active_target` から自律速度指令を生成し、`ypspur_ros2` が
最終 `/cmd_vel` を受けて車体を駆動する。自律 cmd と手動 cmd が同じ `/cmd_vel` へ直接 publish
する構成は、出力元が不明確になり、安全判断と GUI 表示が難しくなる。

本パッケージの責務は以下である。

- `robot_navigator` の出力を `/cmd_vel/autonomous` へ分離する。
- PS3 コントローラー由来の手動入力を `/cmd_vel/manual` へ分離する。
- `drive_cmd_mux_node` が最終 `/cmd_vel` を唯一 publish する。
- 走行状態を `AUTONOMOUS` / `MANUAL` の 2 状態として管理する。
- 実出力ソースを `ZERO` / `AUTONOMOUS_CMD` / `MANUAL_CMD` として別途公開する。
- `AUTONOMOUS -> MANUAL` は誤操作を避ける複合操作でのみ成立させる。
- `MANUAL -> AUTONOMOUS` は L1 入力なしの継続と自律 cmd 有効性により自動復帰する。
- 自律復帰直後は 5 秒間ゼロ速度を出力し、復帰予定 cmd を GUI に表示する。

スコープ外は以下である。

- 物理緊急停止回路の代替。
- `robot_navigator` の追従アルゴリズム変更。
- `route_follower` の信号停止・手動開始イベント仕様変更。
- GPS/LiDAR 融合ローカライザーそのものの実装。
- `robot_console` 全体画面への統合表示。

## 3. 全体構成・アーキテクチャ

速度指令の流れは以下とする。

```text
robot_navigator
  -> /cmd_vel/autonomous

manual_teleop_node
  -> /cmd_vel/manual

drive_cmd_mux_node
  -> /cmd_vel

ypspur_ros2
  <- /cmd_vel
```

`drive_cmd_mux_node` は `/joy` も購読し、L1/PS ボタン状態、入力鮮度、長押し時間を
状態遷移判定に使う。`manual_teleop_node` も `/joy` を購読するが、こちらは stick 軸から
手動 Twist を生成するだけであり、最終出力の採否は `drive_cmd_mux_node` が決定する。

GUI は `/drive_mode_status` と `/cmd_vel` を主入力とする。GPS/RTK 状態や waypoint 情報は
補助表示であり、購読できない場合でも走行切替表示は継続する。

## 4. パッケージ構成・ファイル配置

推奨構成は以下である。

```text
src/drive_mode_manager/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── drive_mode_manager
├── drive_mode_manager/
│   ├── __init__.py
│   ├── drive_cmd_mux_node.py
│   ├── drive_mode_core.py
│   ├── manual_teleop_node.py
│   ├── manual_teleop_core.py
│   ├── drive_status_gui_node.py
│   └── gui_core.py
├── launch/
│   └── drive_mode_manager.launch.py
├── params/
│   └── default.yaml
├── docs/
│   └── drive_mode_manager_詳細設計書.md
└── tests/
    ├── test_drive_mode_core.py
    └── test_manual_teleop_core.py
```

現時点で作成するのは、ROS 2 パッケージとして認識できる最小骨格と本詳細設計書である。
`launch/`, `params/`, `tests/` は実装フェーズで追加する。

`DriveModeStatus.msg` は、GUI やログ解析など他パッケージからも参照される共有 interface である。
実装時は `route_msgs/msg/DriveModeStatus.msg` として追加する方針を基本とする。

## 5. 外部インタフェース仕様

### 5.1 `manual_teleop_node`

| 方向 | Topic | Type | QoS | 内容 |
| --- | --- | --- | --- | --- |
| Subscribe | `/joy` | `sensor_msgs/msg/Joy` | Reliable / Volatile / depth 10 | PS3 コントローラー入力 |
| Publish | `/cmd_vel/manual` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | 手動速度指令 |

`manual_teleop_node` は L1 押下中のみ stick 軸を Twist に変換する。L1 非押下、入力 timeout、
軸数不足、NaN/Inf 検出時はゼロ Twist を publish する。

### 5.2 `drive_cmd_mux_node`

| 方向 | Topic | Type | QoS | 内容 |
| --- | --- | --- | --- | --- |
| Subscribe | `/cmd_vel/autonomous` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | 自律速度指令 |
| Subscribe | `/cmd_vel/manual` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | 手動速度指令 |
| Subscribe | `/joy` | `sensor_msgs/msg/Joy` | Reliable / Volatile / depth 10 | 状態遷移判定用 controller 入力 |
| Publish | `/cmd_vel` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | `ypspur_ros2` へ渡す最終速度指令 |
| Publish | `/drive_mode_status` | `route_msgs/msg/DriveModeStatus` | Reliable / Volatile / depth 10 | 走行状態と出力状態 |

`/cmd_vel` は本ノードだけが publish する。`robot_navigator` は launch remap で
`/cmd_vel/autonomous` へ出力させる。

### 5.3 `drive_status_gui_node`

| 方向 | Topic | Type | QoS | 内容 |
| --- | --- | --- | --- | --- |
| Subscribe | `/drive_mode_status` | `route_msgs/msg/DriveModeStatus` | Reliable / Volatile / depth 10 | 走行切替状態 |
| Subscribe | `/cmd_vel` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | 最終出力の補助確認 |
| Subscribe | `/cmd_vel/autonomous` | `geometry_msgs/msg/Twist` | Reliable / Volatile / depth 1 | 復帰予定 cmd の補助確認 |
| Subscribe | `/rtk_gps/rtk_status` | `rtk_gps_um982_msgs/msg/RtkStatus` | Reliable / Volatile / depth 10 | 任意の RTK 表示 |
| Subscribe | `/follower_state` | `route_msgs/msg/FollowerState` | Reliable / Volatile / depth 10 | 任意の次 waypoint 表示 |
| Subscribe | `/manager_status` | `route_msgs/msg/ManagerStatus` | Reliable / Volatile / depth 10 | 任意の自律走行状態表示 |

GUI は表示専用とし、状態遷移コマンドや速度指令を publish しない。

## 6. パラメータ・設定仕様

### 6.1 `drive_cmd_mux_node`

| パラメータ | 型 | 既定値 | 内容 |
| --- | --- | --- | --- |
| `initial_mode` | string | `autonomous` | 起動直後の走行状態。`autonomous` または `manual` |
| `manual_transition_trigger` | string | `l1_ps_button_hold` | 手動遷移トリガ種別 |
| `manual_transition_hold_s` | double | `2.0` | L1 + PS 長押し判定時間 |
| `manual_to_auto_l1_released_s` | double | `1.0` | L1 入力なし継続判定時間 |
| `auto_resume_delay_s` | double | `5.0` | 自律復帰後にゼロ出力する猶予時間 |
| `autonomous_cmd_timeout_s` | double | `0.5` | 自律 cmd 有効期限 |
| `manual_cmd_timeout_s` | double | `0.3` | 手動 cmd 有効期限 |
| `joy_timeout_s` | double | `0.5` | `/joy` 入力有効期限 |
| `publish_rate_hz` | double | `20.0` | `/cmd_vel` と status の publish 周期 |
| `l1_button_index` | int | `4` | PS3 L1 ボタン index |
| `ps_button_index` | int | `16` | PS ボタン index。実機確認後に調整する |
| `max_autonomous_resume_linear_x` | double | `0.8` | GUI 警告用の復帰予定直進速度閾値 |
| `max_autonomous_resume_angular_z` | double | `1.2` | GUI 警告用の復帰予定角速度閾値 |

`ps_button_index` は controller driver により変わる可能性がある。PS ボタンが `/joy` で安定して
取得できない場合は、`manual_transition_trigger` を `l1_start_button_hold` などに変更できる
構成にする。

### 6.2 `manual_teleop_node`

| パラメータ | 型 | 既定値 | 内容 |
| --- | --- | --- | --- |
| `linear_axis` | int | `1` | 左 stick 縦軸 |
| `angular_axis` | int | `0` | 左 stick 横軸 |
| `linear_y_axis` | int | `-1` | 横移動軸。差動二輪では未使用 |
| `linear_scale` | double | `1.2` | 直進速度倍率 |
| `angular_scale` | double | `1.5` | 角速度倍率 |
| `linear_y_scale` | double | `0.5` | 横移動速度倍率 |
| `deadzone` | double | `0.05` | stick deadzone |
| `linear_axis_invert` | bool | `false` | 縦軸反転 |
| `angular_axis_invert` | bool | `false` | 横軸反転 |
| `enable_button` | int | `4` | L1 デッドマンボタン |
| `turbo_button` | int | `5` | R1 turbo ボタン |
| `turbo_ratio` | double | `1.5` | turbo 倍率 |
| `joy_timeout_s` | double | `0.5` | 入力 timeout |
| `publish_rate_hz` | double | `20.0` | `/cmd_vel/manual` publish 周期 |

tc2025 の `joystick_teleop.py` と同じ軸・ボタン・倍率を初期値として採用する。ただし、
自律 cmd の passthrough は本ノードでは行わない。

## 7. データモデル・内部状態

### 7.1 走行状態

走行状態は以下の 2 状態だけとする。

| 状態 | 意味 |
| --- | --- |
| `AUTONOMOUS` | 自律走行系を採用する状態 |
| `MANUAL` | 手動走行入力を採用可能な状態 |

### 7.2 実出力ソース

実出力ソースは、GUI とログで現在の `/cmd_vel` の由来を明示するために持つ。

| ソース | 意味 |
| --- | --- |
| `ZERO` | ゼロ Twist を出力中 |
| `AUTONOMOUS_CMD` | `/cmd_vel/autonomous` を出力中 |
| `MANUAL_CMD` | `/cmd_vel/manual` を出力中 |

`AUTONOMOUS` 状態でも、自律復帰カウントダウン中や自律 cmd timeout 時は `ZERO` になる。

### 7.3 `DriveModeStatus.msg` 案

```text
builtin_interfaces/Time stamp

uint8 MODE_AUTONOMOUS=1
uint8 MODE_MANUAL=2
uint8 mode

uint8 SOURCE_ZERO=0
uint8 SOURCE_AUTONOMOUS_CMD=1
uint8 SOURCE_MANUAL_CMD=2
uint8 output_source

bool joy_available
bool l1_pressed
bool ps_button_pressed
float32 ps_hold_progress_s

bool manual_input_active
bool autonomous_cmd_alive

bool auto_resume_pending
float32 auto_resume_remaining_s

float32 pending_autonomous_linear_x
float32 pending_autonomous_angular_z

float32 output_linear_x
float32 output_angular_z

string reason
```

`stamp` は `std_msgs/Header` ではなく単独 `builtin_interfaces/Time` とし、表示用 status として
frame_id を持たない。将来 frame_id が必要になった場合は `Header` への変更ではなく別 msg を検討する。

## 8. 処理フロー・状態遷移

### 8.1 `AUTONOMOUS -> MANUAL`

以下の条件が継続した場合に `MANUAL` へ遷移する。

```text
joy_available
and l1_pressed
and ps_button_pressed
and hold_time >= manual_transition_hold_s
```

L1 単独、PS 単独、stick 操作単独では遷移しない。遷移直後は `/cmd_vel` をゼロにし、
次周期から L1 押下中かつ手動 cmd 有効時のみ `/cmd_vel/manual` を採用する。

### 8.2 `MANUAL -> AUTONOMOUS`

以下の条件が成立した場合に `AUTONOMOUS` へ遷移する。

```text
l1_pressed == false が manual_to_auto_l1_released_s 以上継続
and autonomous_cmd_alive
```

`/joy` が timeout した場合は L1 入力なしとして扱う。ただし即座に自律 cmd を出力せず、
`auto_resume_delay_s` のカウントダウンを必ず経由する。

### 8.3 出力選択

`AUTONOMOUS` 状態の選択ルールは以下である。

| 条件 | 出力 | `output_source` |
| --- | --- | --- |
| 自律復帰カウントダウン中 | ゼロ Twist | `ZERO` |
| カウントダウン外かつ自律 cmd 有効 | 最新 `/cmd_vel/autonomous` | `AUTONOMOUS_CMD` |
| 自律 cmd timeout | ゼロ Twist | `ZERO` |

`MANUAL` 状態の選択ルールは以下である。

| 条件 | 出力 | `output_source` |
| --- | --- | --- |
| L1 押下中かつ手動 cmd 有効 | 最新 `/cmd_vel/manual` | `MANUAL_CMD` |
| L1 押下中かつ手動 cmd timeout | ゼロ Twist | `ZERO` |
| L1 非押下 | ゼロ Twist | `ZERO` |
| `/joy` timeout | ゼロ Twist | `ZERO` |

## 9. 主要アルゴリズム・判定ロジック

### 9.1 `JoyState` 正規化

`/joy` callback では、ボタン配列長を確認して L1/PS の押下状態を抽出する。index が範囲外の場合は
押下なしとして扱い、`reason` に `joy_button_index_out_of_range` を設定する。

入力鮮度は `last_joy_time` と node clock の差で判定する。`joy_timeout_s` を超えた場合は
`joy_available=false` とし、L1/PS は false 扱いにする。

### 9.2 `Twist` 有効性判定

`Twist` は以下を満たす場合だけ有効とする。

- 最新受信時刻から timeout を超えていない。
- `linear.x`, `linear.y`, `angular.z` が NaN/Inf ではない。
- 設計上使わない成分が非ゼロでも即異常にはしないが、最終出力では `linear.x`, `angular.z` を主表示とする。

### 9.3 自律復帰予定 cmd 表示

自律復帰カウントダウン中は、最新 `/cmd_vel/autonomous` の `linear.x` と `angular.z` を
`pending_autonomous_*` に格納する。復帰予定 cmd が timeout した場合は、
`auto_resume_pending=true` のまま出力はゼロを維持し、`reason=autonomous_cmd_stale` とする。

## 10. QoS・並行性・タイミング設計

`drive_cmd_mux_node` は 20Hz timer で状態更新、出力選択、status publish を行う。
購読 callback は最新値と受信時刻の更新だけを行い、状態遷移は timer に集約する。

QoS は速度指令系を Reliable / Volatile / depth 1 とする。速度指令は最新値が重要であり、
古い queue を溜めない。`/drive_mode_status` は GUI 表示向けに depth 10 とする。

`drive_status_gui_node` は tkinter の GUI スレッドと rclpy executor スレッドを分ける。
ROS callback から tkinter widget を直接更新せず、`gui_core.py` の dataclass snapshot を
lock 付きで更新し、GUI スレッドが 100ms から 200ms 周期で描画する。

## 11. 起動・終了・launch 設計

`drive_mode_manager.launch.py` は以下を起動する。

| ノード | 既定起動 | 備考 |
| --- | --- | --- |
| `manual_teleop_node` | true | `/joy` から `/cmd_vel/manual` を生成する |
| `drive_cmd_mux_node` | true | `/cmd_vel` の唯一の publish 元 |
| `drive_status_gui_node` | true | 実機運用時は別画面表示を推奨 |

`tc2026_system_bringup` から実機 profile を起動する場合は、`robot_navigator` の
`cmd_vel_topic` を `/cmd_vel/autonomous` に remap し、`ypspur_ros2` は `/cmd_vel` を購読する。

終了時、`drive_cmd_mux_node` はゼロ Twist を publish してから shutdown する。
ただし、最終停止は `ypspur_ros2` の timeout と物理停止系にも依存するため、本ノード単独を
安全停止の唯一手段とはしない。

## 12. エラー処理・ログ・診断

| 事象 | 処理 | ログ |
| --- | --- | --- |
| `/joy` timeout | L1/PS 非押下扱い、出力ゼロ、MANUAL では自律復帰判定対象 | `warn` を throttle |
| PS ボタン index 不正 | 手動遷移は成立させない | 起動時 `warn`、status reason |
| 自律 cmd timeout | `AUTONOMOUS` でも出力ゼロ | `warn` を throttle |
| 手動 cmd timeout | `MANUAL` でも出力ゼロ | `warn` を throttle |
| NaN/Inf Twist | 該当 cmd を無効扱い | `error` を throttle |
| status publish 失敗 | 次周期で再試行 | `warn` |

通常の状態遷移は `info`、周期的な状態詳細は `debug` とする。

## 13. UI・可視化仕様

専用 GUI は `robot_console` とは別ウィンドウとし、運用者が常時確認できる表示にする。
必須表示は以下である。

| 表示 | 内容 |
| --- | --- |
| Drive Mode | `AUTONOMOUS` / `MANUAL` |
| Output Source | `AUTONOMOUS_CMD` / `MANUAL_CMD` / `ZERO` |
| Output Cmd | `linear.x`, `angular.z` |
| Direction | 前進/後退/停止、左旋回/右旋回/直進 |
| Auto Resume | active/inactive、残り秒数 |
| Pending Autonomous Cmd | 復帰時に接続予定の `linear.x`, `angular.z` |
| Joy | connected/unavailable、L1、PS hold progress |
| Reason | 現在の出力理由または停止理由 |

推奨表示は以下である。

- GPS/RTK state、衛星数、heading 有効性、データ鮮度。
- `follower_state` の active waypoint label と距離。
- `manager_status` または `follower_state` 由来の自律走行状態。

GUI は明るい警告色と大きな残り秒数表示を使い、自律復帰カウントダウン中に
「いつ」「どの方向に」「どの速度で」動き出すかを一目で確認できるようにする。

## 14. 依存関係・ビルド設定

`drive_mode_manager` は `ament_python` パッケージとする。

直接依存は以下である。

| 依存 | 用途 |
| --- | --- |
| `rclpy` | ROS 2 Python node |
| `geometry_msgs` | `Twist` |
| `sensor_msgs` | `Joy` |
| `std_msgs` | 補助 topic |
| `builtin_interfaces` | `DriveModeStatus.stamp` |
| `route_msgs` | `DriveModeStatus`, `FollowerState`, `ManagerStatus` |
| `rtk_gps_um982_msgs` | GUI の RTK 状態表示 |
| `python3-tk` | 専用 GUI |

`DriveModeStatus.msg` を `route_msgs` に追加する実装時は、`route_msgs/CMakeLists.txt` に msg を追加し、
`drive_mode_manager/package.xml` の `route_msgs` 依存を維持する。

## 15. テスト計画・受け入れ条件

優先して ROS 非依存コアの pytest を整備する。

| テスト | 対象 | 観点 |
| --- | --- | --- |
| `test_manual_transition_requires_l1_and_ps_hold` | `drive_mode_core.py` | L1 単独や stick 単独で手動遷移しない |
| `test_manual_to_auto_requires_l1_release_and_auto_alive` | `drive_mode_core.py` | L1 押下中の意図的停止では自律復帰しない |
| `test_auto_resume_outputs_zero_until_delay_elapsed` | `drive_mode_core.py` | カウントダウン中はゼロ出力 |
| `test_autonomous_cmd_timeout_outputs_zero` | `drive_mode_core.py` | 自律 cmd timeout 時にゼロ出力 |
| `test_manual_deadman_outputs_zero_when_l1_released` | `manual_teleop_core.py` | L1 非押下でゼロ Twist |
| `test_axis_deadzone_and_invert` | `manual_teleop_core.py` | deadzone、反転、倍率 |

受け入れ条件は以下である。

- `pytest src/drive_mode_manager/tests` が成功する。
- `colcon build --symlink-install --packages-select route_msgs drive_mode_manager` が成功する。
- `colcon build --packages-select route_msgs drive_mode_manager` が成功する。
- 実機 `/joy` で L1 と PS ボタン index を確認し、PS が不安定な場合は代替 trigger を決める。
- ROS 実行確認が必要な GUI 表示、controller 入力、実機駆動は自動テスト外の未確認事項として扱う。

## 16. 互換性・移行・影響範囲

`robot_navigator` は既存の `cmd_vel_topic` launch 引数で `/cmd_vel/autonomous` へ変更するため、
ノード内部 API の変更は不要である。

`ypspur_ros2` は引き続き `/cmd_vel` を購読するため変更不要である。ただし、実機 bringup では
`/cmd_vel` の publish 元が `drive_cmd_mux_node` だけになるよう、他ノードの remap を確認する。

`robot_console` は既存どおり `/cmd_vel` を表示できる。ただし、切替状態と復帰カウントダウンは
専用 GUI を正本表示とし、`robot_console` へ統合する場合も `/drive_mode_status` の購読表示に留める。

`route_msgs` に `DriveModeStatus.msg` を追加するため、interface package の再ビルドが必要になる。
既存 msg/srv の field は変更しない。

tc2025 の `joystick_teleop.py` から継承するのは、Joy 軸変換、deadzone、L1 デッドマン、
timeout 停止、turbo 倍率である。自律 cmd passthrough と waypoint flag publish は移植しない。

## 17. 未決事項・今後の拡張

- PS ボタンが `/joy` で安定して取得できるか。実機 controller で確認する。
- PS ボタンが使えない場合の代替複合操作を決める。
- `manual_transition_hold_s=2.0`、`manual_to_auto_l1_released_s=1.0`、`auto_resume_delay_s=5.0` の
  実運用値を確認する。
- 自律復帰予定速度が閾値を超えた場合、GUI 表示だけにするか、mux 側で復帰を保留するかを決める。
- GPS/RTK、次 waypoint、自律走行状態の表示元を `/drive_mode_status` に含めるか、
  既存 status topic の購読表示に留めるかを決める。
- `DriveModeStatus.msg` を `route_msgs` に置く方針で問題ないかを合意する。
- 専用 GUI をどの PC・画面で表示するかを運用手順で決める。

## 18. 改版履歴

| 版 | 日付 | 変更概要 |
| --- | --- | --- |
| 0.1 | 2026-05-19 | 初版。添付検討資料と tc2025 ROS 1 資産をもとに、`drive_mode_manager` の責務、topic、状態遷移、GUI、テスト計画を整理した |
