# robot_console パッケージ README (phase3 実装版)

## 概要
現行の正式UIは **PyQt5版 `robot_console_qt`** です。ROS通信・状態集約は
`RobotConsoleNode` / `ConsoleCore`、遠隔閲覧は同じSnapshotを読むHTML版が担当します。
下記の旧画面説明は互換用tkinter版です。新規の実機共通起動はPyQt5版を使用します。

### 現行の5タブ（2026-09-20）

| タブ | 内容 |
|---|---|
| ダッシュボード | 運行フェーズ、経路進捗、速度、手動操作、起動操作、ノード稼働、RTK・基地局要約 |
| 自己位置・センサ情報 | 地図、現在位置・経路、配信されているセンサ／認識画像 |
| 起動・設定 | 実機／模擬・手動／自律の選択、起動予定、各ノードの設定 |
| コンソールログ | ノードごとのログ確認 |
| GNSS・基地局 | 接続先・マウントポイント・RTCM受信状態、GNSS品質・アンテナ情報 |

追加画面の仕様・検証・起動環境は [GNSS・基地局表示](docs/gnss_station_ui.md) を参照。


## 主な機能
- `/route_state`・`/manager_status`・`/follower_state` などのトピックを購読し、走行状況や再計画履歴をカード形式で可視化。
- `/sensor_viewer` および外部カメラ映像（走行・信号監視）を 3 つの画像パネルに表示し、障害物ヒントをオーバレイ。
- `manual_start`・`sig_recog`・`road_blocked`・`obstacle_avoidance_hint` の送信 UI を備え、運用者がラッチ値や回避指示を即時に発行可能。
- `NodeLaunchManager` により `ros2 launch` コマンドを GUI から起動／停止し、主要ノードの稼働状況とログをサイドバーとタブで確認。
- 走行距離・速度・目標到達率を自動算出し、閾値を超えた場合に色分けで警告。
- `tools/mock_ui.py` を用いたダミーデータ表示と `tools/tests/` によるロジック単体テストで回帰検出に対応。

## 画面構成
### Dashboard タブ
- **ステータスカード列**：ルート進捗、フォロワ状態、速度・目標距離カードを 5Hz 以内で更新。
- **イベントバナー**：`road_blocked` → `manual_start` → `sig_recog` の優先順位で最新イベントを表示。60 秒経過で自動クリア。
- **制御コマンドタブ**：各トピックの送信 UI を Notebook 形式でまとめ、Spinbox やラジオボタンで値を入力。`frame_image_path` タグでは静止画パスを入力して `/frame_image_path` トピックへ単発 publish できます。送信結果は最終送信値と時刻として即座に反映されます。
- **画像パネル**：ルート地図（`/active_route` の付帯画像）、障害物ビュー（`/sensor_viewer`）、カメラ映像（`/perception/road_blockage/decision_image`・`/perception/traffic_signal/decision_image`）。レターボックス処理でアスペクト比を保持し、障害物ヒント値を左上にオーバレイ表示。
- **ノード起動サイドバー**：主要ノードカードに加え `Drive Mode Manager`、
  `Road Blockage Detector`、`Traffic Signal Recognizer` カードを配置。各カードは対応パッケージの統合 launch を起動し、
  下流判定ノードと用途別 `yolo_detector` インスタンスを同時に立ち上げます。

### Console Logs タブ
- ノードごとに最新ログをリングバッファで保持。GUI 右クリックからコピーでき、検索バーでフィルタリング可能です。
- 各セクションには `RUNNING/STOPPED/ERROR` インジケータと、直近の起動／停止時刻が表示されます。
- 認識系 2 カードも他ノード同様にパラメータ一覧・コンソールログタブへ追加し、選択した
  YAML 内容表示やログファイルの直接オープンが可能です。

## 起動方法
### 通常起動
```bash
ros2 launch robot_console robot_console.launch.py \
  config_file:=/path/to/robot_console.yaml
```
- `config_file` は任意の YAML で、運用環境に合わせて `ros2_src/robot_console/config/` などに配置してください（ディレクトリが存在しない場合は作成が必要です）。省略時はノード組み込みの既定値が使用されます。
- GUI は 1280x720 を基準解像度とし、リサイズ時は比率を維持します。

### 主要パラメータ
| パラメータ | 型 | 既定値 | 説明 |
|------------|----|--------|------|
| `ui.refresh_period_ms` | int | 200 | GUI が `GuiCore.snapshot()` を呼ぶ周期。|
| `ui.image_rate_limit_hz.*` | double | route:2 / sensor:5 / camera_drive:5 / camera_signal:5 | 画像パネルごとの最大更新頻度。|
| `commands.cooldown_ms.*` | int | 500 | manual_start / sig_recog / road_blocked の連打抑止時間。|
| `commands.override_timer_hz.obstacle_hint` | double | 0.5 | 障害物ヒント固定送出の周期。|
| `topics.camera.drive` / `topics.camera.signal` | string | `/perception/road_blockage/decision_image` / `/perception/traffic_signal/decision_image` | 外部カメラの購読トピック名。|
| `topics.road_blocked.external_priority` | bool | true | 外部 `/road_blocked` を GUI 送信より優先するか。|

## ROS インタフェース
### Publisher
| トピック | 型 | 用途 |
|----------|----|------|
| `/manual_start` | `std_msgs/msg/Bool` | 手動開始フラグ（Transient Local）。|
| `/sig_recog` | `std_msgs/msg/Int32` | 信号認識結果 1=GO / 2=STOP。|
| `/obstacle_avoidance_hint` | `tc_route_msgs/msg/ObstacleAvoidanceHint` | GUI 指定の固定回避指示。停止時はゼロ値を送信。|
| `/road_blocked` | `std_msgs/msg/Bool` | 道路封鎖通知（外部入力と競合した場合は外部を優先）。|

### Subscription（抜粋）
| トピック | 型 | 表示場所 |
|----------|----|----------|
| `/route_state` / `/manager_status` | `tc_route_msgs/msg/RouteState` / `tc_route_msgs/msg/ManagerStatus` | ルート進捗カード、再計画履歴。|
| `/follower_state` | `tc_route_msgs/msg/FollowerState` | フォロワ状態カード、イベントログ。|
| `/active_route` | `tc_route_msgs/msg/Route` | ルート地図画像とウェイポイント一覧。|
| `/sensor_viewer` | `sensor_msgs/msg/Image` | 障害物ビュー画像パネル。|
| `/perception/road_blockage/decision_image` / `/perception/traffic_signal/decision_image` | `sensor_msgs/msg/Image` | 走行カメラ・信号監視パネル。|
| `/active_target` / `/localization/pose_enu` | `geometry_msgs/msg/PoseStamped` / `PoseWithCovarianceStamped` | 目標距離計算。|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度カード。|
| `/manual_start` / `/sig_recog` / `/road_blocked` | `std_msgs/msg/Bool` / `Int32` / `Bool` | イベントバナー、タブ表示の現在値。|

## ノード起動管理
- プロファイル定義は `config/node_launch_profiles.yaml`（必要に応じて作成）にまとめられ、起動対象・既定パラメータ・シミュレータ有無を記述します。
- GUI で選択した YAML は `ros2 launch <package> <file> param_file:=<path>` として渡され、`NodeLaunchManager` が `SIGINT → SIGTERM → SIGKILL` の順に安全に停止処理を行います。
- `Drive Mode Manager` カードでは `start_gui` に加えて `joy_input` を指定できます。既定は `joy_node` で、開発用入力源にする場合は `ps3_joy_sim` を指定します。
- `Road Blockage Detector` は既定で NCNN 版 YOLO を使い、カードの「PyTorch版YOLOを使用」を
  有効にした場合のみ PyTorch 版 launch へ切り替えます。
- `Traffic Signal Recognizer` は信号用 PyTorch モデルのみを起動候補とし、NCNN 版への切替は
  提供しません。
- ログはノード単位でリングバッファへ収集され、Console Logs タブから確認できます。

## 運用上のヒント
- `road_blocked` の値は外部ノードからの購読値が優先されるため、GUI で送信後に値が戻る場合は外部ノードが上書きしています。必要に応じて `topics.road_blocked.external_priority=false` に変更してください。
- 障害物ヒントの固定送出は 0.5Hz で継続送信します。現場復帰時は「送出停止」ボタンを押してゼロ値を送信し、`obstacle_monitor` に制御を戻してください。
- 「全起動」は `launch_priority` の昇順で処理し、途中で失敗した場合は残りのノードを停止状態で維持します。ログタブでエラーメッセージを確認のうえ再試行してください。

## 開発・テスト
- GUI なしでロジックを確認したい場合は `python3 -m robot_console.gui_core` でユニットテスト用メインを実行できます（PyYAML / Pillow / OpenCV が未導入でもフォールバック動作）。
- モック画面は `python3 tools/mock_ui.py` で起動し、ROS 環境なしに画面レイアウトと操作フローを確認できます。
- `tools/tests/` 配下に pytest ベースのテストを収録しています。`pytest tools/tests` を実行してロジックの回帰を検出してください。
- ワークスペースの `requirements.txt` で `pytest-forked` も導入してください。
  QtWebEngine の状態をテスト間で共有しないよう各テストを別プロセスで実行します。
  未導入の場合は `pytest.ini` の必須プラグイン検査で実行前にエラーになります。
- テストは既定で `QT_QPA_PLATFORM=offscreen` と
  `QT_QUICK_BACKEND=software` を設定するため、ディスプレイのないCIでも地図タブの
  切り替えを検証できます。これらの設定はテストにのみ適用します。
- `tools/headless_route_stack_eval.py` は tkinter 画面を生成せず、`GuiCore` に
  GUI 操作相当の入力を与えて route stack の簡易回帰評価を行う補助ツールです。
  `route_planner`、`route_manager`、`route_follower`、`drive_mode_manager`、
  `robot_navigator`、`robot_simulator` を起動し、`/route_state`、`/active_route`、
  `/follower_state`、`/cmd_vel`、`/cmd_vel/autonomous`、`/drive_mode_status`、
  `/manual_start` を監視します。

  ```bash
  source install/setup.bash
  run_id=$(date +%Y%m%d_%H%M%S)
  mkdir -p "log/codex/${run_id}/ros" "log/codex/${run_id}/robot_console"
  export ROS_LOG_DIR="$PWD/log/codex/${run_id}/ros"
  python3 src/robot_console/tools/headless_route_stack_eval.py \
    --start-label 10 \
    --goal-label 30 \
    --console-log-directory "log/codex/${run_id}/robot_console"
  ```

  既定では `route_planner` / `route_manager` に `tsukuba.yaml`、`route_follower` /
  `drive_mode_manager` は `start_gui=false` で `joy_node`、manual teleop、mux を同時起動し、`robot_navigator` は
  `cmd_vel_topic=/cmd_vel/autonomous` で起動し、`robot_navigator` の simulator を有効にします。
  評価終了時は `GuiCore.request_stop_all()` 相当の停止処理を行い、各 profile の
  停止状態を出力します。GUI あり評価で専用状態 GUI も起動する場合は
  `tools/gui_route_stack_eval.py --show-drive-status-gui` を指定します。異なる範囲を評価する場合は `--start-label`、`--goal-label`、
  `--timeout-sec`、`--post-goal-wait-sec`、`--no-simulator` などを指定してください。
- `tools/gui_route_stack_eval.py` は `UiMain` を実際に生成し、座標クリックではなく
  automation hook 経由で Combobox、Entry、Checkbutton、Button 相当の操作を行います。
  ローカルデスクトップまたは X11 転送ありの環境で実行してください。

  ```bash
  source install/setup.bash
  run_id=$(date +%Y%m%d_%H%M%S)
  mkdir -p "log/codex/${run_id}/ros" "log/codex/${run_id}/robot_console"
  export ROS_LOG_DIR="$PWD/log/codex/${run_id}/ros"
  python3 src/robot_console/tools/gui_route_stack_eval.py \
    --start-label 10 \
    --goal-label 30 \
    --console-log-directory "log/codex/${run_id}/robot_console" \
    --verify-log-open-buttons
  ```

### 評価ツール実行時のログ

Codex が評価ツールを実行する場合は、ワークスペース直下の `log/codex/` 配下に
実行ごとのログディレクトリを作成し、`ROS_LOG_DIR` を設定してから実行する運用を
基本にします。

```bash
run_id=$(date +%Y%m%d_%H%M%S)
mkdir -p "log/codex/${run_id}/ros" "log/codex/${run_id}/robot_console"
export ROS_LOG_DIR="$PWD/log/codex/${run_id}/ros"
```

`ros2 launch robot_console robot_console.launch.py` 経由で起動した場合は、`ROS_LOG_DIR` が
`console_log_directory` として `RobotConsoleNode` に渡され、`NodeLaunchManager` が
各 profile の stdout/stderr をその配下に保存します。評価ツールを Python から
直接実行する場合は、`--console-log-directory` に
`log/codex/<run_id>/robot_console` を指定してください。この場合も、
`NodeLaunchManager` が各 profile の stdout/stderr をその配下へ保存します。
保存済み ROS ログは `ROS_LOG_DIR` 配下を参照してください。

## 依存パッケージ
- GUI 機能：`tkinter`（標準ライブラリ）、`Pillow`（画像描画）、`opencv-python`（画像デコード）。Pillow / OpenCV は未導入でも縮退動作します。
- ROS2 メッセージ：`tc_route_msgs`、`geometry_msgs`、`sensor_msgs`、`std_msgs`。

以上の内容を参考に、運用開始前に `config/node_launch_profiles.yaml` や `config/robot_console.yaml`（必要に応じて作成）を実際の環境に合わせて整備してください。


## GNSS/LIO融合とデジタルツイン

正式launchはPyQt5 UIを起動する。起動設定の「デジタルツイン／自律走行」と
「実機（融合）／自律走行」はicart_bringupの共通構成を使う。
生成したsession.yamlをi-Cart融合profileで選択する。旧の個別profileと同時起動しない。
GPS/PoseカードはGNSS北CW方位、融合map CCW yaw、推定方位σ、適応baselineを区別して表示する。
実機private RTK topicと模擬公開topicの両方に対応するが、両環境は別DDS domainで実行する。

[共通起動・環境切替](../icart_bringup/docs/共通起動設計.md)と
[方位の実装評価](../gnss_lio_fusion/docs/方位実装評価.md)を参照する。

## 正式UIの不足依存とローカル展開

PyQt5正式入口はQt WebEngineを必要とする。通常は既存package.xmlに従い、
`python3-pyqt5.qtwebengine`をaptで導入する。sudo認証が使えず、Ubuntu 24.04の
基本PyQt5は導入済みの場合、次の補助手順で不足依存をローカル展開できる。

```bash
python3 src/robot_console/tools/prepare_ui_runtime.py --output log/codex/ui_local
source src/robot_console/tools/activate_ui_runtime.bash log/codex/ui_local/runtime
```

補助ツールはaptの不足依存をサイズ・checksum照合して展開し、aptデータベースを変更しない。
Qt 5はWebEngineの資源パスを環境変数だけで移設できないため、qt.confとQt resourceを生成する。
`ROBOT_CONSOLE_QT_RESOURCE`は本UIだけで読み込み、通常のシステム導入時には不要である。
[Qtのqt.conf探索仕様](https://doc.qt.io/qt-6.10/qt-conf.html)に沿い、この環境のQt 5で動作確認した。
生成物はGitに追加しない。異なる場所へ移動した場合は補助ツールを再実行する。

ヘッドレス検証には`--headless-tests`を付けて、Xvfbとpytest-forkedも展開できる。
既存conftestのfork隔離を有効にして実行する。正式入口を模擬ROS graph（domain=86）へ
接続し、4タブと実受信の地図・方位を保存する試験はtools/check_qt_entry.pyである。
Xvfbの非公開仮想画面を使用し、実機への操作指令は送らない。

CLI同時起動時は`--business-environment`で初期環境を指定できる。
後からUIを起動してmanual_start履歴を取り逃した場合でも、鮮度OKのfollower RUNNING/AVOIDINGを
運行フェーズ表示に反映する。コマンド値を推測して再配信することはしない。

### PC側UIからROSBAG保存

ダッシュボードの「ROSBAG保存」で保存先を選び、記録開始 → 停止・保存を操作する。
既定はPC内 `~/rosbags` に日時別保存。全通常トピックを記録し、経過時間・容量・保存先を表示する。
Web UIには追加していない。詳細は [ROSBAG保存](docs/rosbag_ui.md) を参照。
