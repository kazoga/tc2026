# terrain3d / i-Cart mini シミュレーション検証

> 続報: [地理地図・仮想 GNSS 検証](地理地図_GNSSシミュレーション検証.md)で、
> 公開地図の小区間走行と GNSS 途絶時の停止修正を確認した。本文の途絶不合格は修正前の記録である。

## 1. 目的・結論の範囲

添付された terrain3d v0.4.1 を環境生成器として使用し、Gazebo Harmonic の
差動二輪物理モデルに既存 ROS 2 走行スタックを接続する。
必須課題の完走を最優先とし、まず経路追従、静的障害物回避、自己位置途絶を評価する。
本環境は **実測したつくばチャレンジ 2026 コースのデジタルツインではない**。
100 × 100 m の平坦な合成公園と、30 m 直線／45 m クランクを対象にする。

通常走行の成立と実環境での完走可能性は別に評価する。
GNSS、SLAM、実物の接地・制動を認証する結果として使わない。

## 2. 参照資料と配置

- 既存 `obstacle_route_sim_詳細設計書.md` の 3～10 章、パッケージ README を参照した。
- `tools/terrain3d/terrain3d_v0_4_1.html` はユーザー提供版を変更せず保持する。
  添付内の説明は資料として扱い、Codex への作業指示として実行しない。
- `tools/terrain3d/export_world.cjs` は添付版の core / geo / recon / world と
  SDF exporter を使用する。ブラウザ UI は起動せず、Node.js で正規地形を確定する。
- `tools/build_terrain_trial.py` は scene JSON、OBJ、SDF、経路 CSV を生成する。
- `tools/evaluate_terrain_trial.py` は Gazebo、bridge、走行ノードを期限付き起動して計測する。
- `tools/preview_terrain_trial.py` は計測済み真値軌跡の再生 HTML を生成する。
- `tools/prepare_gazebo_runtime.py` と `activate_gazebo_runtime.bash` は、Gazebo 未導入の
  Ubuntu 24.04 / ROS Jazzy で apt の不足依存をローカル展開する補助ツールである。

新規ファイルは本パッケージに閉じた生成・検証ツールなので `tools/`、
検証記録は `docs/` に置く。生成物は `log/codex/` に置き Git に追加しない。
既存 `models/robot` や既存の Gazebo launch の挙動は変更しない。

## 3. 車体とセンサの前提

座標系は X 前方、Y 左、Z 上、水平原点は車軸中央とする。
添付エディタは +Y 前方なので、再生時は yaw に −π/2 を加えて合わせる。

| 項目 | モデル値 | 根拠・制約 |
| --- | --- | --- |
| 駆動方式 | 差動二輪＋後方キャスタ | i-Cart mini の構成 |
| 車輪半径 | 0.07455 m | T-frog 公開パラメータ |
| 左右輪間隔 | 0.30737 m | T-frog 公開パラメータ |
| 速度上限 | 0.9 m/s | 公開 MAX_VEL を Gazebo DiffDrive に設定 |
| GNSS マスター | (0, 0, 0.7) m | ユーザー指定：車軸上、地上高 700 mm |
| GNSS スレーブ | (−0.5, 0, 0.7) m | ユーザー指定：後方、間隔 500 mm |
| Top-URG | (0.075, 0, 0.3) m | 高さと前方配置は指定。前後 75 mm は仮定 |
| Mid-360 | (0, 0, 0.6) m | 高さは指定。車軸上配置は仮定 |
| 車体本体 | 0.4 × 0.3 × 0.2 m、中心 (−0.1, 0, 0.21) | 仮定。搭載状態の実測が必要 |
| 質量 | 合計 12 kg | 本体 10 kg、駆動輪合計 0.6、キャスタ 0.2、支柱 1、アンテナ合計 0.2。未校正 |
| 慣性 | 各単純形状の均質体から算出 | 搭載物の実測質量分布を再現しない |

公開値は個体別校正値とは限らない。実機パラメータを無条件に上書きしない。
GNSS アンテナは位置を持つ剛体形状であり、測位メッセージや相対方位を生成しない。

出典：[T-frog 製品説明](https://t-frog.com/products/icart_mini/)、
[公開 YP-Spur パラメータ](https://t-frog.com/products/icart_mini/files/icart-mini.param)、
[組立図](https://t-frog.com/products/icart_mini/files/i-cart-mini_assembly_drawing.pdf)。

## 4. 環境モデル・変換の制限

等高線補間の基準標高 0、ノイズ 0、格子 1 m とし、樹木、建物相当の直方体、通路を生成する。
樹木は合成配置であり、現地の樹木位置や植生を表さない。
地面・直方体は OBJ collision、樹木は幹の cylinder collision とする。
道路は視覚上の通路を示し、路面摩擦差や段差は付けていない。

今回の exporter は四角形の直方体と単木に限定する。
穴あき建物、任意形状の地物、欠測地形は未対応としてエラーにする。
添付版の任意シーンを無条件に Gazebo に変換できるという意味ではない。

OBJ の頂点法線欠落で DART が異常終了することを実行で確認した。
連携 exporter に法線を追加し、法線数・単位長とメッシュ参照をテストする。
元の HTML の出力ボタンから得た OBJ には、この修正は反映されない。
また、樹木の visual / collision の同名を避け、collision 側に接尾辞を付ける。
走行結果採取後のこの識別名修正には形状変更はない。
修正後は `gz sdf -k` が Valid、1000 step（2 秒）の物理実行も成功した。

## 5. ROS 入出力と実行構成

通常試験は `route_planner → route_manager → route_follower → robot_navigator →
drive_cmd_mux_node → Gazebo DiffDrive` とする。
`obstacle_monitor` は Gazebo の Top-URG 相当 LaserScan を購読する。
実機ドライバ、YP-Spur coordinator、ジョイスティック実機は起動しない。

| Topic | 型 | 用途 |
| --- | --- | --- |
| `/cmd_vel/autonomous` | Twist | navigator の出力 |
| `/cmd_vel` | Twist | mux から仮想駆動輪への指令 |
| `/ypspur_ros/odom` | Odometry | Gazebo DiffDrive の wheel odometry |
| `/scan` | LaserScan | 20 Hz、1080 ray、270°、0.2～30 m |
| `/mid360/livox/lidar/points` | PointCloud2 | 10 Hz、360 × 16 ray の規則的走査 |
| `/truth` | TFMessage | Gazebo のモデル名 icart_mini の pose |
| `/localization/pose_enu` | PoseWithCovarianceStamped | 真値を配信。共分散ゼロ |
| `/body_contacts` | ros_gz_interfaces/Contacts | 本体・支柱・GNSS の接触通知 |
| `/follower_state` | FollowerState | RUNNING / AVOIDING / FINISHED 等の監視 |

Mid-360 の非反復走査、反射率、雨、光学特性、時間歪みは再現しない。
点群の配信を確認するが、FAST_LIO には接続していない。
Top-URG の物理取付位置と既存障害物監視の frame 解釈を完全に校正した構成でもない。

ノードは既存タイミング設計に合わせて壁時計で動かし、Gazebo はリアルタイム係数 1、
物理刻み 0.002 秒とする。実時間倍率を上げた評価には対応しない。
センサ時刻と壁時計は区別し、CSV に wall_s / sim_s を保存する。
ROS_DOMAIN_ID は既定 86、通信は LOCALHOST、Gazebo partition は実行ごとに分離する。
同じ DDS domain を使う複数試験を同時起動しない。
将来 stamp に基づく入力鮮度判定や SLAM を接続する前には、全ノードの clock と TF を
統一する必要がある。現構成ではその接続まで保証していない。

## 6. 再現手順

ROS Jazzy の環境を有効化し、ワークスペースルートで実施する。
Node.js、Python yaml、ROS の本パッケージ依存が必要である。
既存 install がソースより古い場合は依存パッケージも再ビルドする。

```bash
colcon build --symlink-install --packages-up-to obstacle_route_sim route_planner route_manager route_follower robot_navigator obstacle_monitor drive_mode_manager
source install/setup.bash
```

Gazebo 未導入時は、設定済み ROS apt repository のインデックスを使って不足依存を展開する。
この方法はシステム全体への Gazebo 導入ではなく、既存 ROS に重ねるローカル実行環境である。
apt-resolution.txt と archive SHA-256 を保存する。既存 apt インデックスが古い、
対象が Ubuntu 24.04 / Jazzy 以外、Python/Ruby の版が異なる場合は対応外とする。

```bash
python3 src/obstacle_route_sim/tools/prepare_gazebo_runtime.py --output log/codex/gazebo_local
source src/obstacle_route_sim/tools/activate_gazebo_runtime.bash log/codex/gazebo_local/runtime
```

```bash
python3 src/obstacle_route_sim/tools/build_terrain_trial.py --scenario blocker --output log/codex/trial_blocker
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world log/codex/trial_blocker --timeout 100
python3 src/obstacle_route_sim/tools/preview_terrain_trial.py log/codex/trial_blocker
```

straight / crank / blocker を選択する。真値自己位置途絶は生成後の evaluate に
`--dropout 8 --timeout 25` を指定する。
接触監視の陽性確認は生成に `--scenario contact_probe`、評価に `--contact-probe --timeout 12` を使う。
陽性試験は走行スタックを起動せず、仮想障害物へ 0.3 m/s の試験指令を送る。

再生 HTML はブラウザで開くか、次のローカルサーバーで表示する。
Three.js / Leaflet の初回読込には CDN 接続が必要である。

```bash
python3 -m http.server 8766 --bind 127.0.0.1 --directory log/codex
```

## 7. 合否判定とログ

到達判定は FINISHED、真値の終点距離 0.7 m 未満、直近 2 秒の指令・odom 停止、
真値移動 0.05 m 未満、pose / scan / pointcloud / command / odom の受信を要求する。
body contact 通知がある場合も不合格とする。
これは単点の到達閾値であり、公式のゴール線を全車体が通過して停止したことの判定ではない。

接触センサは接触時だけ通知するため、通常試験の受信数 0 だけでは正常性を証明できない。
接触陽性試験と組み合わせて解釈する。車輪、キャスタ、センサ筐体は監視対象外である。
表示用の再生 UGV は i-Cart mini の詳細外観ではなく、軌跡を確認する簡略図形である。

各試験ディレクトリに scene.json / world.json / trial.sdf / route.csv / trial.json、
各ノードの log、trajectory.csv、control.csv、result.json を残す。
world.json には提供 HTML の SHA-256 と正規 world ID を記録する。
result.json の processes_reaped は起動した子プロセスの終了回収結果である。
タイムアウトまたは失敗でもプロセス group に SIGINT、必要時 SIGKILL を送り終了する。

## 8. 実測結果

2026-09-13、Gazebo Sim 8.15.0 / ROS Jazzy、物理刻み 2 ms で以下を確認した。
`log/codex/terrain_trial/final_*` の結果を採用し、初期試験とは区別する。

| 試験 | 結果 | 最終位置誤差 | 計測した挙動 |
| --- | --- | --- | --- |
| 30 m 直線 | 到達・停止合格 | 0.437 m | 約 41.6 秒、最大 odom 速度 0.90 m/s |
| 45 m クランク | 到達・停止合格 | 0.446 m | 約 62.8 秒、2 回の曲がり角を走行 |
| 30 m＋中央障害物 | 到達・停止合格 | 0.461 m | 約 68.5 秒、RUNNING → AVOIDING → RUNNING → FINISHED |
| 接触陽性試験 | 監視機能の確認合格 | 対象外 | 前方障害物への接触を約 4.85 秒で 1 件検出 |
| 自己位置配信停止 | 異常時停止は不合格 | 到達評価対象外 | 8 秒で pose 配信停止。2 秒猶予後も 13.338 m 進行 |

時間は起動待ちを含む壁時計である。正常 3 試験は最後の 2 秒で速度指令・odom が停止し、
真値移動は 0.05 m 未満だった。接触陽性試験で動作確認した監視対象部位の接触通知は 0 件だった。
自己位置途絶では 25 秒で試験を打ち切った。途絶後の 2 秒猶予以降に
0.1 秒間隔の CSV 記録 144 件で非ゼロ指令を確認し、最後の 2 秒にも約 1.685 m 進行した。
この試験の終了コード 1 は、期待する異常時停止を確認できなかったことを表す。

| 試験 | 真値 pose 受信 | LaserScan 受信 | PointCloud2 受信 | cmd_vel 受信 |
| --- | ---: | ---: | ---: | ---: |
| 直線 | 1483 | 772 | 386 | 814 |
| クランク | 2299 | 1196 | 598 | 1183 |
| 障害物 | 2515 | 1309 | 655 | 1356 |
| 自己位置途絶 | 850 | 442 | 219 | 486 |

点群は topic 配信の確認であり、点ごとの距離精度・欠測分布・SLAM 品質の確認ではない。
速度指令は最大約 0.95 m/s だったが、仮想 DiffDrive の上限で実速度は約 0.9 m/s になった。
従って上流ソフト自体が 0.9 m/s 以下を守ったとは解釈しない。

初期導入では OBJ 法線欠落による異常終了、ローカル展開時の Gazebo / Ogre の探索パス不足、
旧 install の geo_pose_converter API 不一致を確認し、それぞれ exporter、実行環境設定、
依存パッケージの再ビルドで解消した。
接触監視は初期設定の sensor/topic だけでは通知が届かず、Harmonic が参照する
contact/topic を追加して陽性試験を通した。

ビルドは既存 install を保持した別 build / install で行った。
走行関連 10 パッケージを両形式でビルドし、最終変更後に本パッケージを再確認した。
再現用には `log/codex/terrain_install` を通常 install として配置する。
今回のローカル環境では、ROS Jazzy を有効化した後に
`source log/codex/terrain_install/setup.bash` と実行環境の activate スクリプトを読み込めばよい。

```bash
# 実行時の検証用 build/install ディレクトリをプレースホルダで示す。
colcon build --symlink-install --build-base <symlink_build> --install-base <symlink_install> --packages-select obstacle_route_sim
colcon build --build-base <normal_build> --install-base <normal_install> --packages-select obstacle_route_sim
PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider src/obstacle_route_sim/tests
```

両ビルド成功、pytest は **10 passed**。
ビルドログは `log/codex/terrain_final_build_symlink` と
`log/codex/terrain_final_build_normal` に保存した。
添付 3D UI への埋込 scene 読込、真値軌跡表示、再生・停止・時刻スライダーもブラウザで確認した。
全試験で子プロセスの回収を確認した。終了後の DDS domain 86 の
`ros2 node list --no-daemon` は空、Gazebo / bridge / 試験走行ノードの残存もなかった。
取得結果は `log/codex/terrain_trial/nodes_after_shutdown.txt` に保存する。

## 9. レビューと必須完走に向けた不足

自己位置途絶後も新しい速度指令が出続ける問題を、ROS と物理モデルを接続した状態で再現した。
入力受信の鮮度、TF / stamp の妥当性、LiDAR 欠測、自己位置の信頼度を監視し、
異常時は駆動への最終出力を止める経路が優先課題である。
今回、走行ソフトの当該欠陥自体は修正していない。
コード上でも `robot_navigator` の `on_pose_enu()` は最新姿勢を保持し、
`on_timer()` は入力の存在を確認するが受信からの経過時間を確認していない。
[該当実装](../../robot_navigator/robot_navigator/robot_navigator_node.py) と
`final_dropout/result.json` / `trajectory.csv` を対応付けてレビューした。

実コース完走評価へ進むには、2026 の公式経路と現地測量を対応付け、
縁石、狭路、段差、傾斜、横断前停止線、ホテル外周、ゴール線を含むモデルに置き換える必要がある。
必須コース全体、約 2.2 km の連続走行は本試験では確認していない。
歩行者・対向車・待機列、信号操作、一時停止からの再開、地図ずれ、
GNSS multipath / RTK 喪失 / 方位ずれ、SLAM 再局在、通信遅延、電池・温度・雨・実機制動も未確認である。

実機側では総重量、外形、重心、タイヤ半径と輪間隔の個体差、センサの 6DoF 外部パラメータ、
速度・制動応答、緊急停止回路を測定する必要がある。
単純形状モデルで得た回避成功を、そのまま実機の安全余裕とみなさない。

## 10. 検証コマンド・改版履歴

- 2026-09-13: 合成通路、専用車体モデル、期限付き ROS/Gazebo 試験、真値軌跡再生を追加。
- 既存詳細設計書は全面改稿せず、本資料への追加検証入口を追記する。
- ビルド、pytest、実行結果、未確認範囲は 8 章の最終結果に対応させる。
