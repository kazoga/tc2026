# 地理地図・仮想 GNSS による走行検証（2026-09-13）

## 1. 対象と結論

提供された terrain3d v0.4.1 を基盤として、つくば市役所付近の 130 m 四方を
国土地理院 DEM5A と OpenStreetMap の建物 4 棟から再構成した。
既存経路 `src/route_planner/routes/tsukuba/fixed/fixed_first.csv` の先頭 12 点を用い、
Gazebo の物理運動から仮想 GNSS を生成し、既存 ROS 走行ノードで終点到達・停止を確認した。

これは公開地図による部分的なデジタルツインである。2026 年の全コース・現在の現物との一致、
段差通過、群衆対応、GNSS 品質低下時の自己位置切替を確認したものではない。
元の HTML は変更しない。原本内の記述は資料としてのみ扱う。

## 2. 構成と配置

- `tools/build_geographic_trial.py`: 標高タイルと OSM 原本を保存し、提供版の
  `TerrainGeo.osmToFeatures`、`TerrainWorld.compile` へ接続する。
  取得失敗・欠測標高を架空の平坦地で埋めず、処理を停止する。
- `tools/terrain3d/export_world.cjs`: 地理地図では提供版の `buildingTriangles`、
  `importedMeshTriangles`、`worldMeshes` を使用する。凹形状・中庭も同じ三角形分割を使う。
  Three.js は元の HTML と同じ 0.128.0 を `package.json` に固定する。
- `tools/gnss_simulator_node.py`: 仮想 GNSS を生成する ROS ノード。
- `tools/evaluate_terrain_trial.py`: 隔離 DDS domain と Gazebo partition で期限付き実行する。
- `tools/preview_terrain_trial.py`: 計測済み真値軌跡を提供版 HTML へ埋め込む。

補助実行ツールはパッケージ内 `tools/`、本記録はパッケージ内 `docs/` に置く。
取得原本・地図・計測ログは `log/codex/digital_twin/` に生成し、Git 管理対象にしない。

```text
terrain3d 地理データ → OBJ/SDF → Gazebo i-Cart mini の物理運動
  → /truth → 仮想 GNSS → /rtk_gps/{fix,heading,rtk_status}
  → geo_pose_converter → /localization/pose_enu
  → route_follower / robot_navigator → drive_cmd_mux → /cmd_vel → Gazebo
```

真値自己位置の直接配信は `--gnss` 指定時に無効とする。
LiDAR・odom・接触は従来と同じ Gazebo bridge を経由する。
実機 driver、ypspur coordinator、実センサは起動しない。

## 3. 地図の根拠・精度

標高は国土地理院の DEM5A PNG タイルをデコードする。
1 m 出力格子は DEM5 を細かく再標本化したものであり、1 m 測量精度を意味しない。
現在の取得処理は最近傍標本化であり、タイル解像度に由来する段状の高さ変化があり得る。
出発点の標高 25.92 m を差し引き、Gazebo の相対 z とする。
建物外周は OSM、未記載の高さ・屋根は提供版の推定値を使う。
地表テクスチャは提供版の手続き生成であり、航空写真ではない。
道路・縁石・植栽・標識の網羅取得は今回の小区間生成ツールには実装していない。

水平方向は提供版の半径 6,371,000 m の球近似による東北座標である。
GNSS はそのローカル座標を WGS84 ENU として逆投影するため、地図と地理座標の厳密な
整合には球近似と楕円体の差の補正が必要である。全コースへ広げる前に是正する。
GNSS 原点の楕円体高 67.078 m は近傍の公開点群原点からの仮置きで、
今回の出発地点の測定値ではない。絶対高度やジオイド補正を評価していない。

2025 年 fuRo 点群は公式一覧で確認したが、HTTP/HTTPS の取得先が 404 で、
今回の幾何形状には使用できなかった。2026 年コースの約 2.2 km 全体を再現したとは扱わない。

出典:
[国土地理院標高タイル](https://maps.gsi.go.jp/development/demtile.html)、
[地理院タイル一覧](https://maps.gsi.go.jp/development/ichiran.html)、
[OpenStreetMap contributors / ODbL](https://www.openstreetmap.org/copyright)、
[つくばチャレンジ公開データ一覧](https://github.com/tsukubachallenge/tc-datasets)、
[2026 課題](https://tsukubachallenge.jp/2026/regulations/tasks)。
各取得 URL と SHA256 は生成した `trial.json` に記録する。

## 4. GNSS 模擬仕様

| 項目 | 設定・挙動 |
|---|---|
| マスター | 車軸中心から (0, 0, 0.70) m |
| スレーブ | 車軸中心から (-0.50, 0, 0.70) m |
| 姿勢 | 真値 quaternion の roll/pitch/yaw でアンテナ位置を回転 |
| 出力 | `/rtk_gps/fix`、`/rtk_gps/slave_fix`: NavSatFix |
| 方位 | `/rtk_gps/heading`: Imu、`/rtk_gps/rtk_status`: RtkStatus |
| 既定周期 | 10 Hz、配信キュー確認 100 Hz、reliable depth 10 |
| 誤差 | 水平各軸 σ=0.02 m、鉛直 σ=0.04 m、方位 σ=0.5° の独立 Gaussian |
| アンテナ間 | 位置誤差は共通。方位誤差は別の試験モデルで加える |
| 遅延 | 0.1 s、元の計測 stamp を保持 |
| 乱数 | seed=42。サンプル列を固定し、Gazebo の実時間スケジュールまでは固定しない |
| 真値欠落 | 0.5 秒以上でキューを消去し、古い真値を新しい測位として再送しない |
| 途絶 | dropout_sec、dropout_duration_sec。0 秒 duration は永久途絶 |
| 品質 | RTK_FIX を仮定。FLOAT、衛星配置、遮蔽、マルチパス、補正通信は未モデル化 |

緯度経度高度変換は `geo_pose_converter.geo_core.enu_to_llh` を再利用する。
方位はスレーブ→マスター方向を北基準時計回りに変換する。
Imu の角速度・加速度は観測していないため covariance[0]=-1 とする。
NavSatFix の共分散を配信し、RtkStatus の raw state に `SIM_RTK_FIX` を入れる。

原点・σ・方位σ・遅延・周期・seed・途絶条件はノード起動パラメータで設定する。
評価 CLI では `--gnss`、`--gnss-sigma`、`--dropout` を提供する。
GNSS と評価器の途絶起点は同じ monotonic 時刻へ合わせる。
タイマーは実時間、stamp は Gazebo 時刻であり `/clock` 同期・早回しは未検証である。

## 5. 接続で発見・修正した問題

`geo_pose_converter` が `NavSatFix.position_covariance` を bool 判定すると、
NumPy 配列の truth value が曖昧なため例外終了していた。
要素数 9 の判定に修正し、実際の ROS message の配列を使う回帰テストを追加した。

`robot_navigator` は自己位置または odom の最終受信から 1.0 秒でゼロ指令を出す。
`input_watchdog_core.py` に監視を分離し、タイムアウト境界・未受信・復帰をテストした。
全入力復帰後は既存目標への追従を再開する。stamp・品質・LiDAR・ノード死活は監視しない。

## 6. 実行結果

実測値は `log/codex/digital_twin/<scenario>/result.json` と `trajectory.csv` に保存する。

| 試験 | 結果 | 終点誤差 | 概要 |
|---|---|---:|---|
| gnss_straight | 到達・停止合格 | 0.313 m | 合成 30 m 通路、仮想 GNSS 経由 |
| cityhall | 到達・停止合格 | 0.239 m | 現地 130 m 四方・既存経路 12 点（42.68 m）、57.78 秒で評価終了 |
| gnss_dropout | 異常時停止を確認 | 到達評価対象外 | 8 秒で GNSS 停止、2 秒猶予後の指令ゼロ・車体停止 |

cityhall の測位結果 551 件、真値 2118 件、LaserScan 1102 件、Mid-360 模擬点群 551 件を受信した。
同時刻付近の最新真値との差による水平 RMSE は 0.0915 m である。
これは遅延中の車体移動も含む値で、静止受信機の測位精度ではない。
終点で最後 2 秒の移動は約 1.3e-8 m、監視対象の車体接触は 0 件であった。
接触監視の陽性対照は前回の合成障害物試験で実施済みである。

GNSS 途絶の 2 秒猶予後の真値移動は 7.4e-9 m、非ゼロ指令は 0 件であった。
関連 pytest は 43 件合格。通常・symlink 両ビルドは成功した。
ROS/Gazebo は全子プロセスを回収し、終了後の専用 domain の残存を確認する。

## 7. 再現手順

ROS Jazzy と Gazebo / ros_gz を用意し、ワークスペース overlay を source する。
ローカル展開版 Gazebo を使う場合は既存の terrain3d 検証記録の手順に従う。

```bash
npm install --prefix log/codex/digital_twin/node_runtime --ignore-scripts --no-audit --no-fund three@0.128.0
export NODE_PATH="$PWD/log/codex/digital_twin/node_runtime/node_modules"
python3 src/obstacle_route_sim/tools/build_geographic_trial.py \
  --output log/codex/digital_twin/cityhall \
  --route src/route_planner/routes/tsukuba/fixed/fixed_first.csv --count 12
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py \
  --world log/codex/digital_twin/cityhall --gnss --timeout 90
python3 src/obstacle_route_sim/tools/preview_terrain_trial.py log/codex/digital_twin/cityhall
python3 -m pytest -q src/robot_navigator/tests src/obstacle_route_sim/tests src/geo_pose_converter/tests
colcon build --symlink-install --packages-up-to obstacle_route_sim
# 形式切替は別 build/install 基点を指定するか、AGENTS.md の生成物クリーン規定に従う。
colcon build --packages-up-to obstacle_route_sim
```

ブラウザの再生は記録の確認用で、ライブ速度指令を送る画面ではない。
元ツールで編集した内容は scene JSON に保存し、SDF を再生成してから再試験する必要がある。

## 8. 残る不足と次の受け入れ条件

1. 2026 年の全経路確定と、現地 LiDAR 点群・路面・縁石・植栽による形状検証。
2. 地図を WGS84 ENU へ厳密に統一し、楕円体高・標高・アンテナ lever arm を校正。
3. 実機 rosbag による GNSS 時間相関・FLOAT・遮蔽・マルチパス・方位欠測モデル。
4. 実際の localization 切替・融合ノードを接続。今回の GNSS pose の直接 remap は
   SLAM/GNSS 切替の評価を省略している。
5. 搭載状態の重量・重心・外形・摩擦・キャスタ・車輪滑り・制動距離の校正。
6. Mid-360 の非反復走査・点時刻・反射・モーション歪みと、実際の FAST-LIO の接続。
7. 動的歩行者、横断・停止線、狭路、雨や逆光、センサ/ROSプロセス故障の試験。

これらを満たすまで、今回の合格を実機の安全性・必須課題完走の保証には使わない。

## 9. 改版履歴

2026-09-13: terrain3d による地理地図再構成、仮想 GNSS、自己位置途絶停止、配列判定修正を記録。
