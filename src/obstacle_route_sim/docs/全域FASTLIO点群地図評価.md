# 全域 FAST-LIO 点群地図評価

## 1. 目的と比較対象

本書の基準走行に対し、conservative・seed=1でも全周試験を実施した。
GNSS制御は完走したが、2D剛体整合後もXY RMSE 16.96 mが残った。
条件・比較表・限界は[センサ誤差モデル評価8章](FASTLIOセンサ誤差モデル評価.md#8-conservative-条件の全周評価)を参照する。

調整済み tc2026_corridor_1m の全 1,142 waypoint、概略 2.224 km を連続走行し、
LiDAR＋IMU から推定した点群地図を保存して、同じ SDF の形状と航空写真上で評価する。
走行制御の自己位置は模擬 GNSS FIX、障害物監視は独立した Top-URG の /scan を使う。
FAST-LIO には GNSS や真値 pose を入力せず、全区間で推定器をリセットしない構成とする。
全域走行と推定精度の達成状況は、実測結果に基づいて別々に記録する。

比較先は航空写真・DEM・OSM・画像抽出候補から生成し、経路の周囲 1 m を空けるために
一部形状を移動・削除した試験用デジタルツインである。現地測量済みの正解地図ではない。
同じ形状から LiDAR を生成しているため、この比較は再構成・自己位置推定の整合性評価である。
実環境の寸法精度や当日の障害物配置を独立に検証するものではない。

既存の FASTLIOシミュレータ接続検証、地理ウェイポイント_FLOAT_FASTLIO評価、
つくば2026全域デジタルツイン、ローカル FAST_LIO の publish_frame_world / publish_map を参照する。

## 2. 追加機能と配置

| ファイル | 役割 |
| --- | --- |
| tools/lio_map_recorder_node.py | /cloud_registered の camera_init 座標点群を分割保存 |
| tools/evaluate_lio_map.py | 地図集積、PCD 出力、SDF 表面比較、航空写真重ね合わせ |
| tools/evaluate_terrain_trial.py | --record-lio-map オプションで記録ノードを同時起動 |
| tests/test_lio_map.py | 表面標本、pose、voxel、binary PCD の検証 |

パッケージ内の開発・評価補助なので tools、テストは tests、記録は docs に配置する。
FAST_LIO 本体・既存ロボット・地形原本を変更しない。
生成ファイルは log/codex/fastlio_full_route に置き、Git 管理対象に追加しない。
既存の NumPy、SciPy、Matplotlib、sensor_msgs_py、rclpy を使用する。

## 3. 保存する地図の意味

FAST-LIO が各観測を自身の推定姿勢で camera_init へ変換した /cloud_registered を購読する。
観測はシミュレーション時刻で 1 秒以上離して採取し、各観測内を 0.25 m voxel で間引く。
再開・推定の補正のために真値で点を配置し直す処理は行わない。
受信 QoS は reliable depth 10、点群フレームと採取間隔を manifest.json に保存する。
1 観測ずつ npz を書き、長時間の全点群を RAM に保持しない。

終了後に全観測を同じ 0.25 m voxel で統合して fastlio_map.pcd を出力する。
各 voxel では最初の実観測点を残し、欠けた面を補完しない。
この地図は FAST-LIO 登録済み観測の集積であり、内部 ikd-tree の直接ダンプではない。
ローカル版の /Laser_map も登録済み観測の集積だが、未間引きで蓄積・全配信するため、
全域試験では負荷とメモリを制限する分割保存を使う。

## 4. 座標整合・評価方法

真値時系列と LIO 時系列が重なる初回時刻だけで XY、Z、yaw を整合する。
LIO は IMU 位置なので、真値モデル原点の高さに 0.6 m を加える。
roll/pitch による lever arm の厳密補正は未実施である。
全軌跡の ICP、ループ閉じ込み、GPS による後処理補正で drift を消さない。
評価用整合後の fastlio_map_enu.pcd も保存し、元座標版と区別する。

SDF の静的 visual 面を対象に、OBJ 三角面、円柱、楕円体等を約 0.5 m 間隔で標本化する。
車体は参照面に含めない。GPU LiDAR が描画形状を観測するため collision のみに限定しない。
点群から最寄りの表面標本への距離の中央値、95 percentile、RMSE、0.5 m / 1 m 以内の比率を出す。
これは厳密な point-to-mesh 距離ではなく、表面標本の疎密による正の距離誤差を含む。
標本は面上にあるため、その面への真の最短距離を小さく見積もる手法ではない。
地面点の比率で結果を良く見せないよう、モデル地表より 1 m 高い点を分けても報告する。
推定が大きくずれた点の地物分類は信頼できないため、この区分は補助指標として扱う。

航空写真には高さ着色点群と真値走行軌跡を重ねる。写真は上空視点であり、
地上 LiDAR が見える壁と屋根・樹冠は一致しない場合がある。
未観測の裏面・屋根を欠落として一律不合格にする完全性評価は行わない。
全点群の完全性を保証するためには、可視性・遮蔽を含む追加評価が必要である。

## 5. 実行手順

ROS Jazzy、obstacle_route_sim overlay、Gazebo runtime を有効化して実行する。
DDS domain と GZ_PARTITION は評価ツールが仮想環境用に隔離する。

```bash
python3 src/obstacle_route_sim/tools/prepare_waypoint_evaluation.py --source log/codex/tc2026_corridor_1m --output <全域waypoint出力先> --start 0 --count 1142
python3 src/obstacle_route_sim/tools/prepare_fastlio_trial.py --source <全域waypoint出力先>/fixed --output <全域LIO出力先>
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <全域LIO出力先> --gnss --fastlio <fast_lioの実行ファイル> --record-lio-map --timeout 7200 --lio-noise-profile reference
python3 src/obstacle_route_sim/tools/plot_lio_evaluation.py <全域LIO出力先>
python3 src/obstacle_route_sim/tools/evaluate_lio_map.py <全域LIO出力先>
```

起動・走行・終了を期限付きで行い、保存した分割観測を維持する。
途中停止や未到達の場合に全域成功として扱わない。
全区間を初期化なしで追跡できたかは、走行進捗、受信記録、LIO 誤差を併せて判断する。

## 6. 制限と残課題

- 模擬 Mid-360 は 70 m の規則格子点群で、反射・非反復走査・点内時刻を完全再現していない。
- 模擬 IMU のノイズ・取付・時刻は仮定で、実機の校正値ではない。
- URG は高さ 0.3 m の単一走査面で、低い段差・穴・上方張り出しを網羅しない。
- 地形は DEM に基づく概略で、実際の縁石・階段・路面摩擦・通行規則を全て含まない。
- GPS FLOAT と LIO の融合走行・品質監視・退化時停止はこの全域試験の対象外である。
- シミュレーション速度と走行ノードの wall timer が混在するため、所要時間は実機性能値ではない。

## 7. 実測結果

2026-09-13、全区間を連続走行して終了した。約 49.5 分の wall 時間を要した。
route_follower の index=1〜1141 の遷移を全て記録し、欠落 index はなかった。
最後に FINISHED と停止を確認した。真値走行距離は約 2,218.43 m だった。
概略経路との差は到達閾値による角の短絡等を含み、実地の完全な経路一致を意味しない。

| 指標 | 結果 |
| --- | --- |
| GNSS 制御の goal_pass | true |
| 真値ゴール誤差 | 0.428 m |
| 監視対象の接触 | 0 件 |
| URG /scan | 52,231 件 |
| 前方障害 hint | 12 件。AVOIDING 遷移なし |
| LIO odometry | 18,114 件 |
| IMU | 522,312 件 |
| LIO の比較時間 | 2,608 秒（sim） |
| LIO XY RMSE / 最大 / 終端 | 1.230 / 1.955 / 0.180 m |
| LIO 精度判定 | false。RMSE 0.2 m / 最大 0.5 m の基準未達 |
| 統合点群地図 | 2,185,546 点、PCD 約 26 MB |
| 参照表面サンプル | 6,115,931 点 |
| 表面サンプル距離 中央値 / P95 / RMSE | 0.760 / 1.870 / 1.036 m |
| 表面サンプルから 0.5 m / 1 m 以内 | 33.25% / 60.85% |
| 地表より 1 m 高い点 | 553,374 点、P95 1.891 m、1 m 以内 67.12% |

全体終了コード 1 は LIO の精度基準未達による。走行自体の失敗やプロセス異常ではない。
全起動プロセスを回収し、残留していないことを確認した。
生の camera_init 地図は fastlio_map.pcd、初回整合版は fastlio_map_enu.pcd に保存した。
map_comparison.png は航空写真への重ね合わせと表面距離分布、lio_comparison.png は
真値・LIO 軌跡と時系列誤差を示す。route_audit.json に全 index の通過監査を保存した。

### 7.1 座標整合と地図の形状を分けた診断

主評価は初回 XY/Z/yaw のみの整合を維持した。
別途、全軌跡の真値に対して剛体最小二乗整合を行い、固定した座標ずれの影響を調べた。
2D 診断では約 0.211 度の回転等の補正で XY RMSE 0.144 m、最大 0.426 m となった。
3D 診断では XYZ 軌跡 RMSE 0.426 m、地図の表面サンプル距離は中央値 0.287 m、
P95 0.738 m、1 m 以内 99.971% となった。拡大縮小は行っていない。
変換行列と数値は rigid_alignment_diagnostic.json に保存した。
これは正解軌跡を使った後処理であり、実運用の自己位置精度として扱わない。
地図の内部形状と地理座標への配置を分けて検討する必要があることを示す。

元の真値 CSV は yaw のみで roll/pitch を保持していなかった。
同じ world の静止初期状態だけを再現し、同時刻の位置差が約 3.2 µm であることを確認して
初期 roll/pitch を調べた。しかしその姿勢を直接適用する診断では距離 P95 が 6.023 m と
悪化したため、主評価や地図をその補正で上書きしていない。
初期姿勢・センサ frame・推定座標の対応は、重力方向を含めて追加確認が必要である。
再現試験は全走行の再試験ではなく、結果は initial_pose_map_diagnostic.json に分離した。
この段階で誤差の全てを LIO の累積 drift、または初期方位だけが原因と断定しない。

### 7.2 判定と次の課題

全区間の走行と点群地図の生成は可能だった。一方、地理座標上の初回整合だけでは
最大約 2 m の自己位置誤差が生じ、±1 m の経路余裕を前提とする制御へそのまま使用できない。
終端誤差が 0.18 m に戻ったことだけを根拠に、全区間の精度を良好とは扱わない。
次は真値 quaternion の保存、センサ frame と初期重力方向の整合検証、
GNSS FIX の位置・方位による初期化、品質に応じた GNSS/LIO 融合、退化時の停止を検討する。
参照地図の実地測量・実機 rosbag による独立検証も必要である。

## 8. 改版・検証

2026-09-13: 分割保存、点群地図化、表面比較を追加。
関連 pytest は 65 件成功。通常／symlink ビルドでパッケージのインストールを確認した。
詳細設計書に 10.8、README に全域評価への入口を追加した。
