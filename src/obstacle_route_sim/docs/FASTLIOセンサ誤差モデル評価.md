# FAST-LIO センサ誤差モデルと比較評価

## 1. 目的と位置づけ

従来の規則格子・最大 70 m の良好な点群と小さい IMU 雑音は、実環境より楽観的だった。
入力側に誤差・欠測を加え、現実を想定した条件と厳しめの条件で推定を評価する。
目的の軌跡誤差になるように推定 pose を加工する処理は行わない。
ノイズを入れた結果が小さな誤差でも、意図的に結果を悪化させるための係数調整はしない。

実機 rosbag・反射強度・時刻差・温度・振動の測定値は未提供である。
そのため field_assumed は実環境を想定した仮定、conservative は厳しい試験条件であって、
実機に校正済みのモデルや、あらゆる実環境より厳しいことを保証する上限ではない。
既存の全域評価・FASTLIOシミュレータ接続検証を参照し、絶対配置と相対精度を分ける。

## 2. 公開仕様の根拠

- [Livox Mid-360 公式仕様](https://www.livoxtech.com/mid-360/specs):
  10 m・反射率 80%・25℃で測距ランダム誤差 1σ≦2 cm、角度 1σ<0.15°。
  検出距離は反射率 10%で40 m、80%で70 m（100 klx）。搭載 IMU 表記は ICM40609。
- [TDK ICM-40609-D 公式仕様](https://www.invensense.tdk.com/en-us/products/6-axis/icm-40609-d):
  gyro 雑音密度 4.5 mdps/√Hz、加速度 100 μg/√Hzを参考にする。
  同系列の公開値であり、Mid-360 内の型番末尾・帯域・フィルタ構成との一致は未確認である。

200 Hz 出力・等価雑音帯域 100 Hz を仮定すると、密度×√帯域は gyro 約0.000785 rad/s、
加速度約0.00981 m/s²となる。これを field_assumed の白色雑音の目安とした。
帯域100 Hzは製品設定の測定値ではない。Gazebo 側の既存雑音にも重ねるので、
合成標準偏差は gyro 約0.000825 rad/s、加速度約0.01020 m/s²になる。
固定 bias、時間変化、欠測率、遠方減衰、時刻誤差は以下に示す試験上の仮定である。

## 3. 条件とモデル

| 項目 | reference | field_assumed（標準） | conservative |
| --- | --- | --- | --- |
| 追加測距σ | 0 | 0.02＋0.0003 max(r−10,0) m | 0.04＋0.0006 max(r−10,0) m |
| 方位・仰角σ | 0 | 各0.10° | 各0.15° |
| 基本欠測率 | 0 | 8% | 20% |
| 遠方減衰の開始／尺度 | なし | 30／25 m | 25／15 m |
| 最大距離 | 70 m | 70 m | 55 m |
| 距離外れ値の確率 | 0 | 0.01% | 0.1% |
| 外れ値の追加距離 | なし | ±1 m 一様分布 | 同左 |
| 追加加速度白色σ | 0 | 0.01 m/s² | 0.02 m/s² |
| 追加角速度白色σ | 0 | 0.0008 rad/s | 0.0016 rad/s |
| 固定加速度 bias の各軸範囲 | 0 | ±0.04 m/s² | ±0.08 m/s² |
| 固定角速度 bias の各軸範囲 | 0 | ±0.0004 rad/s | ±0.001 rad/s |
| 時変加速度 bias の定常σ | 0 | 0.005 m/s² | 0.015 m/s² |
| 時変角速度 bias の定常σ | 0 | 0.0001 rad/s | 0.0003 rad/s |
| bias 相関時定数 | ― | 120 s | 120 s |
| IMU 各軸 scale 誤差の範囲 | 0 | ±0.1% | ±0.3% |
| IMU stamp 固定ずれ | 0 | ＋2 ms | ＋5 ms |
| IMU stamp 一様 jitter 範囲 | 0 | ±0.5 ms | ±1.5 ms |

r は元の距離である。点の保持確率は
(1−基本欠測率)×exp(−(max(r−減衰開始,0)/尺度)²) とし、最大距離の外は削除する。
例えば field_assumed の保持確率は40 mで約78%、70 mで約7%になる。
これは低反射・遮蔽・欠測が混じる観測の便宜的モデルで、各物体の反射率を再現していない。
角度誤差は観測値の方向に与え、変更後の光線を再 ray tracing する処理ではない。
外れ値の確率は保持後点に対する仮定で、製品の全発射に対する false alarm rate と同義ではない。
conservative の外れ値率は通常仕様より高く、悪条件・混合反射を想定したストレス注入である。

時間変化 bias は b次=exp(−dt/τ)b前＋σ√(1−exp(−2dt/τ))N(0,1) の OU 過程を使う。
無制限に増えるランダムウォークにせず、ゆっくり変化する誤差として扱う。
固定 bias と scale は起動時に seed から決め、LiDAR / IMU の乱数系列は分離する。
同じ入力・同じ seed に対する出力は再現できるが、Gazebo / DDS の実行順まで固定するものではない。

IMU covariance には追加白色雑音の分散のみ加算する。未知 bias の真値は推定器に渡さない。
IMU の観測値は元時刻で劣化させた後、stamp をずらす。通信遅延のキュー処理ではない。
200 Hz の通常入力では jitter 差が5 ms未満なので stamp の順序を維持する。
LiDAR の stamp は保持する。点内時刻・走査歪み・非反復走査・移動物体は未再現である。
URG と GNSS は今回の比較で変更せず、走行制御条件を揃える。

## 4. 実装・起動

- tools/lio_noise_core.py: ROS 非依存の誤差生成。
- tools/lio_sensor_adapter_node.py: LiDAR / IMU に適用して LIO へ配信。
- tools/evaluate_terrain_trial.py: profile / seed を指定し、sensor_noise.json に係数と実現値を保存。
- tools/review_lio_noise_trials.py: 全 seed の誤差を比較し PNG を出力。
- tests/test_lio_noise.py: 分布・seed・欠測・時刻・相対誤差のテスト。

既存検証補助と同じ tools に配置する。新しい外部依存はない。
標準 profile は field_assumed に変更する。過去の理想寄りの試験再現には reference を明示する。
FAST-LIO 本体の推定設定は全条件で同じであり、ノイズ条件ごとに調整していない。

```bash
ros2 launch obstacle_route_sim fastlio_sim.launch.py world:=<IMU追加済みtrial.sdf> noise_profile:=field_assumed noise_seed:=1
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <試験フォルダ> --gnss --fastlio <実行ファイル> --lio-noise-profile conservative --lio-noise-seed 1 --timeout 110
python3 src/obstacle_route_sim/tools/review_lio_noise_trials.py <比較試験ルート>
```

profile / seed は起動時に確定する。実行中変更には対応しない。
既定のセンサ topic、frame、整定待機は維持し、誤差モデル選択だけを追加した。
真値の軌跡や目標誤差を参照して誤差量を操作しない。

## 5. 評価条件と公平性

同じ市役所付近の約40 m・22 waypointを、各条件 seed=1,2,3 で実行する。
全条件で GNSS FIX の走行制御と URG 回避入力を共通にする。
3回は小標本であり、信頼区間・実機での故障率・長距離性能を推定できる試験数ではない。

1. 初期整合のみの XY 誤差も保持するが、それだけで不合格にしない。
2. 全体の剛体整合後の XY RMSE は、固定した位置・方位ずれを除く参考指標とする。
   拡大縮小は行わない。正解軌跡による後処理なので制御時精度とは扱わない。
3. 5秒以上となる最初のサンプルとの移動を、それぞれの開始時のロボット座標で比較する。
   真値の移動が1 m以上ある区間を対象に、短時間の相対移動誤差を評価する。
   窓は重複するため、サンプル数を独立した試行数とは扱わない。
4. 初期化失敗や記録不足も含め、成功した seed だけを選ばない。

短区間で出た誤差を2.2 kmへ比例外挿しない。実機以上に厳しいことの実証には、
静止・走行 rosbag の雑音密度、Allan deviation、反射強度別欠測率、時刻差の測定が必要である。

## 6. 結果

log/codex/lio_noise_trials に全記録を保存した。
各 profile で seed=1,2,3 の有効走行を得た。field_assumed の seed 3 は初回に
route_planner の service 応答 timeout が発生し、route_follower が IDLE のままであった。
この試行は真値移動 0.0009 m で、ノイズ下の走行精度として集計しない。
失敗記録を field_assumed_3 に残し、同じ seed の field_assumed_3_retry で走行を再確認した。
したがって有効走行は9回、未走行の試行を含めた実行数は10回である。
小さい静止誤差を良好な走行結果として含めないよう、10 m・10 s 未満を無効と判定する。

| 条件 | 剛体整合後 XY RMSE：平均［範囲］ | 5秒相対移動 RMSE：平均［範囲］ | 点の保持率平均 |
| --- | --- | --- | --- |
| reference | 0.0386 m［0.0375〜0.0406］ | 0.0355 m［0.0303〜0.0406］ | 100% |
| field_assumed | 0.1036 m［0.0719〜0.1353］ | 0.0757 m［0.0625〜0.0863］ | 89.0% |
| conservative | 3.0813 m［1.9950〜3.9608］ | 1.6564 m［0.9519〜2.3386］ | 75.3% |

初期整合のみの XY RMSE は reference 0.0497〜0.0522 m、field_assumed 0.1128〜0.1974 m、
conservative 2.4408〜4.6822 m だった。剛体整合後にも conservative の誤差は残り、
単なる地図全体の配置ずれだけでは説明できない。
有効走行9回の GNSS 制御は全てゴールへ到達したが、LIO 制御の成功とは解釈しない。

field_assumed は基準より誤差が増えたが、この短区間では相対移動推定を維持した。
conservative は3回とも大きく崩れ、通常の実環境精度の予測値ではなく退化ストレス条件とする。
白色雑音だけでなく遠方点減衰等を同時に変更しており、どの成分が支配的かは未分離である。
この試験結果を見て誤差が狙った大きさになるよう係数を変更していない。
この短区間比較時点ではノイズありの全2.2 kmは未実施だった。後続の全周結果を8章に示す。
実機 rosbag による校正、個別成分の ablation は未実施である。

noise_comparison.json に無効試行を含む各結果を保存し、noise_comparison.png に有効走行を表示する。
sensor_noise.json に実際に使った係数・seed・固定 bias・点数を保存する。
手元の実環境がこの仮定より良い場合も悪い場合もあり、「実機は必ずこの程度ずれる」としない。

## 7. 検証・改版

2026-09-13: 誤差モデルと条件選択、比較指標を追加。関連 pytest 70 件成功。
既存 FAST-LIO 接続記録と詳細設計書を参照し、標準設定の変更と過去条件を区別する。
詳細設計書に10.9、READMEに設定の入口を追加し、過去の試験手順には reference を明示した。
通常／symlink ビルドを確認した。通常 launch で見つかった誤差 core の配置誤りを修正し、
実行ファイルと同じ lib 配下へインストールする。
修正後の通常 launch では既定 field_assumed・seed=2、LiDAR 13,002 有効点、IMU 初期化を確認した。
Gazebo、bridge、adapter、FAST-LIO の全プロセスが正常終了した。
記録は log/codex/lio_noise_trials/launch_noise_verified.txt に保存した。

## 8. conservative 条件の全周評価

2026-09-13、seed=1、conservative の係数を変更せず、全1,142 waypointを連続走行した。
出力先は log/codex/fastlio_full_conservative_seed1 である。
GNSS FIXによる走行制御、Top-URGによる障害物監視、独立したFAST-LIO推定を使用した。
途中で推定器をリセットせず、GPS・真値による推定補正、ループ閉じ込みも行っていない。

### 8.1 実行と完走の確認

ROS Jazzy、パッケージoverlay、Gazebo runtimeを有効化して実行する。

```bash
python3 src/obstacle_route_sim/tools/prepare_fastlio_trial.py --source log/codex/full_route_waypoint/fixed --output <厳しめ全周出力先>
python3 src/obstacle_route_sim/tools/evaluate_terrain_trial.py --world <厳しめ全周出力先> --gnss --fastlio <fast_lioの実行ファイル> --lio-noise-profile conservative --lio-noise-seed 1 --record-lio-map --timeout 7200
python3 src/obstacle_route_sim/tools/plot_lio_evaluation.py <厳しめ全周出力先>
python3 src/obstacle_route_sim/tools/evaluate_lio_map.py <厳しめ全周出力先>
python3 src/obstacle_route_sim/tools/review_full_lio_trial.py <厳しめ全周出力先> --baseline log/codex/fastlio_full_route
```

- 実行時間51.78分、LIO比較時間42.89分（シミュレーション時刻）。真値走行距離2,217.85 m。
- index=1〜1141の全遷移を記録した。欠落なし、FINISHED、最後2秒の停止を確認した。
- ゴール誤差0.3375 m。車体等の監視対象への接触イベントは0件。
- URG scan 51,541件、front_blocked hint 6件。状態はIDLE→RUNNING→FINISHED。
  AVOIDINGへの遷移はなく、今回だけで障害物回避成功を主張しない。
- IMU受信515,362件、LIO受信16,461件。LIOの最大受信時刻間隔は1.4秒で、
  区間平均約6.40 Hzだった。全区間で常に10 Hz出力できたという結果ではない。
- 登録点群2,442フレームを保存し、0.25 m voxel統合後は2,267,397点となった。
- 有限・範囲内点に対するノイズ処理後の点保持率は71.17%。全発射光線に対する率ではない。
- 全試験プロセスの終了を確認した。評価コマンドの終了コード1はLIO精度基準の不合格による。
  GNSS走行の未完走やプロセスクラッシュを意味しない。dropout試験は今回は行っていない。

### 8.2 位置ずれと内部変形を分けた結果

| 指標 | 前回 reference 全周 | 今回 conservative 全周 |
| --- | ---: | ---: |
| 初回XY/yaw整合のXY RMSE | 1.230 m | 35.019 m |
| 同・最大XY誤差 | 1.955 m | 59.262 m |
| 同・終了時XY誤差 | 0.180 m | 6.179 m |
| 全軌跡の2D剛体整合後XY RMSE | 0.144 m | 16.960 m |
| 同・最大XY誤差 | 0.426 m | 37.857 m |
| 5秒間相対移動RMSE | 0.0327 m | 0.7682 m |
| 地図の初期整合後・表面距離P95 | 1.870 m | 12.286 m |
| 全軌跡の3D剛体整合後・表面距離P95 | 0.738 m | 5.435 m |
| 同・表面距離1 m以内の点の割合 | 99.971% | 45.686% |

2D/3D剛体整合は全軌跡を使う単一の回転・平行移動である。拡大縮小・鏡映・局所補正を
認めない。3D剛体整合後も軌跡XYZ RMSEは16.993 m、最大37.863 mとなった。
整合後の時間誤差には全体への最小二乗合わせの影響があり、開始時から大きく見えることは
開始時の推定がその距離だけ誤ったことを意味しない。これは事後の形状診断である。
相対誤差は重複する5秒窓であり、16,421組を独立試行数とみなさない。

コースの大まかな形は残っているが、区間ごとの位置・向きの違いが大きく、単一の剛体変換
だけでは重ならない。終点付近の誤差が途中より小さくても、推定が全周で正確だったことには
ならない。この条件でFAST-LIO単独をウェイポイント走行の自己位置に採用できるとは判断しない。
ただし実際にLIO閉ループで走らせた試験ではないため、LIO制御で必ず衝突すると断定しない。

地図距離は全点からSDF表面標本への距離であり、同じ物体への対応を保証しない。
地面や近隣の別物体に近い点は良く評価されるため、地図距離だけで合格とはしない。
3D整合後の地図Z範囲は約−12.38〜20.98 m。未補正の高さ・傾きの変形も含む。
初期整合で「地面より1 m高い」点が約200万点となった区分は、推定の大きなずれの影響を
受けるため、実際の非地面物体数として解釈しない。

### 8.3 レビューと次の改善項目

GNSS FIX制御の全周完了と、LIO精度不合格を分けて結論とする。GNSS FLOATとLIOの融合走行、
LIO異常検知による停止は未検証である。±1 mを空けた調整地図で、通行人等も追加していない。
現地の狭路・植生の揺れ・路面振動・動的障害物まで検証したものではない。

次は実機rosbagから時刻同期、IMU bias/雑音、取付外部パラメータ、点群欠測を校正する。
同時注入した成分のどれが支配的かは未確定なので、時刻差・角度誤差・遠方欠測・IMU誤差を
一つずつ外す比較を行う。成功する値へ恣意的に弱めるのではなく原因を分離する。
GNSS品質を考慮した融合、LIOの更新停止・ジャンプ・残差悪化の監視、安全停止を追加した後、
FLOAT区間を含む閉ループ試験が必要である。今回の値は単一seedのストレス試験の実測であり、
通常の実機精度の予測値や「あらゆる現実より少し厳しい」保証値ではない。

### 8.4 成果物・実装確認

review_full_lio_trial.pyを開発補助のtoolsに、剛体整合の縮尺・鏡映に関するテストをtestsに配置した。
full_review.jsonに全件照合、相対精度、地図距離、変換行列を保存する。
fastlio_map.pcdは元の推定座標、fastlio_map_enu.pcdは初期整合、diagnostic_map_enu.npyは
全軌跡を使った事後剛体整合である。後処理結果を元の推定地図と混同しない。
full_review.pngは全軌跡と全点群範囲、map_comparison.pngは航空写真・モデル比較を示す。
会話内に画像を掲載し、PC用のPCDだけで閲覧を完結させない。

通常／symlinkの対象パッケージビルドは成功した。
pytest -q -p no:cacheprovider に対象 test_full_lio_review.py、test_lio_noise.py、
test_lio_map.pyを指定し、追加の剛体整合テスト2件を含む12件が成功した。
既存の全域評価、誤差モデル評価、詳細設計書10.8/10.9を参照し、本章と全域評価への追記を行った。
