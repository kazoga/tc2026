# i-Cart mini 共通起動

実機とデジタルツインで同じ経路・原点・融合・走行UIを使用する。
起動直後はmanual_start待ちとし、走行開始は既存UIから行う。

## 同梱されたつくば2026地図で確認する

```bash
ros2 run icart_bringup run_digital_twin --output log/digital_twin/session01 --start-ui
```

固定地図を検証・展開し、simulation専用の共通起動へ渡す。地図の再取得・再生成は不要。
保存先は新規のみ。並行試験には `--domain-id 87` など別DDS domainを指定する。
[同梱地図・閲覧・準備手順](../obstacle_route_sim/maps/tsukuba2026/README.md)を参照する。

## 別の地図を使う

```bash
ros2 run icart_bringup prepare_session --trial <IMU追加済み地形ディレクトリ> --output <新規設定ディレクトリ>
ros2 run icart_bringup run_session --session <設定ディレクトリ>/session.yaml --environment simulation --start-ui
```

このCLI一括起動ではUIから同じprofileを再起動せず、manual_startだけを操作する。
UI起動管理から操作する別手順は設計書に記載する。

実機では全停止後、--environment realに変更する。run_sessionがDDS domainも設定する。実機のドライバはsession.yamlのhardware_launchで
既存の起動ファイルを指定するか、従来どおり別に起動する。実機固有のIPやポートを自動推測しない。
[構成・確認手順](docs/共通起動設計.md)を参照する。

ウェイポイント保存はroute_plannerのCSV仕様に合わせる。
prepare_sessionは`routes/fixed/waypoints.csv`をLLH正本（高度空欄）として保存し、
`routes/route_config.yaml`を生成する。ENUはroute_plannerが投影設定から生成する。
既存sessionは書き換えず、新規出力先へprepare_sessionを再実行して移行する。
全周の検証用正本は`route_planner/routes/tsukuba2026_digital_twin/`に配置した。
本番経路としての現地確認は未実施である。

## 手動で本番経路を採取する

```bash
# simulationは86、realは0。既存ノードを停止してから選択する。
export ROS_DOMAIN_ID=86
ros2 launch icart_bringup survey.launch.py environment:=simulation session:=<session.yaml> output:=<新規採取先> joy_input:=joy_node
```

共通UI・位置推定を起動し、初期走行モードをmanualとする。L1を保持してJoy操縦する。
Joy 0採取開始・1停止点・2信号停止点・3採取終了。実機はenvironment:=realとdomainを切り替える。
既にJoy配信中ならjoy_input:=externalとする。採取中に自律走行を開始しない。
編集画面の生成・段差判定の制約はroute_survey/README.mdを参照する。

手動経路採取もFAST-LIOとGNSSの融合位置を使用する。route_surveyの比較表示では、
GNSS単独軌跡と融合軌跡を切替でき、受信状態を時刻ごとに確認できる。
GNSS購読先は実機のgnss_namespace／模擬の/rtk_gpsに自動で合わせる。

## ランダム歩行者

```bash
ros2 run icart_bringup run_digital_twin --output log/digital_twin/people01 --start-ui --pedestrian-density 0.1 --pedestrian-seed 42
```

`session.yaml` の `pedestrian_density` は目標密度（人/100m²、既定0.1）、
`pedestrian_seed` は乱数seed（既定42）。既存セッションにも指定できる。
実行中は同じDDS domainの端末から変更できる。

```bash
ROS_DOMAIN_ID=86 ros2 param set /pedestrian_simulator density 0.5
```

- ロボットのSDFから全LiDARの最大到達距離と取付位置を読み取り、その外側3mで生成する。
  開始時も範囲内には生成しないため、人が歩いて入ってくるまで待ち時間がある。
- 密度は周辺円面積から算出する目標人数で、上限200人。地図境界・障害物・流入時間により
  実際にセンサ内で観測する密度は変動する。0は新規追加を停止し、既存の人は範囲外でのみ削除する。
- 計算対象はロボット周辺と最大6mの生成・削除用余白のみ。範囲内で人を再配置しない。
- 歩行速度は0.6～1.4m/s。ランダムに方向を変え、地形面に追従し、建物・樹幹・歩行者・
  ロボットへの侵入を避ける。地図外と急斜面には進まない。
- 胴体・頭・両脚の簡易形状をGazeboのvisual/collisionとして配置するため、URGとMID-360の
  レイ計測・遮蔽に反映される。歩行アニメーションや群衆の社会行動モデルは含まない。
- シミュレーション時刻を使用し、一時停止中は進まない。真値欠落時は更新を停止する。
  seedによる再現には同じロボット軌跡・時間刻みが必要。歩道の意味分類は使用しない。

開始地点は安定したGNSS区域として、最初の有効な真値位置を中心とする半径10mを
FIX条件にする。`session.yaml` の `gnss_start_fix_radius_m` で半径[m]を変更できる。
0で無効。区域外は従来の建物によるFLOAT・誤差モデルを維持し、区域はロボットに追従しない。
明示的な受信途絶の試験設定はFIX区域内でも有効。

## ROS 2実機統合・稲城／つくばの補正局選択

[実機ハードウェア統合](docs/実機ハードウェア統合.md)に構成・設定・検証範囲をまとめた。
`prepare_real_session --site inagi|tsukuba --station <選択局> --output <新規出力先>`で、
MID-360は車軸上0.414m、roll −0.6°・pitch +26.9°。主アンテナはLiDARの鉛直上15cm、
副アンテナはメインの後50cm（既存値）を共通設定へ反映する。2026-09-22の実測配置を基準とする。
車軸中央x=0は確認済み。高さは昨年度値を引き継ぐ。
`check_rtk_station --site inagi --list`で候補一覧、`--station <選択局>`でRTCM受信を検査する。
採取時は手動固定。新しい実機profileはJoy・手動介入・カメラ・URG・GNSS・Livox・車輪を共通起動する。
既存の外部hardware_launch方式は維持する。実機用YP-Spurパラメータは別途必要。
