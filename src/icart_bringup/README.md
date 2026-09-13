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
