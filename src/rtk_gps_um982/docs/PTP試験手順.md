# 既存EthernetでのMID-360 PTP試験

## 対象と構成

実機を走行させず、UM982の既存シリアルとMID-360のEthernet・電源だけで時刻同期を試験する。

```text
UM982 RMC → GNSS driverの受信フック → chrony SOCK → PC CLOCK_REALTIME
                                                        ↓ ptp4l -S
GNSS fix（RMCの日付で補完）                         MID-360 点群・IMU
                                                        ↓
                                                   FAST-LIO
```

`time_sync.enabled`は既定falseである。有効時のみ、既存シリアルの同じ読み取り処理から
チェックサム・状態A・日付が有効なGPRMC/GNRMCをchronyへ渡す。gpsdや追加ttyは使用しない。
受信COMに`GPRMC 1`を設定する。RMC不正・逆行・欠測時に時刻サンプルを補間して作らない。
SOCKが存在しない場合や権限不足は診断に出し、次の受信時に再試行する。

**本試験はソフトウェアタイムスタンプPTPで、PCのUTCをそのまま配信する。**
ptp4lはマスター固定でPC時計を補正しない。phc2sysは使わない。
ハードウェアPTPはPHCの時系とUTC/TAI補正が別途必要なため、本ツールでは提供しない。
MID-360の型は生UDPで1を期待する。0は未同期、2はGPS直接同期である。

## 1. ビルドと設定生成（一般ユーザー）

ROS JazzyとPython venvを有効化したワークスペースで実行する。

```bash
colcon build --symlink-install --packages-up-to rtk_gps_um982
source install/setup.bash
ros2 run rtk_gps_um982 ptp_trial prepare --output log/ptp_trial/setup
```

出力は`chrony.conf`（取り込み用断片）、`ptp4l.conf`、`gnss.yaml`（ROSパラメータ上書き用）。
同名出力は上書きしない。以下のCLIはインストール後に
`python -m rtk_gps_um982.ptp_trial`でも実行できる。

## 2. PCのchronyを設定する（管理者による手動適用）

Ubuntu 24.04向けの例。既存の時刻サービス設定を確認し、同じ時計を複数サービスで制御しない。
既存ファイルがある場合は比較・退避してから適用する。

```bash
sudo apt install chrony linuxptp acl
sudo install -m 644 log/ptp_trial/setup/chrony.conf /etc/chrony/conf.d/um982.conf
sudo systemctl restart chrony
sudo setfacl -m "u:$(id -un):rw" /run/chrony/um982.sock
```

`confdir /etc/chrony/conf.d`が既存chrony.confにあることを確認する。
SOCKはchronydが作成する。ACLはchrony再起動後に再設定する。GNSSノードをrootで起動しない。
別のNTP源も残してよいが、本ツールのPTP開始条件は`UM98`が選択源`#*`であること。
選択されない原因を`chronyc sources -v`で調査し、`trust`等で無条件に採用させない。

このSOCKはNMEAのUTCと行受信完了時刻との差をサンプルにするため、受信機の処理・USB転送の
遅延を含む。`precision 0.1`は設定上の見積もりであり実測精度ではない。
必要なら外部時刻基準と比較してchronyの`offset`を校正する。
ROSの`transport_delay_ms`は0のままにする。RMCには閏秒予告がないため、
閏秒付近の運用は本試験対象外とし、別時刻源・閏秒管理を含む運用設計が必要である。

## 3. GNSSとLiDARを起動する

GNSSノードは1つだけ起動する。現在使用しているGNSS設定を`config`に指定し、
シリアルポート、通信速度、NTRIP設定を引き継ぐ。

```bash
ros2 launch rtk_gps_um982 rtk_gps_um982.launch.py \
  config:=<使用中のGNSS設定.yaml> time_sync:=true
```

`time_sync`/`chrony_socket` launch引数が設定ファイルより優先される。
既存の別launchからノードを起動する場合は、最後のparametersに生成した`gnss.yaml`を追加する。
実時刻・`stamp_source=gnss_utc`・`transport_delay_ms=0`以外の組合せは起動時に拒否する。
`/rtk_gps/rtk_gps_um982_node/time_sync`にRMC鮮度、SOCK転送数、拒否数、SOCKエラーを配信する。
RMC日付が確定するまで位置を配信せず、2秒より古い日付アンカーも使用しない。

MID-360は既存のLivoxドライバ設定で起動し、点群と内蔵IMUを有効にする。
ネットワークは他のPTPマスターが存在しない専用接続とし、既存ptp4l/phc2sysサービスが
動作していないことを確認する。NIC/IPは既存LiDAR設定に合わせ、TailscaleやWi-Fiを指定しない。

```bash
chronyc sources -v
chronyc tracking
ros2 run rtk_gps_um982 ptp_trial check --interface <有線NIC>
```

開始条件はリンクあり、必要コマンドあり、UM98選択済み、LastRxが5秒以内、
Leap statusがNormal、System timeの残補正が50ms以内。
`--max-clock-offset`で残補正のしきい値を秒で指定できる。これはGNSS絶対精度の保証ではない。
時計のstepが必要な場合はFAST-LIOや融合を起動する前に完了させる。

## 4. PTP配信と生パケット観測（それぞれ別端末）

ROS環境をrootへ渡さず、OS側の`ptp4l`と受動観測だけを管理者として実行する例。
以下はリポジトリルートから実行する。インストール済みモジュールのCLIでも同じ動作になる。

```bash
sudo python3 src/rtk_gps_um982/rtk_gps_um982/ptp_trial.py run \
  --interface <有線NIC> --duration 180 --output log/ptp_trial/run01
```

前提を満たす場合だけ子プロセスを起動し、最長約1秒＋コマンド応答時間ごとに条件を再確認する。
喪失・Ctrl+C・SIGTERM・時間切れで自身のptp4lを停止する。出力先は空白なしの短い新規パスを使う。
`ptp4l.log`と開始時の`preflight.json`を保存する。

PTPが収束してから観測する:

```bash
sudo python3 src/rtk_gps_um982/rtk_gps_um982/ptp_trial.py monitor \
  --interface <有線NIC> --lidar-ip <MID-360のIPv4> \
  --duration 60 --output log/ptp_trial/packets01.jsonl
```

AF_PACKETでNICを受動観測し、指定LiDARの点群・IMUだけを抽出する。
LivoxドライバのUDPポートをbindせず、ドライバと並行して使用できる。
生UDPを配信するドライバが動いていなければデータは来ない。
JSONLには各パケットの同期種別・計測時刻・受信時刻を記録し、端末に集計を表示する。
サマリーの遅延分位点は各ストリーム末尾最大10000パケットが対象。
CRCを検証する通信品質試験ではなく、パケット長・送信元・形式を確認する時刻観測である。

両ストリームで種別1のみ、逆行なし、受信時刻−計測時刻が−50ms〜500msの範囲なら終了コード0。
それ以外やデータなしは2。接続切替中の種別0を含む場合も2となるので、収束後に再測定する。
この広い判定幅はepoch誤りや大きな遅延を検出するためのもので、1ms/10ms精度の合格判定ではない。
受信遅延・OSスケジューリングを含むため、種別1や小さい受信差だけで絶対同期精度を証明しない。

## 5. FAST-LIOとの確認・限界

同期後にFAST-LIOを起動し、`common.time_sync_en=false`、
`common.time_offset_lidar_to_imu=0.0`で内蔵IMUを使用する。
点群先頭時刻と走査終端のOdometry時刻の差を固定時差と誤認しない。
GNSS fix/status・点群・IMU・Odometry・time_sync診断を同時にbagへ記録して比較する。

位置とUNIHEADINGの同一epoch結合は既存実装の課題として残る。
本ツールは既存融合ノードへ自動的に停止指示を送らない。同期喪失試験時は融合を停止し、
同期復帰後にFAST-LIO・融合を再起動して履歴を初期化する。走行制御に直結させない静止試験用である。
NMEAのみで必要精度に届かなければPPSをPCへ追加するか、PPS＋UARTをMID-360へ直接接続する。

## 終了と復旧

PTP試験は時間制限またはCtrl+Cで終了する。MID-360を未同期のまま融合へ入力しない。
GNSSノードを停止してから`time_sync:=false`で通常起動に戻せる。
chrony設定は自動変更・自動復旧しないため、試験前の設定へ戻して再起動する。
PC時計を過去へ戻す操作は行わない。rootで生成したログは必要に応じ所有者を変更して参照する。

## 参照

- [全体提案](../../../docs/GNSS_FASTLIO時刻同期提案.md)
- [LinuxPTP ptp4l: TIME SCALE USAGE](https://www.linuxptp.org/documentation/ptp4l/)
- [chrony SOCK定義](https://github.com/mlichvar/chrony/blob/master/refclock_sock.c)
- [Livox同期手順](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)

## 既存配線での初回セットアップ（2026-09追加）

USB接続UM982を時刻源、PCをchronyで補正し、専用EthernetからMID360へPTPを配信する。
PPSは追加しない。NTPサーバをMID360へ直接指定する構成ではない。

```bash
ros2 run rtk_gps_um982 prepare_clock_host --user nkb --output log/clock_setup
sudo bash log/clock_setup/apply-host.sh
```

生成した適用スクリプトは、走行/自己位置推定プロセスが存在すれば拒否する。
chrony・linuxptp・aclをインストールし、既存chrony設定とACL drop-inを退避する。
Ubuntu既定設定とconf.d内のmakestepを無効化して、以後はslewで時計を収束させる。
GNSSはprefer指定で他の時刻源も残し、無条件のtrustやNTP外部公開は設定しない。
chrony再起動後も一般ユーザーのRMCリレーがSOCKへ書けるよう、systemdでACLを再適用する。
PTPはこのスクリプトから起動しない。インストール時のchrony初回起動もあるため、
必ず走行とFAST-LIOを停止して適用する。既存の独自include設定がある場合はそちらの
makestepも確認する。退避先は実行時に表示する。

既存の実機session設定をそのまま使い、モータを起動せずGNSSとMID360を立ち上げる:

```bash
ros2 launch icart_bringup clock_sensors.launch.py \
  session_directory:=/home/nkb/route_runs/20260922_123358_991938
```

既存GNSS/Livoxドライバが動いている場合は先に終了する。LANの接続先とIPは
session内のlivox.json（このPCではenp0s31f6、192.168.1.5）に合わせる。
`chronyc sources -v`でUM98が`#*`、`ptp_trial check`がreadyになるまで待ち、
前掲のrun/monitorでまず60秒以上測定する。monitorはJSONLと隣接する
`.summary.json`を保存し、両ストリームで観測時間の80%以上をカバーし、
最大受信間隔0.5秒以下、PTPのみ、逆行/不合理なepoch無しを満たす場合だけ成功する。
この受信間隔やepochの条件は動作確認であり、要求精度の保証ではない。

精度評価は、PTP収束後のGNSS・点群・IMU・LIO・車輪の測定時刻と受信時刻を
同時記録し、同じ直線/左右旋回で以前のログと比較する。NMEA/USB遅延はchronyの
残補正量に現れない系統誤差となるため、`precision_verified`は常にfalseとして
別途評価する。固定の-0.29秒を時刻や融合へ埋め込まない。
通常走行への切替はこの確認後に行い、GNSSリレーとPTPが途切れない起動構成を使う。
精度不足ならPPS+RMCの直接配線またはPPSによるPCの同期へ段階的に移行する。
