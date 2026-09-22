# GNSS・基地局表示

## 実装

ダッシュボードに基地局/RTCM状態とマウントポイントを追加。
「GNSS・基地局」タブに地域、局名、ホスト:ポート、マウントポイント、CRC確認済みRTCMの
受信速度・累積量・最終受信からの秒数、再接続回数、通信エラー種別を表示する。
GNSS欄はRTK状態、衛星数、HDOP、補正データ年齢、方位・標準偏差、アンテナ間距離、
主アンテナの緯度・経度・高度。方位はアンテナ間の北基準時計回りであり、融合車体yawとは区別する。
標準偏差は推定品質であって実測誤差ではない。

診断 (`std_msgs/String` JSON) を1 Hzで購読する。購読名は相対名 `rtk_gps/ntrip_status` と
`rtk_gps/rtk_status` で、起動側が配信元へremapする。実機はUM982ドライバのprivate名
（`gnss_namespace` の既定は `/rtk_gps/rtk_gps_um982_node`）、シミュレーションは公開名
`/rtk_gps/...` を指す。bringup経由の起動を基本とし、`bringup.launch.py` が `gnss_namespace`
に応じてremapする。単独起動時は `robot_console.launch.py` の `rtk_status_topic` /
`ntrip_status_topic` 引数で指定する（既定値は公開名）。
HTML版の単独起動では `--ros-args -r` で両診断トピックをremapする。
実機向けの起動コマンドは [HTML遠隔観測UIの公開範囲](html_ui_access.md) を参照する。
GNSS位置コールバックとは独立して診断を送る。接続成功と補正受信を区別し、CRC有効RTCMが
5秒以上届かないと補正途絶。UIが診断topicを2.5秒超受信しないと更新遅延、5秒超で情報途絶。
GNSS自体の鮮度は既存の1秒/3秒閾値。未受信の数値は「—」、古いFIX表示を正常色にしない。
NTRIPを使わない設定は「無効」。外部補正入力の有無をNTRIP無効から推定しない。
シミュレーションのGNSSノードもNTRIP接続を持たないため `DISABLED` を配信し、「無効」と
表示される。これにより、シムの構成上の不使用と実機の診断途絶を区別できる。
認証ユーザー・パスワード・サーバ応答本文は新しい診断JSONに含めない。

局名・地域は `prepare_real_session` が `ntrip.station_id/station_label/site` に設定する。
既存のセッションを使う場合は再生成すると地域名・局名が表示される。
名前未設定でも接続先とマウントポイントは表示される。
局の選択は停止後のセッション生成時に `--site inagi|tsukuba --station ...` で行う。
本タブは観測画面で、走行中の接続先変更機能は設けていない。

HTML遠隔観測UIは測位状態・衛星数・基地局受信状態の3項目のみ常時表示。
詳細は標準details要素にまとめて初期状態で閉じる。開閉状態は定期更新でも維持し、
ページ再読み込み時は閉じる。スマートフォン幅で接続先を折り返す。
HTTPサーバの公開範囲・認証設定は変更していない。PC内のlocalhostはiPhoneへの共有リンクではない。
公開範囲の運用は [HTML遠隔観測UIの公開範囲](html_ui_access.md) を参照する。

## 確認（2026-09-20）

関連136テストに合格。実際のDDSで模擬NTRIP診断をConsoleCoreまで配信し、Qt/HTML表示、
不正JSON拒否、受信途絶、未受信、機密フィールド除外を確認。変更3パッケージをビルド済み。
実機UM982・車輪は起動していない。実測GNSS品質、走行中の挙動、iPhone実機表示は未検証。

画面検証環境は `log/codex/gnss_ui_20260920/venv`（system-site-packages）。
PCの標準PythonにはQtWebEngineが不足していたため、この環境にPyQt5 5.15.11、
PyQtWebEngine 5.15.7、pytest 8.4.2、pytest-forkedを導入して確認した。
通常のシステムPythonでのUI起動には `python3-pyqt5.qtwebengine` の導入が別途必要。
今回の検証環境でPC上に表示する場合は、workspaceで以下を実行する。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source log/codex/gnss_ui_20260920/venv/bin/activate
export LD_LIBRARY_PATH="$VIRTUAL_ENV/lib/python3.12/site-packages/PyQt5/Qt5/lib:$LD_LIBRARY_PATH"
python -m robot_console.ui_qt_main
```

`log/codex/gnss_ui_20260920/` にdashboard.png、gnss_tab.png、gnss_phone.pngを保存。
すべて模擬データによる表示例。gnss_phone.pngは同じ表示カードを縦に配置した閲覧用静止画で、
実際のPC画面の配置はgnss_tab.png。PNGは会話内でも提示し、PC内パスを共有URLと扱わない。

状態モデルと整形はcore、画面はui_qt、ROS購読はros、HTML変換はweb、検証はtools/testsに配置し、
既存のパッケージ分離を維持した。
