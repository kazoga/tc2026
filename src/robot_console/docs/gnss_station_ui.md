# GNSS・基地局表示

## 実装

ダッシュボードに基地局/RTCM状態とマウントポイントを表示する。
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
シミュレーションはNTRIPを持たず、`DISABLED` を配信する。無配信による診断途絶とは区別する。
認証ユーザー・パスワード・サーバ応答本文は診断JSONに含めない。

局名・地域は `prepare_real_session` が `ntrip.station_id/station_label/site` に設定する。
既存のセッションを使う場合は再生成すると地域名・局名が表示される。
名前未設定でも接続先とマウントポイントは表示される。
局の選択は停止後のセッション生成時に `--site inagi|tsukuba --station ...` で行う。
本タブは観測画面で、走行中の接続先変更機能は設けていない。

HTML遠隔観測UIは測位状態・衛星数・基地局受信状態の3項目のみ常時表示。
詳細は標準details要素にまとめて初期状態で閉じる。開閉状態は定期更新でも維持し、
ページ再読み込み時は閉じる。スマートフォン幅で接続先を折り返す。
HTTPサーバはloopbackで待ち受け、認証機能を持たない。公開範囲は[運用手順](html_ui_access.md)に従う。
PC内のlocalhostはiPhoneへの共有リンクではない。

## 確認方法

`tools/tests/` で不正JSON、未受信、期限切れ、機密フィールド除外、Qt/HTML表示を確認する。
模擬NTRIP診断の表示と、実GNSSの測位品質・補正受信は区別する。
Qt WebEngineの導入と正式UI起動は[README](../README.md)を参照する。
