# geo_pose_converter 詳細設計書

## 1. 文書目的・対象範囲

本書は `geo_pose_converter` パッケージの内部設計を定める。対象は変換ライブラリ `geo_core`
と 3 ノードである。

| ノード | 役割 |
| --- | --- |
| `geo_pose_converter_node` | GNSS 単独の測位から LLH 位置と ENU 位置を作る |
| `route_geo_projector_node` | 走行用 ENU から表示用 LLH を作る |
| `llh_osm_viewer_node` | 診断用の読取専用 HTTP ビューア |

トピック名、型、起動方法は [README](../README.md) を正とする。本書は**変換式、投影設定の
共有規則、不一致の扱い、表示用データの構成と制約**を定める。

**本パッケージが座標変換の正本である。** 他のパッケージは変換式と投影原点を持たず、
ここを参照する。融合、自己位置の品質判定、走行制御は本パッケージの責務ではない。

記載内容はすべて実装済みである。実コースの原点での変換精度は実測していない（15 章）。

## 2. 背景・要求・スコープ

LLH（緯度経度高度）と ENU（走行用の平面座標）の相互変換を 1 箇所へ集約する。経路生成、
GNSS 変換、融合、表示が同じ投影条件を使うことを保証するためである。

責務は次の 4 点である。

- WGS84 LLH と map frame の ENU を相互変換する。
- 真北基準の方位と map 基準の yaw を相互変換する。
- 投影条件をメッセージとして配信し、受け取り側が対応関係を確認できるようにする。
- 走行用の ENU 位置・目標を表示用の LLH へ変換する。

非スコープは、位置の融合（`gnss_lio_fusion` が担う）、位置品質の判定、走行制御である。

## 3. 全体構成・アーキテクチャ

| モジュール | ROS 依存 | 責務 |
| --- | --- | --- |
| `geo_core.py` | **なし** | 変換式、投影設定のデータ構造と YAML 読み込み |
| `message_utils.py` | あり | ROS メッセージと `geo_core` の型の相互変換 |
| `geo_pose_converter_node.py` | あり | GNSS 単独の変換 |
| `route_geo_projector_node.py` | あり | 走行用 ENU から表示用 LLH への変換 |
| `llh_osm_viewer_node.py` | あり | HTTP ビューア |

`geo_core.py` は `yaml` 以外の外部ライブラリに依存しない。`numpy` も使わない。
`route_planner` は ROS パッケージとしてではなく**通常の Python ライブラリとして
`geo_pose_converter.geo_core` を import する**（[README](../README.md)）。

## 4. パッケージ構成・ファイル配置

| パス | 用途 |
| --- | --- |
| `geo_pose_converter/geo_core.py` | 変換ライブラリ |
| `geo_pose_converter/message_utils.py` | メッセージ変換 |
| `geo_pose_converter/*_node.py` | 3 ノード |
| `launch/geo_pose_converter.launch.py` | 変換ノードと診断ビューアの起動 |
| `params/default.yaml` | 投影条件の既定値 |
| `tests/` | ROS 非依存の単体試験 |

## 5. 外部インタフェース仕様

### 5.1 入出力

| ノード | 入力 | 出力 |
| --- | --- | --- |
| `geo_pose_converter_node` | `rtk_gps/fix`（NavSatFix）、`rtk_gps/rtk_status`（RtkStatus）、`rtk_gps/heading`（Imu） | `gnss/pose_llh`、GNSS 単独 ENU、`/geo/map_projection` |
| `route_geo_projector_node` | `localization/pose_enu`、`active_route`、`active_target`、`follower_state` | `localization/pose_llh`、`route/active_target_llh` |
| `llh_osm_viewer_node` | LLH 自己位置、`active_route`、LLH 目標 | HTTP 表示、`/pose`・`/state` の JSON |

**`rtk_gps/heading` の Imu は保持するが、GNSS 方位の生成には使わない。** 方位は
`RtkStatus.heading_deg` から取る。

**`heading_deg = 0.0` は真北を向く有効値である。** 値そのもので有効性を判断せず、
`RtkStatus` を 1 件でも受信したかどうかで判断する。未受信と「真北」を同一視しない。

### 5.2 投影条件の配信

`geo_pose_converter_node` が `/geo/map_projection` を **1 Hz で再配信する。** 後から
起動した購読側も投影条件を受け取れるようにするためである。

投影条件のフィールドは
[MapProjection.msg](../../tc_geo_msgs/msg/MapProjection.msg) を正とする。

### 5.3 topic 名の構成

単体 launch は GNSS 単独 ENU を `/localization/pose_enu` へ出す。**共通融合構成では
`/gnss/pose_enu` へ remap し、融合の出力と分離する。** `/localization/pose_enu` の
配信元を 1 つに保つためである。

`gnss_lio_fusion` は本パッケージを経由せず、GNSS の `NavSatFix` と `RtkStatus` を直接
読む。

### 5.4 高度の扱い

**走行用の ENU 位置は 2 次元として扱い、z を 0.0 とする。** 逆変換では原点高度を計算に
使うが、出力する LLH は `has_altitude=false` とする。高度を持たないことを受け取り側が
判定できるようにするためである。

フィールドの意味は
[GeoPoseWithQuality.msg](../../tc_geo_msgs/msg/GeoPoseWithQuality.msg)、
[ActiveTargetLlh.msg](../../tc_route_msgs/msg/ActiveTargetLlh.msg) を正とする。

## 6. パラメータ・設定仕様

### 6.1 投影条件

| Parameter | 型 | ノード宣言の既定値 | YAML の値 | 用途 |
| --- | --- | --- | --- | --- |
| `projection_id` | string | `default` | `tokyo_station` | 投影条件の識別子 |
| `datum` | string | `WGS84` | `WGS84` | 測地系 |
| `map_frame_id` | string | `map` | `map` | ENU pose の frame |
| `earth_frame_id` | string | `earth` | `earth` | LLH pose の frame |
| `origin_latitude` | double | **`0.0`** | `35.681382` | 変換原点の緯度 |
| `origin_longitude` | double | **`0.0`** | `139.766084` | 変換原点の経度 |
| `origin_altitude` | double | **`0.0`** | `3.86` | 変換原点の楕円体高 [m] |
| `map_yaw_offset_rad` | double | `0.0` | `0.0` | ENU 軸から map 軸への回転 |

**ノード宣言の原点の既定値は 0.0 であり、YAML の値とは異なる。** パラメータを与えずに
`ros2 run` で起動すると、原点が緯度 0・経度 0（ギニア湾）になる。launch は既定で
`params/default.yaml` を読むため、`ros2 launch` では東京駅原点が有効になる。

`geo_core.projection_config_from_mapping()` は辞書に値が無い場合に東京駅原点を補う。
**ノードのパラメータ宣言とは既定値が異なる。** YAML を読み込む経路（`route_planner` など）
では東京駅原点が、ノードのパラメータ経路では 0.0 が既定になる。

**同梱の東京駅原点を実コースの原点として流用しない。** 使用する経路の設定を指定する。

### 6.2 ノード別のパラメータ

| Parameter | 型 | 既定値 | 対象ノード |
| --- | --- | --- | --- |
| `child_frame_id` | string | `gps_link` | `geo_pose_converter_node` |
| `pose_enu_topic` | string | `/localization/pose_enu` | `route_geo_projector_node` |
| `pose_llh_topic` | string | `/localization/pose_llh` | `route_geo_projector_node` |
| `pose_child_frame_id` | string | `base_link` | `route_geo_projector_node` |
| `http_host` | string | `127.0.0.1` | `llh_osm_viewer_node` |
| `http_port` | int | `8765` | `llh_osm_viewer_node` |
| `stale_timeout_s` | double | `1.0` | `llh_osm_viewer_node` |
| `lost_timeout_s` | double | `3.0` | `llh_osm_viewer_node` |

### 6.3 投影条件の共有方法

`params/default.yaml` は **ROS 2 の wildcard（`/**`）に投影条件を定義し、両ノードが同じ
値を受け取る構成**にする。ノード別の `origin_*` 定義は持たせない。

`route_geo_projector` には、ROS 2 の parameter file の対象ノードとして認識させるために
**空の `ros__parameters` のみを置く。** launch や運用 YAML で上書きする場合も、投影条件は
共通定義として 1 箇所で管理する。

`geo_core.load_projection_config_from_yaml()` は `/**` の `ros__parameters` を読み、
ノード名を指定した場合はそのノードの定義で**上書きする。**

共通起動は session の `projection_params` を各ノードへ渡す。融合併用は
`gnss_pose_enu_topic:=/gnss/pose_enu`、GNSS を使わない場合は
`enable_geo_pose_converter:=false` を指定する。

## 7. データモデル・内部状態

### 7.1 データ構造

| 型 | 内容 |
| --- | --- |
| `LlhPoint` | 緯度・経度・楕円体高（不変） |
| `EnuPoint` | map frame の x・y・z（不変） |
| `ProjectionConfig` | 原点緯度経度高度、map yaw offset、投影 ID、測地系、2 つの frame 名（不変） |

### 7.2 ノードが保持する状態

| ノード | 保持する状態 | 更新契機 |
| --- | --- | --- |
| `geo_pose_converter_node` | 最新の `RtkStatus`、最新の heading Imu | 各受信 |
| `route_geo_projector_node` | 起動時の投影設定、実効の投影設定、最新の経路・目標・追従状態・LLH 自己位置 | 各受信 |
| `route_geo_projector_node` | 不一致ログの出力済みフラグ 3 個 | 初回の不一致検出 |
| `llh_osm_viewer_node` | 最新の LLH 自己位置・経路・LLH 目標と各受信時刻 | 各受信 |

**不一致のログは種別ごとに 1 回だけ出す。** 毎周期の出力でログが埋まるのを避けるためで
ある。

## 8. 処理フロー・状態遷移

### 8.1 GNSS 単独の変換

1. `NavSatFix` を受ける。
2. 最新の `RtkStatus` から方位の有無と値を決める（5.1 節）。
3. LLH を ENU へ変換し、`PoseWithCovarianceStamped` を作る。
4. LLH 位置に品質情報（RTK 状態から導出した fix quality）を付けて配信する。
5. ENU 位置を配信する。

投影条件は 1 Hz のタイマーで別に配信する。

### 8.2 走行用 ENU から表示用 LLH への変換

`localization/pose_enu` を受けるごとに次を行う。

1. frame 名が投影設定の map frame と一致するか確認する。不一致は warn（初回のみ）。
2. ENU 位置を LLH へ逆変換する。
3. `GeoPoseWithQuality` として配信する。`status_text` に `projected_from_enu` を入れる。
4. 目標の表示用データを作る（8.3 節）。

### 8.3 表示用目標の構成

`active_target` を 1 件でも受信していなければ何も配信しない。

1. 追従状態があれば、そこから経路世代・目標 index・目標ラベルを取る。無ければ経路の
   世代のみを使い、index とラベルは未確定とする。
2. 目標の waypoint を探す。**index で引き、ラベルが空か一致すればそれを採用する。**
   一致しない場合はラベルで全体を走査する。
3. 見つかった waypoint が LLH を持つ場合は**その LLH をそのまま使う。** 持たない場合は
   `active_target` の ENU を逆変換する。
4. 自己位置との距離と方位を付けて配信する。
5. 目標 index が負の場合、**回避サブゴールとして印を付ける。** 経路上の点ではないことを
   受け取り側が判定できるようにするためである。

**表示用の LLH は走行制御の目標を置き換えない。** 走行制御は ENU の `active_target` を
使う。

### 8.4 経路の投影条件の照合

`active_route` を受けると、メッセージに含まれる投影条件を**実効の投影条件として優先
する。** 起動時の設定と比較し、投影 ID・測地系・frame 名・原点・回転のいずれかが異なれば
**error ログを出す（初回のみ）。**

**不一致でも処理は続ける。** 経路側の投影条件に従って変換するため、表示は経路と整合する。
起動設定の誤りを検出して知らせることが目的である。

### 8.5 終了処理

ノードを破棄して ROS をシャットダウンする。終了時に安全値を配信しない。ビューアは HTTP
サーバを停止する。

## 9. 主要アルゴリズム・判定ロジック

### 9.1 LLH から ENU への変換

WGS84 の長半径 6378137.0 m、扁平率 1/298.257223563 を使う。

1. 対象点と原点をそれぞれ測地座標から ECEF へ変換する。
2. ECEF の差分を、原点の緯度経度で定まる局所座標系へ射影して East / North / Up を得る。
3. East / North を `map_yaw_offset_rad` で回転して map の x / y にする。Up はそのまま z と
   する。

逆変換は同じ手順を逆に辿る。ECEF から測地座標への変換は反復を使わない閉形式
（Bowring 法）で求める。

### 9.2 接平面の誤差と地表点の逆変換

**走行用 ENU の z を 0.0 のまま逆変換すると、水平位置に誤差が出る。**

局所接平面では、原点から離れるほど地表が接平面から下がる。距離 d に対して概ね
`d² / 2R` である。z = 0.0 のまま逆変換すると「原点の鉛直方向へ持ち上げた点」を逆投影する
ことになる。原点の鉛直方向は遠方では地表の鉛直方向から `d / R` だけ傾いているため、
水平位置に `(d² / 2R) × (d / R)` の誤差が出る。**原点から 52.7 km 先で約 1.8 m である。**

そのため、地表点の逆変換には専用の関数を使う。指定した地表高度と整合する接平面上の z を
**反復的に求めてから**逆投影し、水平位置の誤差を除去する。これにより順変換との往復が
一致する。

反復回数は既定 3 回である。**数 km から数十 km の範囲では 3 回で収束する。** それを超える
範囲での収束は確認していない（15 章）。

### 9.3 方位と yaw の変換

**方位（heading）は真北基準の時計回り、yaw は map の +x から反時計回り**である。

```
yaw[rad]      = π/2 − radians(heading[deg])
heading[deg]  = normalize(degrees(π/2 − yaw[rad]))
```

`normalize` は `[0, 360)` へ丸める。map 回転が 0 の場合、`heading_deg = 90 − degrees(yaw)`
を `[0, 360)` へ正規化した値になる。

**map 回転が 0 でない場合、この式は成り立たない。** 回転を含む方位の算出には、map 差分を
ENU へ戻してから `atan2(east, north)` を取る関数を使う。

### 9.4 品質情報の付与

GNSS 単独の変換では、`RtkStatus` の状態を fix quality へ写像して付ける。補正データ年齢が
不明な場合は −1.0 を入れる。

**`route_geo_projector_node` が配信する LLH 自己位置は、品質を実際には評価していない。**
source を未知、融合状態を OK として固定で入れ、`status_text` に `projected_from_enu` を
入れるだけである。**入力の ENU 位置の品質は引き継がない。** この topic の品質フィールドを
位置の信頼度として扱わない（17.1 節）。

### 9.5 ビューアの鮮度判定

自己位置の受信からの経過時間で 3 段階に分ける。

| 条件 | 表示 |
| --- | --- |
| `stale_timeout_s`（既定 1.0 秒）以内 | 正常 |
| `lost_timeout_s`（既定 3.0 秒）以内 | STALE |
| それ以上 | LOST |

**これは観測精度の判定ではない。** 受信が続いているかどうかだけを表す。

## 10. QoS・並行性・タイミング設計

| 対象 | QoS |
| --- | --- |
| `active_route` の購読 | RELIABLE / TRANSIENT_LOCAL / depth 1 |
| その他の変換ノードの入出力 | RELIABLE / VOLATILE / depth 10 |

| 周期 | 値 |
| --- | --- |
| 投影条件の再配信 | 1 Hz |
| その他 | 入力の受信契機のみ |

変換ノードは callback group を指定せず、単一スレッドの executor で直列に処理する。
ロックを持たない。

ビューアは `ThreadingHTTPServer` を別スレッドで動かし、ROS の spin と分離する。表示用の
最新値はロックで保護する。

## 11. 起動・終了・launch 設計

entry point は 3 ノード分である。launch の引数と起動例は [README](../README.md) を正と
する。

`launch/geo_pose_converter.launch.py` は変換ノードと診断ビューアをまとめて起動する。
`enable_geo_pose_converter:=false` で GNSS 変換ノードを起動しない構成にできる。

起動時に外部デバイスへ接続しない。ファイルの読み込みはパラメータ YAML のみである。

## 12. エラー処理・ログ・診断

| 事象 | 挙動 |
| --- | --- |
| 経路の投影条件と起動設定の不一致 | **error**（初回のみ）。経路側の条件で処理を続ける |
| 入力 pose の frame 名の不一致 | **warn**（初回のみ）。処理を続ける |
| `active_target` の frame 名の不一致 | 同上 |
| `RtkStatus` が未受信 | 方位を無効として扱う |
| 目標の waypoint が見つからない | `active_target` の ENU を逆変換して使う |
| `active_target` が未受信 | 表示用目標を配信しない |

`debug` と `fatal` は使わない。`DiagnosticReporter` を使わず、**健全性監視の対象に
入っていない。**

## 13. UI・可視化仕様

`llh_osm_viewer_node` が自己位置・経路・目標を地図上に表示する。`/pose` と `/state` で
JSON も返す。

**読取専用である。** 走行指令も起動停止も提供しない。

**Leaflet と背景タイルの取得には外部通信が必要である。** 通信できない環境では地図が
表示されない。

HTTP サーバは既定で 127.0.0.1 の 8765 番で待ち受ける。**PC の localhost は遠隔端末用の
共有 URL ではない。**

## 14. 依存関係・ビルド設定

`ament_python` でビルドする。依存は `rclpy`、`sensor_msgs`、`geometry_msgs`、`std_msgs`、
`tc_geo_msgs`、`tc_route_msgs`、`rtk_gps_um982_msgs`、`python3-yaml` である。

`geo_core.py` は `yaml` のみに依存する。`numpy` を使わないため、他パッケージから軽量に
import できる。

## 15. 確認方法と未確認範囲

### 15.1 自動確認

`tests/` の 3 ファイルが ROS 非依存で動く。

| 試験 | 固定する内容 |
| --- | --- |
| `test_geo_core.py` | LLH / ENU の往復、map 回転、方位と yaw の変換、地表点の逆変換 |
| `test_message_utils.py` | 高度フラグ、投影条件のメッセージ変換、品質の写像、目標データの構成 |
| `test_llh_osm_viewer.py` | 鮮度判定、JSON の内容 |

### 15.2 手動で確認する範囲

- 経路・自己位置・目標に**同じ投影条件が使われていること**
- `/localization/pose_enu` が**単一の配信元**であること（5.3 節）
- 投影条件の不一致で error が出ること
- 診断ビューアの表示と、外部通信が無い環境での挙動

### 15.3 未確認範囲

- **実コースの原点での変換精度を実測していない。** 変換式と誤差の見積り（9.2 節）は
  計算によるもので、測量値との照合ではない。
- 地表点の逆変換の反復回数 3 回は、**数 km から数十 km の範囲でのみ収束を確認している。**
  それを超える範囲は未確認である。
- `map_yaw_offset_rad` が 0 でない構成を実機で使っていない。回転を含む変換は単体試験での
  確認のみである。
- 原点から遠い位置（数百 km 以上）での変換精度を確認していない。
- 診断ビューアの長時間表示を確認していない。

## 16. 互換性・移行・影響範囲

`tc_geo_msgs` の `MapProjection`、`GeoPoseWithQuality`、`GeoPose`、`GeoPoint` と、
`tc_route_msgs` の `Route`、`Waypoint`、`ActiveTargetLlh`、`FollowerState` に依存する。

**投影原点を変更すると、既存の LLH 系経路 CSV から生成される ENU 座標がすべて変わる。**
`route_planner` の経路データ、走行側の自己位置の原点、保存済みの ICP 地図を同時に
合わせる必要がある。

`geo_core` の変換関数は `route_planner` が Python ライブラリとして import する。関数の
名前と引数を変える場合は `route_planner` を併せて更新する。

`params/default.yaml` の `/**` 構成（6.3 節）を変えると、両ノードが別の原点を受け取る
構成になり得る。ノード別定義を追加しない。

## 17. 未決事項・今後の拡張

### 17.1 表示用 LLH 自己位置の品質が実体を持たない

9.4 節のとおり、`route_geo_projector_node` が配信する LLH 自己位置の品質フィールドは
固定値である。入力の ENU 位置が持つ共分散や、融合の品質を引き継いでいない。
入力から引き継ぐか、品質フィールドを使わないことを型の側で表すかの選択が残る。

### 17.2 ノード宣言の原点の既定値が 0.0 である

6.1 節のとおり、パラメータを与えずに `ros2 run` で起動すると原点が緯度 0・経度 0 になる。
誤りに気付かないまま変換結果を配信し得る。原点が未設定の場合に起動を失敗させるか、
`geo_core` 側と同じ既定値へ揃えるかの選択が残る。

### 17.3 投影条件の不一致を検出しても処理を続ける

8.4 節のとおり、経路の投影条件と起動設定が異なる場合、error を出して経路側の条件で続ける。
表示は経路と整合するが、**走行側の自己位置が起動設定の原点で動いている場合、表示と走行の
原点が食い違う。** 不一致を致命的として扱うかは未決である。
