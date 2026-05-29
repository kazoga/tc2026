# tc_geo_msgs 詳細設計書

## 1. 文書目的・対象範囲

本書は `tc_geo_msgs` パッケージの詳細設計を定義する。対象は LLH 系座標、LLH pose、測位品質、地図投影条件を表す ROS 2 interface である。実装対象 phase では `route_msgs` のパッケージ名は変更せず、`tc_geo_msgs` を新規追加して `route_msgs`、`geo_pose_converter`、将来の `localization_fusion` から共有する。

## 2. 背景・要求・スコープ

既存 route CSV は緯度経度を持つ場合があるが、公開される route topic では LLH 情報が失われていた。GUI、HTML 遠隔観測 UI、ログ、将来の route editor が同じ地理座標を参照できるよう、route に依存しない地理系 interface を分離する。

本パッケージの責務は message 定義のみである。座標変換、GNSS 受信、route 生成、localization fusion は本パッケージの責務外とする。

## 3. 全体構成・アーキテクチャ

`tc_geo_msgs` は `ament_cmake` の interface package として構成する。`std_msgs/Header` を用いて時刻と frame を保持し、地理情報の意味を field 名とコメントで固定する。

```text
src/tc_geo_msgs/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── GeoPoint.msg
│   ├── GeoPose.msg
│   ├── GeoPoseWithQuality.msg
│   └── MapProjection.msg
└── docs/
    └── tc_geo_msgs_詳細設計書.md
```

## 4. パッケージ構成・ファイル配置

| ファイル | 役割 |
| --- | --- |
| `msg/GeoPoint.msg` | WGS84 の latitude / longitude / altitude を表す |
| `msg/GeoPose.msg` | LLH point と heading / ENU yaw を持つ pose を表す |
| `msg/GeoPoseWithQuality.msg` | GNSS または fusion 後 pose の品質情報を表す |
| `msg/MapProjection.msg` | LLH と map ENU の変換条件を表す |

## 5. 外部インタフェース仕様

本パッケージは topic を直接 publish / subscribe しない。各 message は以下の用途で使用する。

| Message | 主な利用 topic / field | 用途 |
| --- | --- | --- |
| `GeoPoint` | `GeoPose.point` | LLH 位置 |
| `GeoPose` | `Waypoint.geo_pose`, `ActiveTargetLlh.target_pose` | LLH pose |
| `GeoPoseWithQuality` | `/localization/pose_llh`, `/gnss/pose_llh` | 自己位置 LLH と品質 |
| `MapProjection` | `Route.projection`, `/geo/map_projection` | map ENU と LLH の投影条件 |

## 6. パラメータ・設定仕様

該当なし。message package のため ROS parameter は持たない。投影原点などの値は `MapProjection` を publish するノード側の parameter で管理する。

## 7. データモデル・内部状態

### `GeoPoint`

| field | 型 | 単位 | 意味 |
| --- | --- | --- | --- |
| `latitude` | `float64` | deg | WGS84 緯度。北緯を正とする |
| `longitude` | `float64` | deg | WGS84 経度。東経を正とする |
| `altitude` | `float64` | m | WGS84 楕円体高または入力データの高度 |
| `has_altitude` | `bool` | - | altitude が有効かどうか |

### `GeoPose`

`heading_deg` は真北 0 度、時計回り正とする。`yaw_enu_rad` は ENU 座標系の yaw で、東 0 rad、反時計回り正とする。両方を持つ理由は、運用者表示では heading が自然で、制御・変換処理では ENU yaw が扱いやすいためである。

### `GeoPoseWithQuality`

`source` は GNSS、fusion、route file などの由来を表す。`fix_quality` は unknown / standalone / DGPS / RTK float / RTK fix を表す。`fusion_status` は fusion ノードが pose を採用可能かを表す。

### `MapProjection`

`projection_type=PROJECTION_LOCAL_TANGENT_PLANE` を初期実装の標準とする。`origin_latitude`、`origin_longitude`、`origin_altitude` は map frame の原点に対応する LLH である。`map_yaw_offset_rad` は ENU east/north 軸から map x/y 軸への回転角である。

## 8. 処理フロー・状態遷移

該当なし。message package のため runtime 処理は持たない。

## 9. 主要アルゴリズム・判定ロジック

該当なし。座標変換アルゴリズムは `geo_pose_converter` の `geo_core.py` に置く。

## 10. QoS・並行性・タイミング設計

該当なし。利用 topic の QoS は publish / subscribe する各ノードで定義する。

## 11. 起動・終了・launch 設計

該当なし。

## 12. エラー処理・ログ・診断

該当なし。値の妥当性検証は message 利用側で実施する。

## 13. UI・可視化仕様

本パッケージは UI を持たない。GUI は `GeoPoseWithQuality` と `ActiveTargetLlh` を参照し、GPS 受信状態、自己位置、active target を表示する。

## 14. 依存関係・ビルド設定

`std_msgs` と `rosidl_default_generators` に依存する。runtime では `rosidl_default_runtime` を export する。

## 15. テスト計画・受け入れ条件

- `colcon build --packages-select tc_geo_msgs` が成功する。
- `ros2 interface show` 相当の確認はローカル ROS 実行確認が許可された場合に実施する。
- downstream の `route_msgs` と `geo_pose_converter` が同時に build できる。

## 16. 互換性・移行・影響範囲

新規 package のため既存 topic 互換性への直接影響はない。ただし `route_msgs/Waypoint` と `route_msgs/Route` が本パッケージの message を参照するため、route stack の再ビルドが必要である。

## 17. 未決事項・今後の拡張

- `localization_fusion` 実装時に `GeoPoseWithQuality` の `fusion_status` と covariance 表現を追加拡張するか確認する。
- UTM 投影を正式採用する場合、`MapProjection` の UTM field の運用規則を追記する。

## 18. 改版履歴

| 日付 | 版 | 変更概要 |
| --- | --- | --- |
| 2026-05-28 | 1.0 | 初版。LLH 共通 message package として新規作成 |
