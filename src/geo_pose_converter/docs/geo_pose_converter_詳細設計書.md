# geo_pose_converter 詳細設計書

## 1. 責務と構成

geo_coreはWGS84 LLH↔ECEF↔ENUとheading/yawの変換を行う。
geo_pose_converter_nodeはGNSS単独位置、route_geo_projector_nodeは走行用ENUから表示用LLHを生成する。
llh_osm_viewer_nodeは診断用の読取専用HTTPビューアを提供する。
融合・自己位置品質判定・走行制御は本パッケージの責務ではない。

## 2. 入出力とQoS

| ノード | 入力 | 出力 |
| --- | --- | --- |
| geo_pose_converter_node | NavSatFix、RtkStatus、heading Imu | `/gnss/pose_llh`、GNSS単独ENU、`/geo/map_projection` |
| route_geo_projector_node | `/localization/pose_enu`、`/active_route`、`/active_target`、`/follower_state` | `/localization/pose_llh`、`/route/active_target_llh` |
| llh_osm_viewer_node | LLH自己位置・active_route・LLH目標 | HTTP表示、`/pose`・`/state` JSON |

active_routeはRELIABLE / TRANSIENT_LOCAL / depth 1、他の変換ノード入出力は
RELIABLE / VOLATILE / depth 10。map_projectionは1 Hzで再配信する。
heading Imuは保持するが、GNSS方位の生成にはRtkStatusを使う。

単体launchはGNSS単独ENUを `/localization/pose_enu` に出す。
共通融合構成は `/gnss/pose_enu` にremapし、融合の出力と分離する。
gnss_lio_fusionはGNSSのNavSatFix/RtkStatusを直接読む。

## 3. 座標とデータ

ProjectionConfigはprojection ID、datum、frame、原点緯度経度高度、map yaw offsetを保持する。
route_planner・GNSS変換・融合・projectorで同じ設定を使う。
WGS84楕円体からECEFへ変換し、原点との差をEast/North/Upへ射影する。
走行用位置は2Dのためz=0とする。逆変換は原点高度を計算に使うが、出力LLHはhas_altitude=falseとする。

headingは真北時計回り、yawはmapの+xから反時計回り。
map回転が0なら `heading_deg = 90 - degrees(yaw)` を0〜360°へ正規化する。
heading=0は真北として有効であり、未受信と同一視しない。

## 4. projectorの処理

受信したRoute.projectionを優先し、起動設定とのID・datum・frame・原点・回転の不一致をerrorログに出す。
入力poseのframe不一致はwarningとする。
自己位置ENUを逆変換し、目標はroute waypointのLLHがあればそれを使い、なければactive_targetを逆変換する。
followerのindex/labelと自己位置から、表示用目標の距離・方位を構成する。
表示用LLHは走行制御目標を置き換えない。

## 5. 起動と設定の共有

[README](../README.md)に単体launch・診断ビューアの例を示す。
融合併用は `gnss_pose_enu_topic:=/gnss/pose_enu`、GNSSなしは `enable_geo_pose_converter:=false` とする。
共通起動はsessionのprojection_paramsを各ノードへ渡す。
東京駅の同梱既定値を実コース原点として流用せず、使用する経路の設定を指定する。

## 6. パラメータ・設定仕様

| Parameter | 型 | 既定値 | 用途 |
| --- | --- | --- | --- |
| `projection_id` | string | `tokyo_station` | 投影条件識別子。既定値は開発チーム内の共通仕様に合わせた東京駅原点を表す |
| `datum` | string | `WGS84` | 測地系 |
| `map_frame_id` | string | `map` | ENU pose frame |
| `earth_frame_id` | string | `earth` | LLH pose frame |
| `child_frame_id` | string | `gps_link` | GNSS pose の child frame。`geo_pose_converter_node` のみで使う |
| `origin_latitude` | double | `35.681382` | LLH/ENU 変換原点の緯度。既定値は東京駅 |
| `origin_longitude` | double | `139.766084` | LLH/ENU 変換原点の経度。既定値は東京駅 |
| `origin_altitude` | double | `3.86` | LLH/ENU 変換原点の高さ [m]。計算に使用する原点高度 |
| `map_yaw_offset_rad` | double | `0.0` | ENU 軸から map 軸への回転 |
| `pose_enu_topic` | string | `/localization/pose_enu` | `route_geo_projector_node` が LLH へ変換する ENU 自己位置 topic |
| `pose_llh_topic` | string | `/localization/pose_llh` | `route_geo_projector_node` が publish する LLH 自己位置 topic |

`origin_latitude`、`origin_longitude`、`origin_altitude` は ENU 座標と LLH 座標を相互変換するための投影原点である。`geo_pose_converter_node` と `route_geo_projector_node` は同じ map frame 上の ENU 座標を扱うため、同一の `ProjectionConfig` を使わなければならない。`params/default.yaml` では `/**` の ROS 2 wildcard parameter に投影条件を定義し、両 node が同じ値を受け取る構成にする。node 別の `origin_*` 定義は持たせない。`route_geo_projector` には ROS 2 parameter file の target node として認識させるために空の `ros__parameters` のみを置く。launch や運用用 YAML で上書きする場合も、投影条件は共通定義として 1 箇所で管理する。


## 7. 表示と確認

診断ビューアは自己位置・経路・目標を表示し、受信から1秒でSTALE、3秒でLOSTとする。
これは観測精度の判定ではない。Leafletと背景タイルには外部通信が必要である。
PCのlocalhostは遠隔端末用の共有URLではない。

`tests/` でLLH/ENU往復、map回転、高度フラグ、投影不一致と目標生成を確認する。
統合時は経路・自己位置・目標に同じprojectionが使われていること、
`/localization/pose_enu` が単一配信元であることを確認する。
