# route_manager 詳細設計書

## 構成

route_manager_nodeがROS入出力、manager_coreが経路・再計画判断、manager_fsmが非同期状態遷移を担当する。
初期経路はget_route、可変ブロックの再計画はupdate_routeをroute_plannerへ要求する。
走行開始はroute_followerのmanual_start／start_immediatelyで制御する。

## インタフェースと世代

active_routeとmission_infoはRELIABLE / TRANSIENT_LOCAL / depth 1、
route_stateとmanager_statusは継続配信する。
follower_stateを購読して進捗・完了を反映し、report_stuckで滞留の詳細を受ける。
型・設定の一覧は[README](../README.md)を参照する。

plannerのRoute.versionはmajor版。managerはmajor/minorを持ち、公開値を
`(major % 100) * 100 + (minor % 100)` とする。plannerへはmajorだけを渡す。
初期major=1は公開値100である。ローカルなshift・skip・再配信はminorを進める。
report_stuckの世代とindex/labelを照合し、別世代の経路へ判断を適用しない。

## 状態と再計画

FSMはIDLE、REQUESTING、ACTIVE、WAITING_REROUTE、UPDATING、COMPLETED、ERRORを持つ。
followerのFINISHEDを受けてCOMPLETEDへ移るが、車輪速度ゼロを認定する処理ではない。

| 滞留条件 | 判断 |
| --- | --- |
| 通常・可変区間 | planner更新を試し、失敗時に理由に応じてshift、skipを試す |
| 通常・固定区間 | planner更新を省略し、shift、skipを試す |
| ROAD_BLOCKED・固定区間 | 現在経路を再配信する |
| ROAD_BLOCKED・可変区間 | planner更新のみ。shift／skipで封鎖を抜けない |

shiftはFRONT_BLOCKED、AVOIDANCE_FAILED、理由未分類の条件で検討する。
全候補が失敗したらfailedを返す。道路封鎖の固定区間再配信は、その道路が通れることの検証ではない。

ReportStuckの要求はcurrent_pose（PoseStamped）、reason_code／reason_detail、
route_version、current_index、current_wp_label、回避試行・hint・適用オフセットを持つ。
応答はdecision_code、waiting_deadline、offset_hint、noteである。
[サービス定義](../../tc_route_msgs/srv/ReportStuck.srv)を正とする。

## 実行・確認

planner接続待ち、サービスtimeout・retry、状態配信周期はノードパラメータで設定する。
FSM状態とroute_state.statusの表現は異なるため、文字列をそのまま相互代用しない。
異常応答、世代不一致、固定／可変、各滞留理由、停止属性を伴うskipを確認する。
経路・projection・LLH付帯情報が更新後も維持されることを確認する。
