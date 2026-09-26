# FollowerStateのフィールド

## FollowerState.msg のフィールド一覧

| セクション | フィールド | 型 | 説明 |
| --- | --- | --- | --- |
| 進捗情報 | `route_version` | int32 | ルートバージョン番号 |
|  | `state` | string | `FollowerStatus` 名称 |
|  | `active_waypoint_index` | int32 | 現在追従しているウェイポイントのインデックス |
|  | `active_waypoint_label` | string | 現在追従ウェイポイントのラベル |
| 距離情報 | `active_target_distance_m` | float32 | 現在地とアクティブターゲットのユークリッド距離[m] |
|  | `segment_length_m` | float32 | 直前ウェイポイントから現在ウェイポイントまでの距離[m] |
| 回避・滞留 | `avoidance_attempt_count` | int32 | 現在ウェイポイントでの回避試行回数 |
|  | `last_stagnation_reason` | string | 直近の滞留理由ラベル |
| 障害物ヒント | `front_blocked` | bool | Hint 多数決による前方遮蔽判定 |
|  | `front_clearance_m` | float32 | Hint から得た前方余裕距離[m] |
|  | `left_offset_m` / `right_offset_m` | float32 | Hint サンプルの左右オフセット中央値[m] |


姿勢は/localization/pose_enu、停止属性はactive_routeのWaypointから取得する。
FollowerStateの状態は実測速度ゼロの保証ではない。
型の正本は[msg/FollowerState.msg](../msg/FollowerState.msg)を参照する。
