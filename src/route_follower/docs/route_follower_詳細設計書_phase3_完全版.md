# route_follower 詳細設計書

## 構成と入出力

route_follower_nodeがROS通信、follower_coreが追従状態・回避・滞留判定を担当する。
active_routeを受信し、pose_enu、obstacle_avoidance_hint、manual_start、sig_recog、road_blockedを読む。
active_target、follower_state、信号認識の実行を切り替えるrecog_flagを配信する。
滞留報告はtc_route_msgs/srv/ReportStuckを使用する。
QoSと起動設定は[README](../README.md)を参照する。

## 状態と停止属性

IDLE、RUNNING、WAITING_STOP、STAGNATION_DETECTED、AVOIDING、WAITING_REROUTE、FINISHED、ERRORを持つ。
start_immediately=falseはmanual_start待ちとなる。
到達半径の既定は0.6 m。停止属性に到達するとWAITING_STOPへ移り、manual_startで再開する。
信号停止ではsig_recog=1も再開条件となる。
通常制御周期は20 Hz、目標再送の既定間隔は1秒。

WAITING_STOP／FINISHEDは車輪速度の停止確認ではない。
目標がNoneのときノードは配信しないが、navigatorの保持目標は解除されない。
manual_start=falseでRUNNINGを停止する機能もない。

## 滞留と回避

位置履歴の進捗距離・速度が閾値を下回る状態の継続から滞留を検出する。
ヒントの閉塞多数決と左右幅の統計、経路の左右許容幅から回避候補を作る。
L字の横シフト点と前進点をキューへ積み、試行失敗・再滞留ではreport_stuckへ進む。
再計画待ちには期限を持ち、失敗時はERRORとなる。

最低回避オフセットは0.35 mである。pick_offsetは最低幅を優先するため、
正の許可幅が0.35 m未満でも、それを超える候補を生成する場合がある。
この実装を通行境界の厳密な制限や、旋回を含む車体領域の検証として扱わない。
回避後の経路復帰では停止属性を持つ点のスキップを抑制する。

## メッセージと設定

Waypointの幅フィールドはleft_open／right_open、停止属性はline_stop／signal_stopである。
ROS型の正本は[Waypoint.msg](../../tc_route_msgs/msg/Waypoint.msg)、
[ReportStuck.srv](../../tc_route_msgs/srv/ReportStuck.srv)、
[FollowerState.msg](../../tc_route_msgs/msg/FollowerState.msg)とする。
ReportStuck.current_poseはPoseStampedである。

Core内の閾値とROSから指定できるパラメータは同一ではない。
ノードが宣言する値はroute_follower_node.pyとlaunch、ロジックの初期値はFollowerCore.__init__を参照する。

## 確認方法

初回開始、停止点・信号停止、manual_startの消費、閉塞継続、回避不能、
世代更新、サービス未接続・失敗・期限切れを確認する。
状態だけでなく、active_targetと下流cmd_velを併せて確認する。
幅0・最低幅未満・境界値、未観測ヒント、待機中の入力も含める。
