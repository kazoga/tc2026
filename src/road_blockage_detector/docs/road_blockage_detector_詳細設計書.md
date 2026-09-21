# road_blockage_detector 詳細設計書

## 1. 目的とスコープ
road_blockage_detector ノードは YOLO 推論結果と自己位置を入力として、経路封鎖看板の有無と封鎖位置を判定する。判定結果は robot_navigator を経由してロボット動作（停止／再開）へ反映される。本設計書ではノードの責務、I/F、内部処理、保持データ構造、認識結果の出力、異常時動作を定義する。

## 2. 背景と前提条件
- YOLO 推論結果は `yolo_detector` パッケージが `vision_msgs/msg/Detection2DArray` として publish 済みである。
- `road_blockage_detector` は YOLO モデルのロードや画像推論を行わず、検出結果の意味判定だけを担当する。
- ノードは `/localization/pose_enu` を唯一の自己位置情報として使用し、TF2 には依存しない。
- YOLO の生の検出矩形だけを確認する画像は `yolo_detector` 側の `/perception/road_blockage/detection_image` を使用する。
- 本ノードは画像を購読・配信しない。経路封鎖判定の根拠を確認するため、検出矩形と判定状態を `/perception/road_blockage/overlay` へ publish し、画像への重畳は表示側（`robot_console`）が行う。
- 過去の経路封鎖位置はノード内メモリで累積保持すればよい。永続化要件は無し。

## 3. ロボット挙動との連携と全体フロー
- YOLO で経路封鎖看板を検出し、`road_blocked` を true で publish した時点で robot_navigator が走行を一時停止する。停止中も本ノードは検知を継続する。
- 検知開始から `confirmation_duration` 秒経過前に封鎖検知が途切れた場合、`road_blocked` を false で publish し、robot_navigator が走行を再開する（誤検知扱い）。
- 検知が `confirmation_duration` を超えて継続した場合は封鎖を確定し、封鎖位置を `blocked_positions` に追加する。`road_blocked` は true のまま維持し、ロボットは停止を継続する。
- 封鎖確定後、`route_follower` 側で `stagnation_duration_sec` が経過すると滞留と判定され、`report_stuck` を `route_manager` に要求する。`route_manager` はリルートを実施し、封鎖を回避した経路で走行を再開する。
- 検知が取り消され `road_blocked=false` を publish した場合、robot_navigator は hold 時間経過後に解除候補として取り込むため、誤検知時に自動で走行再開できる。

## 4. ノードの役割・責務
| 区分 | 内容 |
| ---- | ---- |
| 主要役割 | YOLO 検知から経路封鎖看板候補を抽出し、時系列の判定カウントを管理して仮／確定封鎖を判断する。 |
| 責務1 | 検知パラメータ（クラス ID・スコア・バウンディングボックス閾値）に基づくフィルタリング処理を実装する。 |
| 責務2 | 判定期間内のカウント履歴から封鎖確率を評価し、`road_blocked` Bool を publish する。 |
| 責務3 | 確定封鎖とみなした自己位置を履歴として保持し、再侵入時の多重検知抑止を行う。 |
| 責務4 | `/localization/pose_enu` が取得できない場合や Detection と `/localization/pose_enu` の時刻差が大きい場合に警告ログを出力する。 |
| 責務5 | 検出矩形、採用可否、判定状態を `PerceptionOverlay` として publish し、表示側が画像へ重畳できるようにする。 |

## 5. 外部 I/F
### 5.1 サブスクライブ
| トピック | 型 | QoS | 用途 |
| -------- | -- | --- | ---- |
| `/perception/road_blockage/detections` | `vision_msgs/msg/Detection2DArray` | SensorData | YOLO 推論結果受信。 |
| `/localization/pose_enu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Default | 最新の自己位置キャッシュ。Detection 時刻との差も確認する。 |

### 5.2 Publish
| トピック | 型 | QoS | 条件 |
| -------- | -- | --- | ---- |
| `/road_blocked` | `std_msgs/msg/Bool` | Default | 仮判定状態遷移時（false→true、true→false）にのみ publish し、robot_navigator へ通知する。 |
| `/perception/road_blockage/overlay` | `tc_perception_msgs/msg/PerceptionOverlay` | SensorData | `Detection2DArray` 処理時に、検出矩形と判定状態を publish する。header は元画像フレームのものを引き継ぐ。 |

### 5.3 画像 topic の位置づけ
- `/perception/road_blockage/detection_image` は `yolo_detector_road_blockage` が publish する YOLO 生検出の確認用画像である。
- `/perception/road_blockage/overlay` は `road_blockage_detector` が publish する意味判定後の認識結果である。
- `robot_console` は `/usb_cam/image_raw` へ本 overlay を重畳して 1 枚のカメラパネルに表示する。未受信時は判定チップを未受信として扱う。
- 認識結果の配信は運用確認用であり、制御判断の正本は `/road_blocked` とする。

### 5.4 自己位置取得
- `/localization/pose_enu` を subscribe し、メッセージヘッダーの時刻を含めてキャッシュする。
- Detection2DArray のヘッダー時刻と最新の `/localization/pose_enu` ヘッダー時刻に 3 秒以上の差がある場合は警告を出すが、処理自体は続行する。
- 最新 `/localization/pose_enu` が未取得の場合は警告を出して検知処理をスキップする。

## 6. パラメータ仕様
| 名称 | 型 | 既定値 | 説明 |
| ---- | -- | ------ | ---- |
| `detections_topic` | string | `/perception/road_blockage/detections` | YOLO 推論結果入力 topic。 |
| `pose_enu_topic` | string | `/localization/pose_enu` | 自己位置入力 topic。 |
| `road_blocked_topic` | string | `/road_blocked` | 経路封鎖判定出力 topic。 |
| `overlay_topic` | string | `/perception/road_blockage/overlay` | 認識結果の出力 topic。 |
| `target_class_id` | int | 0 | 経路封鎖看板に対応するクラス ID。 |
| `score_threshold` | float | 0.5 | 最小スコア。未満の検知は除外。 |
| `bbox_width_min` | float | -1 | 幅閾値下限 [pixel]。負値なら判定スキップ。 |
| `bbox_width_max` | float | -1 | 幅閾値上限 [pixel]。負値なら判定スキップ。 |
| `bbox_height_min` | float | -1 | 高さ閾値下限 [pixel]。負値なら判定スキップ。 |
| `bbox_height_max` | float | -1 | 高さ閾値上限 [pixel]。負値なら判定スキップ。 |
| `bbox_bottom_max` | float | -1 | バウンディングボックス下端の最大位置 [pixel]。画像下端からの距離で定義し、負値なら判定スキップ。 |
| `decision_duration` | float | 3.0 | 判定期間長 [秒]。この期間のカウント履歴を保持。 |
| `decision_frame_ratio` | float | 50.0 | 判定期間内で「カウント>=1」の秒バケット割合 [%] がこの値以上で仮判定成立。 |
| `confirmation_duration` | float | 10.0 | road_blocked=true 継続時間が本値を超えたら封鎖確定。 |
| `multi_detection_suppression_range` | float | 10.0 | 過去封鎖位置中心から本距離[m]以内に現在位置が入った場合はカウント0としてスキップ。 |

## 7. 内部データ構造
| 名称 | 形式 | 内容 |
| ---- | ---- | ---- |
| `count_history` | `collections.deque[tuple[float, int]]` | 「秒単位バケットの開始時刻（float, ROS time 秒）」と、その1秒間に記録した判定カウントの合計を保持。`decision_duration` 秒を超えた古いバケットは随時削除する。検知周期に依存せず秒単位のスライディングウィンドウを構成する。 |
| `last_detection_time` | `builtin_interfaces/msg/Time` | 最後に `detections` を処理した時刻。間引きや欠損検出に利用。 |
| `temporary_decision_count` | int | 仮判定カウント。秒バケット割合に応じて加算／リセット。 |
| `blocked_positions` | `list[geometry_msgs.msg.Pose]` | 確定封鎖と判断した際の `map` 座標（`/localization/pose_enu` 基準）。多重検知抑止に使用。 |
| `latest_pose_enu` | `Pose` | `/localization/pose_enu` からの最新値キャッシュ。 |
| `latest_pose_enu_time` | `rclpy.time.Time` | `/localization/pose_enu` メッセージヘッダーの時刻。Detection のヘッダー時刻との乖離チェックに利用。 |
| `blocked_state_started_at` | float | road_blocked=true に遷移した ROS 時刻 (秒)。経過時間で確定判定。 |
| `latest_image_header` | `std_msgs/msg/Header` | `latest_image` に対応するヘッダー。publish 時に可能な範囲で継承する。 |
| `last_valid_detection_count` | int | 最新フレームで閾値条件を満たした検知数。debug ログに利用。 |
| `last_detection_ratio` | float | 最新の `count_history` から算出した検知割合 [%]。debug ログに利用。 |

## 8. 処理フロー
1. **起動処理**
   - パラメータ宣言・取得。
   - サブスクライバ／パブリッシャを生成。
   - 判定期間から秒単位バケットの上限幅を決定し、deque を初期化。
   - ロガーで起動を通知。

2. **画像受信 (`sensor_msgs/msg/Image` コールバック)**
   - 画像変換に失敗した場合は警告または error ログを出し、直前の画像を維持する。
   - 画像受信のみでは判定処理を行わない。

3. **検知メッセージ受信 (`Detection2DArray` コールバック)**
   1. 最新の `/localization/pose_enu` を取得できていない場合は警告を出して処理を終了。取得済みの場合はヘッダー時刻差を確認し、3 秒以上ずれていれば警告を出す。
   2. `blocked_positions` と比較し、現在位置がいずれかの確定封鎖地点から `multi_detection_suppression_range` 未満なら、`count_history` へ 0 を push し残処理をスキップ。
   3. 各 `Detection2D` について以下を実施：
      - `results` を走査し、最大スコアクラスを決定。
      - スコアが `score_threshold` 未満の場合は除外。
      - クラス ID が `target_class_id` と一致しない場合は除外。
      - バウンディングボックスの幅・高さ・下端位置を算出し、指定閾値の範囲に入らなければ除外。ただし負値指定の閾値は判定をスキップ。
   4. 条件を満たした検知数を計数し、メッセージ時刻の秒バケットに加算する。新規バケットを生成する際は `decision_duration` 秒より古いバケットを削除する。
   5. 検知結果と判定状態を `PerceptionOverlay` として publish する。

4. **判定ロジック**
   - `count_history` の秒バケット数を分母とし、値が `>=1` のバケット割合を算出。
   - 割合が `decision_frame_ratio` 以上の場合：
     - `temporary_decision_count` を +1。
     - 直前まで 0 だった場合は road_blocked=true を publish、`blocked_state_started_at` を現在時刻でセットし、`confirmation_duration` 計測を開始する。
   - 割合が閾値未満の場合：
     - `temporary_decision_count` を 0 にリセット。
     - 直前まで >0 だった場合は road_blocked=false を publish し、封鎖計測時間をログに出したうえで `blocked_state_started_at` を None に戻す（誤検知として走行再開）。

5. **封鎖確定処理**
   - `temporary_decision_count > 0` かつ `blocked_state_started_at` が設定済みの場合に経過時間をチェック。
   - `now - blocked_state_started_at >= confirmation_duration` なら、最新 `/localization/pose_enu` から取得した pose を `blocked_positions` に追加し、仮判定カウントを 0 にクリア、`blocked_state_started_at` も None に戻す。road_blocked は true のまま維持し、以降は `route_follower` の滞留判定と `route_manager` のリルート処理により走行再開が行われる。

## 9. 認識結果の出力
`/perception/road_blockage/overlay` には以下を載せる。本ノードは画像を扱わない。

- 受信した全 bbox。有効検知として採用したものは `adopted=True`、閾値未満・対象 class 外・
  bbox 条件外で除外したものは `adopted=False` とする。採用外も残すことで、判定に
  使われなかった検知を表示側で描き分けられる。
- `decision`（`0=封鎖なし` / `1=封鎖あり`）と `decision_text`（`clear` / `judging` / `confirmed`）。
- 自己位置未取得や多重検知抑止で判定できなかった場合は `status_note` に `no_pose` /
  `suppressed` を入れる。

描画内容の決定は表示側の責務とする。`robot_console` は採用した検出を太線、採用外を
細線で描き分け、判定結果は画像へ焼き込まずチップとして表示する。

配信は運用確認用であり、制御判断の正本は `/road_blocked` とする。

## 10. ロギング方針
| レベル | タイミング |
| ------ | ---------- |
| info | ノード起動、road_blocked true/false への遷移、封鎖確定時（位置情報含む）。 |
| warn | `/localization/pose_enu` 未取得時、Detection と `/localization/pose_enu` の時刻差が 3 秒以上ある場合、検知メッセージが連続で欠損した場合、判定画像の元画像が長時間未受信の場合。 |
| debug | フィルタ後の検知数、割合計算結果、履歴長などの内部状態（パラメータでオンオフ可）。 |

## 11. エラー／例外ハンドリング
- `Detection2DArray` に要素が無い場合でも `count_history` へ 0 を push し、割合計算を継続する。
- `/localization/pose_enu` が未取得の場合は処理をスキップし、警告ログを出力する。回復後は通常処理へ復帰する。
- Detection のヘッダー時刻と `/localization/pose_enu` のヘッダー時刻の差が大きい場合は警告のみを出し、処理は継続する。
- 判定画像 publish は制御系へ影響させない。画像処理例外は捕捉してログ出力に留める。

## 12. traffic_signal_recognizer との対応関係
`road_blockage_detector` と `traffic_signal_recognizer` は、YOLO 推論層の後段で意味判定を行う対になるパッケージとして扱う。

| 項目 | road_blockage_detector | traffic_signal_recognizer |
| ---- | ---- | ---- |
| YOLO 入力 | `/perception/road_blockage/detections` | `/perception/traffic_signal/detections` |
| raw 画像入力 | `/usb_cam/image_raw` | `/usb_cam/image_raw` |
| 制御出力 | `/road_blocked` | `/sig_recog` |
| 認識結果 | `/perception/road_blockage/overlay` | `/perception/traffic_signal/overlay` |
| YOLO 生検出画像 | `/perception/road_blockage/detection_image` | `/perception/traffic_signal/detection_image` |

`/perception/*/detection_image` は YOLO 推論結果の確認用である。後段判定結果の確認は `/perception/*/overlay` を `robot_console` が生画像へ重畳した表示で行う。

## 13. 今後の検討事項
- road_blocked publish の QoS（信頼性・一回送信）を要件に応じて調整する。
- `blocked_positions` の上限や経過時間による自動削除の必要性を実走テストで検討する。
- `PerceptionOverlay` に載せる項目が robot_console の表示要件を満たすか実機で評価する。
