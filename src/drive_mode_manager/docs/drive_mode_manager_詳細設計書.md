# drive_mode_manager 詳細設計書

## 構成と責務

manual_teleop_nodeはJoyから手動Twistを作り、drive_cmd_mux_nodeが自律／手動を選択して
最終cmd_velを配信する。ROS非依存の判定はmanual_teleop_core.pyとdrive_mode_core.pyに置く。
drive_status_gui_nodeはDriveModeStatusを表示する。物理非常停止回路の代替ではない。

```text
/joy → manual_teleop → /cmd_vel/manual ────┐
/cmd_vel/autonomous ─────────────────────┴→ drive_cmd_mux → /cmd_vel
```

共通融合起動ではmuxの自律入力を `/cmd_vel/fusion_limited` にremapする。
最終cmd_velの配信元をmux一つにする。手動指令は自律側の融合減速・障害物監視を通らない。

## 状態・遷移

状態はAUTONOMOUS／MANUAL、出力元はZERO／AUTONOMOUS_CMD／MANUAL_CMDを別々に持つ。
AUTONOMOUSでも、入力途絶や復帰待ちでは出力がZEROになる。

- 起動時はinitial_modeで選ぶ。単体既定はautonomous、採取起動はmanual。
- L1＋PSを2秒保持すると手動へ移る。L1だけでは自律から切り替わらない。
- 手動中は新鮮なJoyとL1保持、新鮮な手動指令が揃う場合に出力する。
- L1を離して1秒経過し、新鮮な自律指令があれば復帰待ちへ入る。5秒間ゼロ指令後に自律へ戻る。
- allow_auto_resume=falseではL1解放による自律復帰を行わない。共通採取起動がこの設定を使う。

## 入出力

Joyはmanual_teleopとmuxが購読する。
手動・自律・最終cmdはgeometry_msgs/Twist、状態はtc_route_msgs/DriveModeStatusを使う。
TwistはRELIABLE / VOLATILE / depth 1、Joy・状態はdepth 10で扱う。
状態には入力鮮度、L1/PS、長押し進捗、復帰待ち時間、予定指令、実出力を含む。
正確なフィールドは[DriveModeStatus.msg](../../tc_route_msgs/msg/DriveModeStatus.msg)を参照する。

## 設定

| 項目 | 単体既定 | 用途 |
| --- | --- | --- |
| linear_axis / angular_axis | 1 / 0 | 前後／旋回 |
| linear_scale / angular_scale | 1.2 / 1.5 | 手動入力倍率、m/s・rad/s |
| deadzone / in_place_turn_deadzone | 0.05 / 0.12 | 通常の中立域／旋回中の前後軸微小入力除去 |
| enable_button / l1_button_index | 4 | L1 |
| ps_button_index | 16 | 単体・Joy simulator。共通実機SDL設定は10 |
| turbo_button | −1 | 無効。R1=5は融合へのGNSS途絶模擬に使用 |
| joy_timeout_s | 0.5秒 | Joyの期限 |
| autonomous_cmd_timeout_s | 0.5秒 | 自律指令の期限 |
| manual_cmd_timeout_s | 0.3秒 | 手動指令の期限 |
| publish_rate_hz | 20 Hz | 指令・状態配信 |

軸不足・非有限値・L1解放・Joy途絶時は手動Twistをゼロにする。
入力倍率は車輪の実効上限ではない。ドライバが速度・加減速度を制限する。
設定一覧は[params/default.yaml](../params/default.yaml)、
実機操作は[ハードウェア統合](../../icart_bringup/docs/実機ハードウェア統合.md)を参照する。

## 起動・確認

[README](../README.md)のlaunchでjoy_node・teleop・mux・任意の状態GUIを起動する。
joy_inputはjoy_node、ps3_joy_sim、externalを選び、配信元を一つにする。
状態GUIは表示専用である。キー入力でJoyを作る画面は[Joy simulator](ps3_joy_sim_設計書.md)を使う。

testsで長押し、期限切れ、復帰待ちの中断、手動固定、微小入力、非有限値を確認する。
模擬Joyでの成立と、実コントローラの番号・車体の向き・実制動性能は分けて確認する。
