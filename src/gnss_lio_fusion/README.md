# GNSS/LIO 融合

FIX/FLOAT・衛星数・適応baselineでGNSSの重みを決め、FAST-LIOの相対移動と平面EKFで融合する。
FIX/FLOAT 以外の状態では位置・方位を補正しない。方位精度値が残っていても無効観測として扱う。
低信頼時はLIO優先で減速して継続する。実機の校正・完走保証を意味するものではない。

[詳細設計書](docs/詳細設計書.md)に設定、QoS、座標、時刻、異常系、残課題を記載する。
[実装評価](docs/実装評価.md)に閉ループ試験、全周記録再生、成功・未達事項を記載する。

```bash
colcon build --symlink-install --packages-select gnss_lio_fusion
ros2 launch gnss_lio_fusion fusion.launch.py projection_params:=<ENU原点YAML>
```

既存の/localization/pose_enu配信元は停止またはremapして、融合出力と競合させない。
FAST-LIOのOdometryを/lio/odometryへremapする。GPSのfix/statusは同じ観測時刻で配信する。
自律muxの入力cmd_vel/autonomousを/cmd_vel/fusion_limitedへremapすると減速が有効になる。
融合launchは実機ドライバ・自律モードを起動しない。実機確認は別途行う。

検証はobstacle_route_simのevaluate_terrain_trial.pyに--gnss --fusion --fastlioを指定する。
--lio-noise-profile conservative --building-gnssで厳しめLiDAR/IMUと建物近傍FLOATを併用する。
review_fusion_trial.pyは同一融合走行のGPS/LIO観測比較と経路横ずれを出力する。


方位更新は位置観測と分離し、角度の折返し・反転・LIO異常後の再取得を扱う。
[方位実装評価](docs/方位実装評価.md)に修正と再試験を記録する。
実機と模擬環境の共通起動は[icart_bringup](../icart_bringup/README.md)を使う。
前master・後slaveのUM982生方位は後方を向くため、共通起動が180度補正する。
単体launchを使う場合は、受信機側の補正有無と取付方向に合わせて明示設定する。

車輪odomはLIO異常・不整合・途絶時の退避に使用する。通常はLIOを優先する。
退避中もURG停止は維持し、GPS_WHEEL/WHEEL_PRIORITYと退避回数を診断に出す。
車輪スリップを見抜けるという意味ではない。

## 傾斜したMID-360と前後アンテナ配置

`lio_mount_roll_deg/pitch_deg/yaw_deg`はIMUの車体に対する取付角、
`lio_forward_m/left_m/height_m`は車体からIMU原点への位置とする。
LIOから車体の3D姿勢を求め、レバーアームを除いた平面位置を融合する。
`master_forward_m/left_m/height_m`で主アンテナ位置を補正する。
`publish_base_tf`（既定false）をtrueにするとmap→base_link平面TFも配信する。
実機設定はicart_bringupのprepare_real_sessionが一括生成する。
LiDAR内部extrinsicに車体への取付角を足さない。
