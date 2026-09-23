# robot_console の状態表示

route_state・manager_status・follower_stateを受信し、進捗・状態・再計画の表示を構成する。
未受信や期限切れを区別し、表示値だけで走行の許可を判断しない。

- [構造とデータフロー](robot_console_gui_architecture_design.md)
- [6タブの画面仕様と操作](robot_console_gui_screen_function_design.md)
- [起動・採取・記録ルート走行](../README.md)
- [GNSS・基地局](gnss_station_ui.md)
- [ROSBAG保存](rosbag_ui.md)
