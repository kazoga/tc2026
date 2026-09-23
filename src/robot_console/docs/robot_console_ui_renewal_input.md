# robot_console UIの構成と責務

操作はPCのPyQt5画面、遠隔端末はHTML観測画面を使用する。
両者は同じSnapshotを参照し、HTMLには走行操作を提供しない。

- [構造とデータフロー](robot_console_gui_architecture_design.md)
- [6タブの画面仕様と操作](robot_console_gui_screen_function_design.md)
- [起動・採取・記録ルート走行](../README.md)
- [GNSS・基地局](gnss_station_ui.md)
- [ROSBAG保存](rosbag_ui.md)
