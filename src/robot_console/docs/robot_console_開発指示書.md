# robot_console の開発・確認

画面の追加はui_qt、観測や操作の集約はcore、ROS変換はros、HTTP整形はwebへ配置する。
Coreの値とQt／HTMLの表示をtools/testsで確認し、Qt WebEngineの試験はpytest-forkedで分離する。

- [構造とデータフロー](robot_console_gui_architecture_design.md)
- [6タブの画面仕様と操作](robot_console_gui_screen_function_design.md)
- [起動・採取・記録ルート走行](../README.md)
- [GNSS・基地局](gnss_station_ui.md)
- [ROSBAG保存](rosbag_ui.md)
