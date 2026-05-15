# traffic_signal_recognizer 設計メモ

`traffic_signal_recognizer` は、tc2025 まで ROS 1 ノード
`tc2023_signal_detector.py` が担っていた信号横断可否判定を ROS 2 に移行するための
パッケージである。

YOLO モデルのロードと推論は `yolo_detector` に任せ、本パッケージは
`vision_msgs/msg/Detection2DArray` から GO/STOP を判定する。

外部 interface は route stack との接続契約を優先し、`/recog_flag` と `/sig_recog` を維持する。
判定重畳画像は `/perception/traffic_signal/decision_image` に publish する。
`/recog_flag == 1` の間だけ判定を有効化し、直近 `judge_count` 回の判定が green の場合に
`/sig_recog=1` を publish する。それ以外は `/sig_recog=2` を publish する。

詳細な interface、パラメータ、判定仕様、起動構成は
`traffic_signal_recognizer_詳細設計書.md` を参照する。
