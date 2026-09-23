# つくば2026デジタルツイン検証用経路

同梱の1,142 waypoint（label 0〜1141）を、route_planner READMEの
CSV仕様に合わせて保存する。現地確認済みの本番経路ではない。
航空写真・公開地図等から構築したデジタルツインの検証用であり、通行可否、
停止点、信号属性は本番走行前に確認する必要がある。

- 正本: `fixed/waypoints.csv`。LLHのみを保存する。高度は空欄。
- 経路構成: `route_config.yaml`。固定ブロックをCSVの行順で連結する。
- 投影設定: `projection.yaml`。実機とシミュレータで同じ設定を用いる。
- heading_deg: 真北0度、時計回り。ENU quaternionは起動時に生成する。
- 回避幅・停止・スキップ属性は検証用の値であり、現地確認を要する。

共通sessionのroute_configとcsv_base_dirを本ディレクトリの設定・所在へ向け、
projection_paramsを同梱projection.yamlへ向ける。パスはsession.yamlからの相対とする。
start_labelは'0'、goal_labelは'1141'とする。
