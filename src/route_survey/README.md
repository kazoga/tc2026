# 手動走行による経路採取・編集

既存drive_mode_managerのJoy操縦を使い、融合位置とFAST-LIO点群から経路を採取する。
ノード自身は速度指令を出さない。5 mごと、25度以上の方向変化（最低移動1 m）、
停止ボタンの押下位置にwaypointを登録する。自動幅は確認候補であり、未観測部分は幅0とする。

```bash
ros2 launch route_survey record.launch.py output:=<新規採取ディレクトリ> projection:=<共通projection.yaml> use_sim_time:=true
```

実機はuse_sim_time:=falseとする。既存手動操縦・融合・FAST-LIOを先に起動する。
Joy 0:採取開始、1:停止、2:信号停止、3:採取終了。番号はparams/default.yamlで変更できる。
PS3のL1=4/PS=16による既存走行切替と衝突しない既定値とした。実controllerの割当確認は必要である。
FAST-LIOのpublish.scan_bodyframe_pub_enをtrueにし、body点群とlio/odometryを配信する。

保存先はfixed/waypoints.csv、route_config.yaml、projection.yaml、survey.jsonである。
CSVはroute_planner規定のLLH正本、高度空欄。survey.jsonには幅の観測理由・要確認フラグと
融合位置へ重畳した点群を保存する。1秒ごとに更新し、終了時にも保存する。
出力先は新規のみ許可し、既存経路を上書きしない。

```bash
ros2 run route_survey build_editor.py --survey <採取先>/survey.json --output <採取先>/editor.html
```

デジタルツインの航空写真・地図は--trial <地形ディレクトリ>で追加する。
--map-points <map座標XYZ.npy>で既存FAST-LIO点群も使用できる。原点整合は利用者が確認する。
HTMLは単体・オフライン動作。選択、位置ドラッグ、追加、削除、方位、停止属性、左右幅、
undo/redo、CSV/JSON/PNG保存を備える。編集は走行中routeへ自動適用しない。
編集したCSVをfixed/waypoints.csvへ配置し、走行停止後に経路を読み直す。

PC内HTMLはiPhoneへ自動配信されない。会話内の静止画像・本文を閲覧版とし、
iPhoneで編集する際はアクセス可能な共有先が必要である。外部公開は行わない。
詳細・検証範囲はdocs/設計と検証.mdを参照する。

本番として確認した経路の配置先は`src/route_planner/routes/<コース名>/`とする。
採取・編集の途中結果は別ディレクトリへ保存し、確認後にCSV・route_config.yaml・projection.yamlを
まとめて配置する。survey.jsonも確認履歴の原本として保持する。

最終確認ではGazebo Joy操縦で6点・停止種別各1点を保存した。自動幅は多くが0・要確認であり、
全自動で本番経路を確定できる精度は確認できていない。実機の縁石・センサ姿勢の校正が必要である。

手動操縦中もGNSS＋FAST-LIOの融合位置で採取する。位置分散が0.25 m²を超えても
要確認として記録を続け、自動幅は0にする。融合位置自体が来ない場合は記録できない。
新しいsurvey.jsonにはGNSS単独・融合の比較軌跡を保存する。編集画面で個別表示、
時刻選択、FIX/FLOAT・衛星数・HDOP・基線長・補正経過・方位精度を確認できる。
旧データには受信履歴を補完しない。追加のGNSS欠落試験の範囲は設計と検証を参照。

実機共通sessionの`recorder_params`で、融合と一致するIMU取付位置・角度を読み込む。
body点群を車体へ変換してから車体傾斜を補正する。単独記録の既定値は従来どおり水平・高さ0.6m。
25度配置の実機では生成されたrecorder.yamlを使う共通survey.launch.pyで起動する。
共通採取ではallow_auto_resume=falseによりL1を離しても手動モードを保持する。
