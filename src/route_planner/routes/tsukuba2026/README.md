# つくば2026 本走行コース

2026年のつくばチャレンジ本走行コースを、全域を固定ブロックとして走る経路である。
選択課題には対応せず、記録した周回をそのまま走る。

- 正本: `fixed/waypoints.csv`。LLHのみを保存し、高度は空欄とする。列は
  [route_planner README](../../README.md) のCSV仕様に従う。
- 経路構成: `route_config.yaml`。固定ブロック `tsukuba2026_full_course` 1つで
  CSV全体を連結する。可変ブロックを持たないため、滞留時の再探索はできない。
- 規模: 3,704点、経路長 約2,241 m。waypoint間隔は最大 0.75 m。
- 停止属性: `line_is_stop` が16点、`signal_is_stop` は全点0である。信号交差点の
  道路端2点（E07、E11）も `line_is_stop` だけを持ち、オペレータが再開する。
- 回避幅: `right_is_open` / `left_is_open` に区間ごとの値が入る。3,464点が
  0より大きい幅を持ち、240点は両側0（横移動を許さない）である。
- 投影: 起動カードは `geo_pose_converter` の共通設定 `params/default.yaml` を指す。
  `icart_bringup` のsessionから起動する場合は、session側の `projection_params` が
  位置推定と経路の双方へ同じ投影条件を与える。

## 由来

`route_survey` の成果物 `src/route_survey/results/tsukuba2026_waypoints.csv` を
そのまま配置したものである。成果物の作り方は
[waypointの回避幅を求める手順](../../../route_survey/docs/waypointの回避幅を求める手順.md)
にある。経路の幾何と停止属性は `src/route_survey/tools/build_route.py` が、回避幅は
`src/route_survey/tools/apply_region_widths.py` が決める。

**停止属性や幾何を変える場合は、この経路CSVを直接編集せず、生成側を直して
`route_survey` の成果物を作り直し、再度配置する。**

## 登録をやり直す

成果物を更新したときは次を実行する。`cmp` と pytest が、配置した経路と
`route_survey` の成果物が同一であることを検査する。

```bash
cp src/route_survey/results/tsukuba2026_waypoints.csv \
   src/route_planner/routes/tsukuba2026/fixed/waypoints.csv
cmp src/route_survey/results/tsukuba2026_waypoints.csv \
    src/route_planner/routes/tsukuba2026/fixed/waypoints.csv
python3 -m pytest src/route_planner/tests/test_tsukuba2026_route.py
```

## 起動する

起動カードは `route_planner` と `route_manager` の双方に同名で置く。`robot_console`
からはこのカード名で選ぶ。`route_manager` の `start_label` と `goal_label` は空であり、
全域を走る。

- `src/route_planner/params/tsukuba2026.yaml`
- `src/route_manager/params/tsukuba2026.yaml`

```bash
colcon build --packages-select route_planner route_manager
source install/setup.bash
ros2 launch route_planner route_planner.launch.py \
  param_file:=$(ros2 pkg prefix route_planner)/share/route_planner/params/tsukuba2026.yaml
ros2 launch route_manager route_manager.launch.py \
  param_file:=$(ros2 pkg prefix route_manager)/share/route_manager/params/tsukuba2026.yaml
```

CSVはビルド時に共有ディレクトリへコピーされる。通常installでCSVを更新した場合は
再ビルドし、ノードを再起動する。

## 確認した範囲

- `route_config.yaml` から3,704点が1ブロックとして組み立てられ、全点が固定区間、
  停止16点、点間隔0.75 m以下であることを `tests/test_tsukuba2026_route.py` で確認した。
- 配置した経路CSVが `route_survey` の成果物とバイト単位で一致することを確認した。

**実機では走行していない。** 回避幅の値、停止位置、信号交差点の扱いは実機走行で
確認していない。模擬環境で確認した範囲は
[回避幅の算出結果](../../../route_survey/results/README.md) にある。
