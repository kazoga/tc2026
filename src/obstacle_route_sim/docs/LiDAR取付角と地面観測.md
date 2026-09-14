# LiDAR取付角と地面観測

MID-360の前下がり角をパラメータ化し、地面を観測できた範囲の判定に反映する。
正の25度が前方へ25度下げた取付、0度が水平、負が前上がりとなる。
既存データの水平取付を維持するため、未指定時の基本値は0度。

## terrain3dでの設定

`tools/terrain3d/terrain3d_v0_4_1.html`のロボット詳細で、以下を設定する。

| 設定欄 | scene.jsonのrobots[].sensors | 25度マウントの例 |
| --- | --- | --- |
| LiDAR前下がり [°] | lidarPitchDownDeg | 25 |
| LiDAR最小仰角 [°] | lidarMinElevationDeg | -7 |
| LiDAR最大仰角 [°] | lidarMaxElevationDeg | 52 |
| センサ高 [m] | height | 0.6（実機で光学原点の高さを測定） |

取付角は±89度以内、センサ固有の仰角は−90〜90度で最小≦最大とする。
仰角の既定値−7〜52度はMID-360の範囲であり、別センサでは変更する。
`pitchDeg`はカメラ用。旧シーンに`lidarPitchDownDeg`がない場合のみ
`-pitchDeg`から初期化し、読み込み後はLiDARとカメラを独立して変更する。

```json
{
  "lidarPitchDownDeg": 25,
  "lidarMinElevationDeg": -7,
  "lidarMaxElevationDeg": 52,
  "height": 0.6,
  "lidarRange": 30
}
```

「観測・再現実験」→「基準LiDARを生成」で点群JSONと地面観測の判定を生成する。
画面には緑（観測支持範囲で参照地形も通行可）、赤（観測地面で参照地形が通行不可）、
灰（未観測または支持範囲の観測不足）の地図を表示する。
JSONの`observedTraversability`に同じマスク、原点、解像度、角度、件数を保存する。
配列は地図+Xを列、+Yを行とするグリッド頂点配置。OccupancyGrid形式ではない。

全周ビームは取付姿勢で剛体回転する。各ビームの仰角に一律25度を足し引きしない。
前下がりでは前方のビームは下へ、後方のビームは上へ向く。水平横向きビームの高さは変わらない。
高さ0.6mの平面上で最下段ビームの前方地面交点は、水平取付で約4.89m、25度取付で約0.96m。
この数値は幾何学上の交点であり、車体の遮蔽、距離下限、路面反射率等を保証しない。

## 判定の意味

`TerrainWorld.traversability`は既知地形・水面・障害物・車体寸法から求める参照通行性で、
センサ角度を変えても変わらない。角度に応じて変化するのは、センサから地面を確認できる範囲。

`TerrainExperiments.observedTraversability`は地面への反射が得られたグリッド点を記録し、
車体の円形包絡が重なる周囲のセルも観測され、参照通行性も満たす場合だけ1を返す。
未観測・欠測・空中通過・無反射は−1の未確認として残す。地面が見えた場合でも
傾斜・水面等で参照判定が不可なら0となる。障害物の背後を地面観測済みにはしない。
地面への一つの反射を車体全体の支持確認としないため、疎なスキャンでは緑が少なくなる。

これは**既知のデジタルツイン地形と単一の理想スキャンによる参照評価**。
点群だけから地面・段差を推定する実測地図生成器や、安全を保証する走行制御ではない。
最近傍グリッドへの丸めを行うため、解像度より小さい障害物は評価できない。
既存の地形PGM出力・経路干渉検査は参照通行性を使い、今回の観測マスクと混同しない。

## Gazebo / FAST-LIO用試験データ

ワークスペースルートでPython仮想環境を有効化してから実行する。

```bash
python src/obstacle_route_sim/tools/build_terrain_trial.py \
  --output /tmp/mid360_down25 --scenario straight --mid360-pitch-deg 25
python src/obstacle_route_sim/tools/prepare_fastlio_trial.py \
  --source /tmp/mid360_down25 --output /tmp/mid360_down25_lio
```

既存のIMU未追加試験を複製して角度だけ変える場合：

```bash
python src/obstacle_route_sim/tools/prepare_fastlio_trial.py \
  --source /path/to/source_trial --output /tmp/down25_lio --mid360-pitch-deg 25
```

生成器の既定値は0度。FAST-LIO試験準備では引数を省略すると元SDFの姿勢を維持する。
LiDARと追加するIMUのposeを一致させ、scene.jsonの該当ロボットにも角度を保存する。
Top-URGの姿勢は独立して維持する。これらのコマンドは試験データの生成で、実機を起動しない。

SDFは前方+X・左+Y・上+Zなので、前下がり25度はpitch=+0.436332313rad。
terrain3dは前方+Yなので横軸回り−25度。符号が違っても物理的な姿勢は同じ。
実機のTF・FAST-LIOの初期姿勢・GNSS融合のセンサオフセットは別途実機構成で整合させる。
本パラメータは実機ドライバ設定、FAST-LIO内部のLiDAR–IMU較正値を自動変更しない。

## 検証

`tests/test_lidar_mount.py`から、前後・横ビーム、ロボットyaw、地面交点の解析解、
0/25度の観測マスク差、無反射の未確認扱い、SDF/IMU姿勢一致、不正角度の拒否を検証する。

MID-360の画角は[メーカーのユーザーマニュアル](../../../docs/references/mid360/Livox_Mid-360_User_Manual_EN.pdf)を参照。
