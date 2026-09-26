# route_planner 詳細設計書

## 役割

YAMLの固定／可変ブロックを連結し、get_routeとupdate_routeへ応答する。
固定ブロックはCSVの点列、可変ブロックはグラフ探索とCSVセグメントを使用する。
走行速度・停止確認・サービス待機の管理は担当しない。

## データと座標

CSVはLLHまたはENUの片方を正本とする。両方が有効な行はLLHを優先し、差を検査する。
LLHからの変換はgeo_pose_converter.geo_coreを使い、Route.projectionへ原点と回転を格納する。
走行用poseのzは0、LLH高度は有効フラグで区別する。
列・許容差・ブロック定義は[READMEのCSV仕様](../README.md#csv-仕様)を参照する。

## 初期経路と更新

get_routeはstart_label、goal_label、checkpoint_labelsを受け、ブロックを連結して範囲を切り出す。
経路のpose、LLH、画像、versionを返す。
CSVはキャッシュされるため、変更の適用は走行停止後にノードを再起動する。

update_routeは世代・index/label・隣接関係・同一可変ブロックを検査する。
閉塞エッジを除いて探索し、現在位置からの仮想区間と後続ブロックを接続する。
固定ブロックは再探索対象外で、失敗応答となる。
plannerのversionはmajorであり、managerの公開major/minor値とは区別する。

## 起動・確認

config_yaml_path、csv_base_dir、projection_config_pathと、必要なら地図画像を指定する。
生成する経路の原点と、走行自己位置の原点を一致させる。
起動例は[README](../README.md)を参照する。
CSV解析・LLH往復・属性保持・グラフ探索・閉塞・ラベル不整合・固定区間失敗を確認する。
