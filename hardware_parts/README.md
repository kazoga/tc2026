# ハードウェアパーツ

ロボットに取り付ける部品のCADソース、製造用データ、組立説明を、
ソフトウェアと同じGitリポジトリで管理します。

| 部品 | 内容 | 状態 |
| --- | --- | --- |
| [MID-360 25°マウント](mid360_mount_25deg/README.md) | MISUMI JTABS板とHFS5横梁に対応する印刷ブラケット | 試作。実機適合・強度・放熱は未検証 |

## 配置

```text
hardware_parts/
├── README.md
├── COLCON_IGNORE
└── mid360_mount_25deg/
    ├── README.md           # 寸法、部材、組立・印刷手順、検証状態
    ├── build_model.py      # CAD生成用ソース
    ├── preview_model.py    # 静止画生成
    ├── validate_model.py   # 形状・寸法の検証
    ├── requirements.txt    # この部品のCAD用依存関係
    └── exports/            # STL・STEP・DXF・寸法情報・閲覧用PNG
```

部品が増えたら `hardware_parts/<part_name>/` を追加し、この一覧に登録します。
部品名は用途が分かる英小文字とアンダースコアで統一します。
編集用ソースと製造・閲覧用データを一緒に管理し、キャッシュや仮想環境は含めません。
`COLCON_IGNORE` により、このディレクトリはROSのビルド対象から除外します。

メーカーのマニュアルは [`docs/references/`](../docs/references/README.md) を参照し、
各部品に重複してコピーしません。ROSのセンサ姿勢設定やシミュレーション用モデルは、
それぞれの `src/` 配下のパッケージで管理します。

STL・STEPだけでなく静止画も各部品の説明に掲載します。
PC内のファイルパスはスマートフォン向け共有URLではありません。
