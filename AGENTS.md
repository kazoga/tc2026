# AGENTS.md

このファイルは、Codex がこの ROS 2 ワークスペースで作業する際の共通指示である。
GitHub リポジトリに登録し、開発メンバ間で共通利用することを前提とする。

---

# Common Instructions

- 回答、作業計画、作業報告、警告、補足説明は日本語で行う。
- 技術用語、API 名、ROS メッセージ名、クラス名、関数名は英語表記を維持してよい。
- 実装や修正にあたって、省略、要約、抽象化で必要な処理を落とさない。
- 変更前に、対象ファイル、変更方針、確認方法を簡潔に説明する。
- 機能追加・修正時は、既存のディレクトリ構成、命名規則、コメント粒度、docstring 形式、ログ出力方針と整合させる。
- 関係ないコメント、空行、フォーマット、命名を変更しない。
- リポジトリ外のファイルやディレクトリを前提にした実装、設定、ドキュメントを追加しない。
- 絶対パスは記述しない。必要な場合は、リポジトリルートからの相対パスまたはプレースホルダを使う。
- `build/`, `install/`, `log/`, `.git/` は編集しない。ただし `colcon build` の再確認に
  必要なクリーンとして、`build/`, `install/`, `log/` 配下の生成物を削除することは許容する。
- `sudo`、ファイル削除、破壊的操作は明示指示がある場合のみ行う。
- 生成物、キャッシュ、一時ファイルをリポジトリ管理対象として追加しない。

---

# Workspace Structure Instructions

このワークスペースは、ROS 2 の colcon ワークスペースとして扱う。

```text
.
├── AGENTS.md
├── README.md
├── requirements.txt
├── docs/
├── src/
├── build/
├── install/
└── log/
```

- `src/` 配下に ROS 2 パッケージを配置する。
- `src/<package_name>/` を 1 つの ROS 2 パッケージの基本単位として扱う。
- `requirements.txt` はワークスペース共通の Python pip 依存関係を管理するファイルとして扱う。
- ワークスペース全体に関わる資料は `docs/` 配下に置く。
- 特定パッケージに閉じる資料は `src/<package_name>/docs/` 配下に置く。
- `build/`, `install/`, `log/` は colcon の生成物として扱い、直接修正しない。ただし
  ビルド確認のためのクリーンでは削除してよい。
- 新規ファイルを追加する場合は、このファイルの推奨構成に従い、配置理由を作業報告に含める。

---

# ROS 2 Package Structure Instructions

## Recommended Package Layout

パッケージ内のファイル・ディレクトリは、以下の推奨構成に合わせる。
現在存在するパッケージ名や個別構成を前提にせず、役割に応じて必要なものだけを置く。

```text
src/<package_name>/
├── package.xml
├── setup.py
├── setup.cfg
├── CMakeLists.txt
├── resource/
│   └── <package_name>
├── <package_name>/
│   ├── __init__.py
│   ├── <node>_node.py
│   └── *_core.py
├── src/
├── launch/
│   └── <name>.launch.py
├── params/
│   └── <name>.yaml
├── msg/
│   └── <Name>.msg
├── srv/
│   └── <Name>.srv
├── docs/
├── tests/
│   └── test_*.py
├── tools/
├── routes/
├── maps/
├── models/
├── rviz/
└── third_party/
```

## Required Items

- `package.xml` はすべての ROS 2 パッケージで必須とする。
- `ament_python` パッケージでは、`setup.py` と `resource/<package_name>` を必須とする。
- `ament_cmake` パッケージ、C++ 実装を含むパッケージ、`msg` / `srv` を定義するパッケージでは、`CMakeLists.txt` を必須とする。
- Python モジュールを持つパッケージでは、`<package_name>/` と `<package_name>/__init__.py` を配置する。
- C++ 実装を持つパッケージでは、実装ファイルを `src/` 配下に配置する。

## Conditional Items

- `setup.cfg` は `ament_python` パッケージで使用することを推奨する。
- `launch/` は launch ファイルを提供する場合に配置する。
- `params/` は ROS 2 パラメータ YAML を提供する場合に配置する。
- `msg/` は独自メッセージを定義する場合に配置する。
- `srv/` は独自サービスを定義する場合に配置する。
- `docs/` はパッケージ固有の設計書、検討記録、仕様変更メモを置く場合に配置する。
- `tests/` は pytest ベースのテストを置く場合に配置する。
- `tools/` は開発、検証、可視化、変換用の補助スクリプトを置く場合に配置する。
- `routes/` は経路データを置く場合に配置する。
- `maps/` は地図データを置く場合に配置する。
- `models/` は学習済みモデルや推論用モデルを置く場合に配置する。
- `rviz/` は RViz 設定を置く場合に配置する。
- `third_party/` は外部コードや外部ライブラリを同梱する場合に配置する。

## Protected and Generated Items

- `__pycache__/`, `.pytest_cache/`, `*.egg-info/` は生成物として扱い、追加・編集しない。
- `third_party/` 配下は原則として編集しない。変更が必要な場合は、影響範囲と理由を明確にする。
- `routes/`, `maps/`, `models/` はデータ領域として扱い、明示指示がある場合を除き編集しない。

## Directory Roles

- `package.xml`, `setup.py`, `setup.cfg`, `CMakeLists.txt` はパッケージ定義、依存関係、エントリポイント、ビルド設定を管理する。
- `resource/<package_name>` は `ament_python` のパッケージ登録用ファイルとして維持する。
- `launch/` には launch 単位の起動構成を置く。
- `params/` には ROS 2 パラメータ YAML を置く。
- Python パッケージディレクトリには、ノード、ROS 非依存コアロジック、内部モジュールを置く。
- `*_core.py` には、可能な限り ROS 非依存の処理、状態管理、計算ロジックを置く。
- `<node>_node.py` には、ROS 通信、パラメータ、QoS、ログ、タイマー、publisher/subscriber/service/action などを置く。
- `src/` には C++ の実装ファイルを置く。
- `msg/`, `srv/` には ROS interface 定義を置く。
- `docs/` には設計書、検討記録、仕様変更メモを置く。
- `tests/` には pytest 互換のテストを置く。
- `tools/` には開発・検証・可視化・変換用の補助スクリプトを置く。

## Naming Rules

- ノードファイル名は `<name>_node.py` を基本とする。
- コア処理ファイル名は機能名に対応した `*_core.py` を基本とする。
- launch ファイル名は `<name>.launch.py` とする。
- パラメータファイル名は内容を表す `<name>.yaml` とする。
- テストファイル名は `test_*.py` とする。
- ROS トピック名は lower_case とスラッシュ区切りを基本とする。
- 既存の公開 API 名や ROS interface 名は不用意に変更しない。
- 既存ファイルの命名規則がある場合は、既存規則との整合を優先する。

## Development Policy

- パッケージ構成を不用意に崩さない。
- コード変更と設計書・検討記録の整合を保つ。
- 依存関係を追加する場合は、`package.xml`, `setup.py`, `CMakeLists.txt` の必要箇所を確認する。
- pip install が必要な Python モジュールを追加する場合は、ワークスペース直下の `requirements.txt` にも記述する。
- ROS 非依存で表現できるロジックは、ノード本体ではなく `*_core.py` などの独立したモジュールに寄せる。
- ノード本体は ROS 入出力とライフサイクル管理を中心に薄く保つ。
- 新規パッケージを追加する場合は、推奨構成に従う。

---

# Python Instructions

## Style

- Google Python Style Guide を基本にする。
- PEP 8 に従う。
- 1 行は原則 100 文字以内にする。
- クラス間は空行 2 行、関数間は空行 1 行を基本とする。
- 既存ファイルの書式がある場合は、既存書式との整合を優先する。

## Type Hints

- すべての関数・メソッドに可能な限り型ヒントを付ける。
- 戻り値がない場合は `-> None` を明記する。
- 型が複雑になる場合は、読みやすい型定義や補助型を検討する。

## Naming

- クラス名は `PascalCase` とする。
- 関数名、メソッド名、変数名は `snake_case` とする。
- 定数は `UPPER_CASE` とする。
- 既存の公開 API 名や ROS interface 名は不用意に変更しない。

## Comments and Docstrings

- コメント、docstring、ログメッセージは日本語を優先する。
- 英語は固有名詞、外部 API、ROS メッセージ名など技術的に必要な箇所に限定する。
- コメントは、処理の目的、背景、例外条件、アルゴリズム選定理由が分かる粒度で記述する。
- 自明な処理をなぞるだけのコメントは追加しない。
- 既存の日本語コメント・docstring を不要に書き換えない。

Docstring は以下の形式を基本とする。

```python
def compute_distance(self, pose_a: Pose, pose_b: Pose) -> float:
    """2 点間のユークリッド距離を算出する.

    Args:
        pose_a (Pose): 始点の位置姿勢.
        pose_b (Pose): 終点の位置姿勢.

    Returns:
        float: pose_a から pose_b までの距離 [m].
    """
```

## Imports

import 順序は以下を基本とする。

1. 標準ライブラリ
2. 外部ライブラリ
3. ROS 2 関連ライブラリ
4. 同一パッケージ内モジュール

```python
import math

import numpy as np

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from <package_name>.<module_name> import SomeClass
```

## ROS 2 Python Node Policy

- ROS ノードでは `self.get_logger()` を使う。
- ノードは、起動、主要状態遷移、終了時に必要なログを出す。
- 内部状態確認は `debug` を使う。
- 通常の進行報告は `info` を使う。
- 想定内の異常や再試行は `warn` を使う。
- 明確な障害は `error` を使う。
- 致命的停止は `fatal` を使う。
- QoS、パラメータ、トピック名、タイマー周期は、既存設計または関連ドキュメントと整合させる。

---

# C++ Instructions

- C++ 実装では、既存の CMake 設定、依存関係、命名規則に従う。
- C++ ソースはパッケージ内の `src/` 配下に配置する。
- 依存関係を追加する場合は、`package.xml` と `CMakeLists.txt` の両方を確認する。
- ROS 2 の publisher/subscriber/service/action、パラメータ、QoS は関連ドキュメントと整合させる。

---

# Build Instructions

- このワークスペースは ROS 2 Jazzy を前提とする。
- ビルド前に、各自の開発環境に合わせて ROS 2 環境を有効化する。
- 開発時の基本ビルドは以下を使う。

```bash
colcon build --symlink-install
```

- 変更対象が明確な場合は、まず対象パッケージだけをビルドする。

```bash
colcon build --symlink-install --packages-select <package_name>
```

- ビルド確認では、`--symlink-install` 付きと無しの両方を確認する。

```bash
colcon build --symlink-install --packages-select <package_name>
colcon build --packages-select <package_name>
```

- `--symlink-install` 付きと無しを切り替えて再ビルドする場合、既存生成物により
  symlink 作成や通常 install が競合することがある。その場合は、確認対象パッケージに
  対応する `build/<package_name>/`, `install/<package_name>/`, `log/` 配下の生成物を
  クリーンしてから再実行してよい。
- 複数パッケージやワークスペース全体の確認でパッケージ単位のクリーンでは解消できない場合は、
  `build/`, `install/`, `log/` を colcon 生成物として削除してから再実行してよい。
- クリーンを行った場合は、削除対象と理由を作業報告に明記する。

- ワークスペース全体の確認が必要な場合も、`--symlink-install` 付きと無しの両方を確認する。

```bash
colcon build --symlink-install
colcon build
```

- ビルド失敗時は、最初の本質的なエラーを特定してから修正する。
- エラー出力を要約だけで判断せず、該当ファイル、行番号、依存関係、entry point、import error を確認する。
- `build/`, `install/`, `log/` の生成物を直接修正しない。ビルド確認のために
  クリーンした場合も、生成物の手修正は行わない。
- 確認結果を報告する場合は、実行したコマンド、成功/失敗、失敗時の原因、未確認事項を明記する。

---

# Run Instructions

- Codex は原則として `ros2 run`, `ros2 launch`, `ros2 topic`, `ros2 service`, `ros2 action`, `ros2 interface` などの ROS 2 実行確認を行わない。
- 実機、センサ、GUI、Gazebo、外部デバイス、長時間起動ノードに依存する確認も原則として行わない。
- 必要な動作確認が `colcon build` と pytest で代替できない場合は、未確認事項として作業報告に明記する。
- README やドキュメントに利用者向けの実行手順を書く場合は、リポジトリ外の絶対パスを使わない。

---

# Test Instructions

- テストは `tests/test_*.py` に配置する。
- `pytest` 互換の構文を使う。
- 既存テストの意図を尊重する。
- 失敗テストを単に削除・緩和しない。
- まず `*_core.py` など ROS 非依存ロジックの単体テストを優先する。
- テストでは、境界値、異常系、状態遷移、パラメータ差分を確認する。
- 外部環境に依存するテストは、実行できない環境で失敗し続けないよう skip 条件を検討する。
- テスト追加後は、可能な範囲で以下を実行する。

```bash
pytest
```

- 対象パッケージが明確な場合は、以下を使う。

```bash
pytest src/<package_name>/tests
```

- テスト結果を報告する場合は、実行コマンド、成功/失敗、失敗時の原因、未確認範囲を明記する。

---

# Documentation Instructions

## Language

- 設計書、検討記録、README 補足、調査レポートは原則日本語で記述する。
- 設計書本文は常体「する」「を行う」を基本とする。
- チャット向け説明は丁寧語でよい。
- 英語の外部仕様、API 名、ROS メッセージ名は保持してよい。
- 英語の原文を引用・保持する場合は、必要に応じて日本語補足を併記する。

## Placement

- ワークスペース全体に関わる資料は `docs/` 配下に保存する。
- 特定パッケージに閉じる資料は `src/<package_name>/docs/` 配下に保存する。
- 一時的な調査メモであっても、後から参照する可能性がある場合は `docs/` 配下に保存する。

## Consistency

- コード変更により仕様、トピック、パラメータ、状態遷移、QoS、launch 引数、ファイル構成が変わる場合は、関連ドキュメントの更新を検討する。
- 既存の章立て、phase 表記、用語、粒度を尊重する。
- コードとドキュメントが矛盾している場合は、どちらを正とするか確認し、作業報告に明記する。
- 仕様変更を伴う場合は、変更理由、影響範囲、確認方法を記録する。

## Detailed Design Documents

パッケージ固有の詳細設計書を新規作成または更新する場合は、既存の
`src/<package_name>/docs/*詳細設計書*.md`、`design.md`、検討記録、README を確認し、
そのパッケージの phase 表記、章番号、用語、表形式、粒度に合わせる。
既存文書に章立てがある場合は無理に全面改稿せず、不足章を追加または既存章へ追記する。

詳細設計書には、対象パッケージの性質に応じて以下の章を設けることを基本とする。
該当しない章は「該当なし」と明記するか、既存文書の粒度に合わせて省略理由を記載する。

1. 文書目的・対象範囲
   - 設計書の目的、対象パッケージ、対象ノード、対象 phase、実装済み／設計中／将来予定の区別を記載する。
   - 本文書で確定する仕様と、検討記録・README・上位アーキテクチャ文書との関係を明記する。
   - 実機、シミュレーション、GUI、センサ、外部デバイスなど前提環境がある場合は明記する。

2. 背景・要求・スコープ
   - パッケージを追加・変更する背景、解決する課題、上位システム内での位置づけを記載する。
   - 責務、スコープ、非スコープ、将来 phase に送る内容を分けて書く。
   - 既存パッケージとの分担や、他パッケージへ移譲すべき処理を明確にする。

3. 全体構成・アーキテクチャ
   - ノード、コアモジュール、補助ツール、launch、params、msg、srv、データファイルの関係を記載する。
   - ROS 依存処理と ROS 非依存ロジックの分担を示し、`*_core.py` などへ切り出す範囲を明記する。
   - 複数プロセス、スレッド、timer、callback、GUI mainloop、外部プロセスを持つ場合は構成と責務を記載する。

4. パッケージ構成・ファイル配置
   - 追加・変更する主なファイル、ディレクトリ、配置理由を記載する。
   - `launch/`, `params/`, `config/`, `routes/`, `maps/`, `models/`, `rviz/`, `tools/`, `docs/` などを使う場合は用途を明記する。
   - 生成物、外部モデル、third_party、実機固有ファイルを扱う場合は、管理対象と非管理対象の境界を記載する。

5. 外部インタフェース仕様
   - Publisher、Subscriber、Service、Action、TF、parameter event、外部プロセス入出力を表で整理する。
   - topic/service/action 名、型、方向、QoS、発行・購読条件、更新周期、初期値、停止時の扱いを記載する。
   - 独自 `msg` / `srv` を使う場合は、各 field の意味、単位、許容範囲、互換性への注意点を記載する。
   - topic remap や namespace を前提にする場合は、既定名と変更方法を記載する。

6. パラメータ・設定仕様
   - パラメータ名、型、既定値、単位、許容値、制約、使用箇所、変更時の反映タイミングを表で整理する。
   - launch 引数、YAML、環境依存設定、モデルファイル、地図、経路 CSV などとの対応を記載する。
   - パラメータを追加する場合は、`params/*.yaml`、launch、README、テストの更新要否を確認する。

7. データモデル・内部状態
   - 内部で保持する状態、dataclass、構造体、キャッシュ、履歴、FSM 状態、GUI snapshot などを記載する。
   - 状態の初期値、更新契機、寿命、クリア条件、排他制御、古いデータの扱いを明記する。
   - 座標系、frame_id、時刻、単位、信頼度、ステータス値などの解釈を明記する。

8. 処理フロー・状態遷移
   - 初期化、通常処理、callback、timer、service 処理、終了処理、再起動時復帰の流れを記載する。
   - FSM や phase を持つ場合は、状態一覧、遷移条件、遷移時に publish する値、ログ、エラー復帰を記載する。
   - ノード間連携が重要な場合は、テキストシーケンスまたは表で publish / subscribe / service 呼び出し順を示す。

9. 主要アルゴリズム・判定ロジック
   - 経路生成、追従、障害物判定、信号認識、画像処理、座標変換、速度制御などの計算手順を記載する。
   - 入力、出力、前提条件、閾値、境界値、丸め、フィルタ、デバウンス、タイムアウトを明記する。
   - 実装上の関数・クラス名を示す場合は、公開 API と内部実装を区別する。

10. QoS・並行性・タイミング設計
    - QoS の reliability、durability、history、depth、deadline、liveliness を必要範囲で記載する。
    - timer 周期、publish 周期、入力 timeout、再送周期、GUI 更新周期、外部プロセス監視周期を記載する。
    - callback group、スレッド、ロック、キュー、GUI スレッド連携など並行性の注意点を記載する。

11. 起動・終了・launch 設計
    - `ros2 launch`、entry point、node name、namespace、remap、parameter file、simulator 同時起動の扱いを記載する。
    - 起動時のパラメータ検証、外部デバイス接続、モデル読み込み、初期 publish の有無を明記する。
    - 終了時の停止指令、process cleanup、resource 解放、最後に publish する安全値を記載する。

12. エラー処理・ログ・診断
    - 入力欠落、型不整合、ファイル不在、モデル読み込み失敗、外部プロセス失敗、タイムアウト時の挙動を記載する。
    - `debug`、`info`、`warn`、`error`、`fatal` の使い分け、スロットル、重複抑制、運用時に見るべきログを記載する。
    - 異常時に publish を止めるのか、安全値を出すのか、直前値を保持するのかを明記する。

13. UI・可視化仕様
    - GUI、RViz、画像出力、地図表示、viewer、console log 表示を持つ場合に記載する。
    - 画面構成、操作、入力制約、表示更新、topic との対応、ユーザー操作が publish/service へ変換される流れを記載する。
    - 表示専用情報と制御に影響する入力を区別し、誤操作時の安全策を記載する。

14. 依存関係・ビルド設定
    - `package.xml`、`setup.py`、`CMakeLists.txt`、`requirements.txt` に必要な依存関係を整理する。
    - ROS 2 Jazzy、Python、C++、外部ライブラリ、submodule、モデル形式、実機デバイスの前提を記載する。
    - 依存関係を追加・変更する場合は、ビルド設定と README の更新要否を明記する。

15. テスト計画・受け入れ条件
    - 単体テスト、ROS 非依存コアテスト、launch/ビルド確認、異常系、境界値、状態遷移、パラメータ差分の観点を記載する。
    - 実機、センサ、GUI、Gazebo、外部デバイスが必要で自動確認できない項目は、未確認事項として分ける。
    - Definition of Done を置く場合は、成功条件、確認コマンド、残リスクをチェックリストで記載する。

16. 互換性・移行・影響範囲
    - 既存 topic、service、msg/srv field、パラメータ、launch 引数、ファイル配置への互換性影響を記載する。
    - downstream / upstream パッケージ、README、検討記録、上位アーキテクチャ文書への影響を整理する。
    - 仕様変更時は、旧仕様からの移行方法、同時に更新すべきパッケージ、暫定対応を記載する。

17. 未決事項・今後の拡張
    - 未確定仕様、保留した設計判断、将来 phase の候補、確認待ち事項を箇条書きで記載する。
    - 未決事項には、誰が・何を確認すれば確定できるか、実装を止める blocker かどうかを可能な範囲で書く。

18. 改版履歴
    - 文書が継続的に更新される場合は、日付、版、変更概要、関連 issue / PR / phase を記載する。
    - 実装完了後に設計書を更新した場合は、実装との差分と確認結果を記録する。

詳細設計書を更新した作業報告では、参照した既存文書、追加・更新した章、コードとの整合確認方法、
ビルドまたは pytest を実行しなかった場合の理由を明記する。
