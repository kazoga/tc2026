# Livox SDK2 vendor

`third_party/Livox-SDK2` の固定 submodule revision から SDK とヘッダーを
colcon の install prefix へ配置する。システムへの SDK インストールは不要。
上流ソースとライセンスは submodule 内に保持する。

ワークスペースのルートで実行する。

```bash
git submodule update --init --recursive
colcon build --symlink-install --packages-up-to fast_lio
source install/setup.bash
```

ルートの `colcon.meta` は、上流 `livox_ros_driver2` を変更せずに
`livox_sdk2_vendor` → `livox_ros_driver2` → `fast_lio` のビルド順を指定する。
driver は CMake の prefix 探索で SDK を検出する。SDK 更新時には driver も再ビルドする。
