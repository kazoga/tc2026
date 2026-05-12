# scripts/ ― system 時刻同期サンプル

これらのファイルは **手動配置** が前提のサンプルです。`colcon build` でシステムに
インストールされることはありません。

## 構成 (PPS なし、NMEA only)

UM982 を **シリアル 2 系統に分配** し、片方を `rtk_gps_um982_node`、もう片方を
`gpsd` に渡します。chrony は gpsd の共有メモリから時刻を読み取ります。

```
UM982 ──tty── socat (pty分配) ──┬── /dev/ttyUSB-ROS ──► rtk_gps_um982_node
                                 └── /dev/ttyUSB-GPSD ──► gpsd ──► chrony (SHM 0)
```

または、ROS ノード側を停止しているときだけ chrony 同期を有効化する運用でも可。

## 適用手順 (例)

1. パッケージインストール
   ```bash
   sudo apt install chrony gpsd gpsd-clients
   ```
2. `chrony-gpsd.conf.sample` を `/etc/chrony/chrony.conf` の末尾に追記
3. `/etc/default/gpsd` で `DEVICES=` を設定 (PTY 分配時はそのデバイス名)
4. `sudo systemctl restart gpsd chrony`
5. 同期確認: `chronyc sources` で `#? GPS` が出ること

## 期待精度

- PPS なし: ±30〜50 ms
- 詳細は `../docs/design.md` §13 を参照
