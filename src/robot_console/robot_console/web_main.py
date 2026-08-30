"""HTML遠隔観測UIのスタンドアロン起動エントリポイント。

Phase7時点ではROS 2ノードとの統合を行わず、既定（空）の`ConsoleSnapshot`を
返すサーバのみを起動する。ROS統合は後続フェーズで `ros/console_node.py` から
本サーバへSnapshot/ImageStoreを供給する構成へ拡張する
（robot_console_gui_architecture_design.md 3章・15章）。
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional

from robot_console.core.snapshot_model import ConsoleSnapshot
from robot_console.web.server import DEFAULT_HOST, DEFAULT_PORT, WebObservationServer


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='robot_console HTML遠隔観測UI（閲覧専用）')
    parser.add_argument(
        '--host', default=DEFAULT_HOST, help=f'bindするホスト（既定: {DEFAULT_HOST}）'
    )
    parser.add_argument(
        '--port', type=int, default=DEFAULT_PORT, help=f'bindするポート（既定: {DEFAULT_PORT}）'
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    """HTML遠隔観測UIサーバを起動し、Ctrl+Cまでブロックする。"""

    args = _parse_args(argv if argv is not None else sys.argv[1:])
    server = WebObservationServer(lambda: ConsoleSnapshot(), host=args.host, port=args.port)
    server.start()
    address = server.address
    if address is not None:
        print(f'robot_console HTML遠隔観測UIを起動しました: http://{address[0]}:{address[1]}/')
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
