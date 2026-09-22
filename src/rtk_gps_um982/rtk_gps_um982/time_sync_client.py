"""固定版UM982Clientの受信箇所に時刻リレーを追加するアダプター。"""

import time

from rtk_gps_um982.ntrip_client import CorrectedUM982Client as UM982Client

from rtk_gps_um982.time_sync_core import RmcClockRelay


class ClockRelayClient(UM982Client):
    """シリアルを二重openせず、解析前のRMCをchronyに渡す。"""

    def __init__(self, *args, clock_relay: RmcClockRelay, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.clock_relay = clock_relay

    def _readline(self) -> str | None:
        line = super()._readline()
        if line:
            self.clock_relay.observe(line, time.time(), time.monotonic())
        return line

    def set_output_rate(self, *args, **kwargs) -> None:
        super().set_output_rate(*args, **kwargs)
        # N4の省略ASCII形式で現在のCOMにRMCを1Hz出力する。
        self._write_line('GPRMC 1')
