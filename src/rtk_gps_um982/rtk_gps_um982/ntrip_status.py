"""補正データ受信診断。認証情報やサーバ応答本文は公開しない。"""

class NtripStatus:
    def __init__(self):
        self.previous = None

    def sample(self, client, *, enabled, host, port, mountpoint,
               station_id='', station_label='', site='', now):
        total = client.total_bytes if client else 0
        last = client.last_rtcm_monotonic if client else None
        age = max(0., now-last) if last is not None else None
        connected = bool(client and client.transport_connected)
        attempts = client.connection_attempts if client else 0
        rate = 0.
        if self.previous and now > self.previous[0]:
            rate = max(0., total-self.previous[1])/(now-self.previous[0])
        self.previous = now, total
        if not enabled:
            state = 'DISABLED'
        elif not host or not mountpoint:
            state = 'ERROR'
        elif connected:
            state = 'WAITING' if age is None else ('RECEIVING' if age <= 5. else 'STALE')
        else:
            state = 'RECONNECTING' if attempts else 'CONNECTING'
        return dict(state=state, host=host, port=port, mountpoint=mountpoint,
                    station_id=station_id, station_label=station_label, site=site,
                    transport_connected=connected, rtcm_bytes_total=total,
                    rtcm_bytes_per_s=rate, last_rtcm_age_s=age,
                    reconnect_count=max(0, attempts-1),
                    last_error=client.last_error if client else '')
