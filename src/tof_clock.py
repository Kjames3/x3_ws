"""Causal two-way Teensy millisecond clock intervals, not exposure timestamps.

No symmetric USB latency assumption. Bounds are conditional on no MCU reset
between exchange and frame and the configured relative frequency-drift bound.
Firmware v1 without sync replies remains explicitly unsynchronized.
"""


def signed_ms_delta(a, b):
    return ((a - b + (1 << 31)) & 0xffffffff) - (1 << 31)


class TeensyClock:
    def __init__(self, drift_ppm=1000, max_age_ns=3_000_000_000):
        self.drift_ppm = drift_ppm
        self.max_age_ns = max_age_ns
        self.sample = None
        self.pending = {}
        self.token = 0

    def request(self, send_ns):
        self.token = self.token % 999999999 + 1
        self.pending = {self.token: send_ns}
        return f's{self.token}\n'.encode('ascii')

    def observe(self, record, receive_ns):
        send_ns = self.pending.pop(record['token'], None)
        if send_ns is None or receive_ns < send_ns:
            return False
        dt = signed_ms_delta(record['tx_ms'], record['rx_ms'])
        if dt < 0 or dt * 1_000_000 > receive_ns - send_ns + 1_000_000:
            return False
        # Actual MCU rx lies in [rx_ms, rx_ms+1ms). Host send precedes rx,
        # MCU tx precedes host receipt. Express offset at the tx tick.
        source_rx_ns = (record['tx_ms'] - dt) * 1_000_000
        low = send_ns - source_rx_ns - 1_000_000
        high = receive_ns - record['tx_ms'] * 1_000_000
        if low > high:
            return False
        self.sample = dict(tick=record['tx_ms'], low=low, high=high,
                           receive_ns=receive_ns, rtt_ns=receive_ns-send_ns)
        return True

    def estimate(self, tick, now_ns):
        s = self.sample
        if s is None or not 0 <= now_ns-s['receive_ns'] <= self.max_age_ns:
            return dict(synchronized=False, method='unavailable')
        delta = signed_ms_delta(tick, s['tick'])
        if abs(delta) * 1_000_000 > self.max_age_ns:
            return dict(synchronized=False, method='source_outside_sync_window')
        drift = int(abs(delta) * self.drift_ppm)  # ms * ppm == ns
        source = (s['tick'] + delta) * 1_000_000
        lo = source + s['low'] - drift
        hi = source + 1_000_000 + s['high'] + drift
        return dict(synchronized=True, method='two_way_causal_interval',
                    read_complete_host_monotonic_ns=(lo+hi)//2,
                    read_complete_interval_ns=[lo,hi],
                    synchronization_uncertainty_ns=(hi-lo+1)//2,
                    sync_round_trip_ns=s['rtt_ns'],
                    assumed_relative_drift_ppm=self.drift_ppm,
                    acquisition_time_ns=None,
                    assumption='no_MCU_reset_since_exchange; configured_drift_bound')
