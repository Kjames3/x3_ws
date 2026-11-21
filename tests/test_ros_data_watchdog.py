from ros_data_watchdog import DataWatchdog


class Clock:
    t = 0.0
    def __call__(self):
        return self.t


def test_never_arrived_is_stall_only_after_grace():
    c = Clock(); w = DataWatchdog(['/scan', '/odom'], stall_after=5, startup_grace=30, clock=c)
    c.t = 10; w.beat('/odom')
    assert w.check()['ok'] is None            # /scan still in grace
    c.t = 31; w.beat('/odom')
    st = w.check()
    assert st['ok'] is False and st['stalled'] == ['/scan']


def test_stall_after_flow_and_recovery():
    c = Clock(); w = DataWatchdog(['/scan'], stall_after=5, startup_grace=30, clock=c)
    c.t = 1; w.beat('/scan')
    assert w.check()['ok'] is True
    c.t = 7
    st = w.check(); assert st['ok'] is False and st['stalled_for'] == 0.0
    c.t = 9; assert w.check()['stalled_for'] == 2.0
    w.beat('/scan'); assert w.check()['ok'] is True
