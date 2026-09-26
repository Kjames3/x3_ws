import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from rgbd_pairing import RGBDPairer
from tof_clock import TeensyClock


def packet(seq,ms): return {'meta':{'seq':seq,'device_ns':ms*1000000},'image':None}


def test_nearest_bracketed_pair_not_first_arrival():
    p=RGBDPairer()
    assert p.add('depth',packet(1,90))==[]
    assert p.add('rgb',packet(1,100))==[]
    pairs=p.add('depth',packet(2,105))
    assert len(pairs)==1 and pairs[0]['depth']['meta']['seq']==2
    assert pairs[0]['skew_ns']==5000000
    assert p.add('rgb',packet(2,105))==[]  # depth is not reused


def test_skew_bound_duplicates_reset_and_capacity():
    p=RGBDPairer(capacity=2)
    p.add('depth',packet(1,100))
    assert not p.add('rgb',packet(1,30))
    assert p.counters['rgb_unpaired']==1
    p.add('depth',packet(1,100));assert p.counters['depth_duplicate']==1
    p.add('depth',packet(2,110));p.add('depth',packet(3,120))
    assert len(p.depth)==2 and p.counters['depth_overflow']==1
    p.add('depth',packet(0,1));assert p.epoch==1 and len(p.depth)==1


def test_clock_causal_interval_contains_asymmetric_true_time():
    clock=TeensyClock()
    # True host offset=10s; send 11s, MCU rx 1003 ms, tx 1004 ms,
    # reply reaches host at 11.012s: deliberately asymmetric delays.
    assert clock.request(11000000000)==b's1\n'
    assert clock.observe(dict(token=1,rx_ms=1003,tx_ms=1004),11012000000)
    e=clock.estimate(1020,11030000000)
    lo,hi=e['read_complete_interval_ns']
    assert lo<=11020000000<=hi
    assert e['synchronization_uncertainty_ns']>0
    assert e['acquisition_time_ns'] is None
    assert not clock.estimate(1020,15000000000)['synchronized']


def test_clock_wrap_and_unsolicited_reply():
    clock=TeensyClock()
    assert not clock.observe(dict(token=9,rx_ms=1,tx_ms=2),100)
    clock.request(10000000000)
    assert clock.observe(dict(token=1,rx_ms=0xffffffff,tx_ms=0),10010000000)
    assert clock.estimate(3,10012000000)['synchronized']


def test_depth_history_pairs_delayed_rgb_without_reusing_depth():
    # A 300 ms-delayed RGB packet must still find its 80 Hz depth neighbour.
    pairer = RGBDPairer(capacity=64)
    for seq in range(1, 41):
        pairer.add('depth', {'image': None, 'meta': {'seq': seq, 'device_ns': seq * 12_500_000}})
    pairs = pairer.add('rgb', {'image': None, 'meta': {'seq': 1, 'device_ns': 200_000_000}})
    assert len(pairs) == 1
    assert pairs[0]['depth']['meta']['seq'] == 16
    assert pairs[0]['skew_ns'] == 0
    assert all(p['meta']['seq'] > 16 for p in pairer.depth)
