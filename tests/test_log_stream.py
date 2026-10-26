import io
import logging
import subprocess
import sys

from log_stream import LogRing, RingHandler, guess_level, pump_process, MAX_LINE_CHARS


def test_since_returns_only_new_lines_in_order():
    ring = LogRing(maxlen=10)
    for i in range(3):
        ring.append('server', 'INFO', f'line {i}')
    lines, dropped = ring.since(1)
    assert [e['msg'] for e in lines] == ['line 1', 'line 2']
    assert dropped == 0
    assert ring.since(ring.last_seq) == ([], 0)


def test_since_reports_lines_that_fell_out_of_the_ring():
    ring = LogRing(maxlen=5)
    for i in range(12):
        ring.append('server', 'INFO', str(i))
    lines, dropped = ring.since(2)       # seqs 3..7 were overwritten
    assert [e['seq'] for e in lines] == [8, 9, 10, 11, 12]
    assert dropped == 5


def test_since_keeps_the_newest_when_over_limit():
    ring = LogRing(maxlen=100)
    for i in range(50):
        ring.append('server', 'INFO', str(i))
    lines, dropped = ring.since(0, limit=10)
    assert [e['seq'] for e in lines] == list(range(41, 51))
    assert dropped == 40


def test_long_lines_are_truncated():
    ring = LogRing()
    ring.append('server', 'INFO', 'x' * (MAX_LINE_CHARS * 3))
    assert len(ring.tail(1)[0]['msg']) < MAX_LINE_CHARS + 50


def test_handler_copies_records_with_level_and_traceback():
    ring = LogRing()
    log = logging.getLogger('test_log_stream.handler')
    log.propagate = False
    log.addHandler(RingHandler(ring))
    log.setLevel(logging.DEBUG)
    log.debug('not kept: handler level is INFO')
    log.warning('careful')
    try:
        raise ValueError('boom')
    except ValueError:
        log.exception('failed')
    entries = ring.tail(10)
    assert [e['lvl'] for e in entries] == ['WARNING', 'ERROR']
    assert entries[0]['src'] == 'server'
    assert 'test_log_stream.handler: careful' == entries[0]['msg']
    assert 'ValueError: boom' in entries[1]['msg']


def test_guess_level():
    assert guess_level('12:00:00 - x - WARNING - slow') == 'WARNING'
    assert guess_level('[WARN] [1700000000.1] [node]: hmm') == 'WARNING'
    assert guess_level('Traceback (most recent call last):') == 'ERROR'
    assert guess_level('  File "/x.py", line 3, in <module>') == 'ERROR'
    assert guess_level('ZeroDivisionError: division by zero') == 'ERROR'
    assert guess_level('goal reached in 4.2 s') == 'INFO'


def test_pump_process_captures_stdout_stderr_and_exit_code():
    ring = LogRing()
    code = ("import sys\n"
            "print('hello')\n"
            "print('bad thing', file=sys.stderr)\n"
            "raise SystemExit(3)\n")
    proc = subprocess.Popen([sys.executable, '-c', code],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, errors='replace')
    echo = io.StringIO()
    pump_process(proc, 'ab_reactive', ring, echo=echo).join(timeout=10)
    msgs = [e['msg'] for e in ring.tail(10)]
    assert msgs[0].startswith('--- started (pid')
    assert 'hello' in msgs and 'bad thing' in msgs
    assert msgs[-1] == '--- exited with code 3 ---'
    assert ring.tail(1)[0]['lvl'] == 'ERROR'
    assert all(e['src'] == 'ab_reactive' for e in ring.tail(10))
    # Still reaches the journal through the server's stdout.
    assert '[ab_reactive] hello' in echo.getvalue()
