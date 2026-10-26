"""In-memory log tail for the GUI console.

Two sources feed one ring buffer:

  - the server's own `logging` records (``RingHandler`` on the root logger),
  - stdout/stderr of the test scripts the GUI launches (``pump_process``).

The server polls ``LogRing.since(cursor)`` for each subscribed client and sends
the new lines over the existing WebSocket, so nothing is sent while no panel is
open.  Everything here is thread-safe and never blocks for long: the event loop
only ever takes the lock for a deque slice.

Test-script output is still echoed to the server's stdout, so it keeps reaching
the journal (``journalctl -u x3_server``) exactly as before.
"""

import collections
import logging
import re
import sys
import threading
import time

MAX_LINE_CHARS = 2000      # a runaway repr must not become a 1 MB message

_LEVEL_RE = re.compile(r'\b(CRITICAL|ERROR|WARNING|WARN|INFO|DEBUG)\b')
_ERROR_HINTS = ('Traceback (most recent call last)', 'Error:', 'Exception:')


def guess_level(line):
    """Level of a free-text line from a child process.

    Python's logging format puts the level name in the line, so look for it.
    Tracebacks and bare ``FooError:`` lines count as errors; anything else is
    INFO.
    """
    m = _LEVEL_RE.search(line)
    if m:
        name = m.group(1)
        return 'WARNING' if name == 'WARN' else name
    if any(h in line for h in _ERROR_HINTS) or line.startswith('  File "'):
        return 'ERROR'
    return 'INFO'


class LogRing:
    """Bounded, sequence-numbered log buffer shared by threads."""

    def __init__(self, maxlen=2000):
        self._lines = collections.deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._seq = 0

    def append(self, source, level, msg, t=None):
        msg = str(msg)
        if len(msg) > MAX_LINE_CHARS:
            msg = msg[:MAX_LINE_CHARS] + ' ...[truncated]'
        with self._lock:
            self._seq += 1
            self._lines.append({
                'seq': self._seq,
                't': time.time() if t is None else t,
                'src': source,
                'lvl': level,
                'msg': msg,
            })

    @property
    def last_seq(self):
        with self._lock:
            return self._seq

    def since(self, seq, limit=200):
        """Entries with seq > ``seq``, oldest first, at most ``limit``.

        Returns ``(entries, dropped)``.  ``dropped`` counts lines the caller
        missed: either they fell out of the ring before it asked, or there were
        more than ``limit`` new ones and only the newest ``limit`` are returned.
        """
        with self._lock:
            if not self._lines or self._seq <= seq:
                return [], 0
            oldest = self._lines[0]['seq']
            start = max(seq + 1, oldest)
            new = list(self._lines)[start - oldest:]
        dropped = start - (seq + 1)
        if len(new) > limit:
            dropped += len(new) - limit
            new = new[-limit:]
        return new, dropped

    def tail(self, n):
        with self._lock:
            return list(self._lines)[-n:] if n > 0 else []


class RingHandler(logging.Handler):
    """Copies logging records into a LogRing as source 'server'."""

    def __init__(self, ring, level=logging.INFO):
        super().__init__(level)
        self.ring = ring

    def emit(self, record):
        try:
            msg = record.getMessage()
            if record.exc_info:
                msg += '\n' + logging.Formatter().formatException(record.exc_info)
            self.ring.append('server', record.levelname,
                             f'{record.name}: {msg}', t=record.created)
        except Exception:
            self.handleError(record)


def pump_process(proc, source, ring, echo=None):
    """Drain ``proc.stdout`` into ``ring`` on a daemon thread.

    ``proc`` must be started with ``stdout=PIPE, stderr=STDOUT, text=True``.
    The thread always reads to EOF: if it stopped early the child would block
    on a full pipe, which for a motion test means a robot that stops reacting.
    """
    if echo is None:
        echo = sys.stdout

    def _run():
        ring.append(source, 'INFO', f'--- started (pid {proc.pid}) ---')
        try:
            for raw in proc.stdout:
                line = raw.rstrip('\n')
                if not line:
                    continue
                ring.append(source, guess_level(line), line)
                try:
                    echo.write(f'[{source}] {line}\n')
                    echo.flush()
                except Exception:
                    pass
        except Exception as exc:     # never let a decode error stop the drain
            ring.append(source, 'ERROR', f'log reader failed: {exc}')
        finally:
            code = proc.wait()
            ring.append(source, 'INFO' if code == 0 else 'ERROR',
                        f'--- exited with code {code} ---')

    t = threading.Thread(target=_run, name=f'log_pump_{source}', daemon=True)
    t.start()
    return t
