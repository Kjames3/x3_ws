"""Find what blocks the asyncio event loop.

A coroutine stamps a heartbeat every 10 ms. A watcher thread notices when the
stamp is older than ``threshold_s`` and logs the loop thread's stack, so a
periodic stall names its own culprit. Cheap enough to leave on.
"""
import asyncio
import logging
import sys
import threading
import time
import traceback

logger = logging.getLogger(__name__)


def start(loop_thread_id, threshold_s=0.15, log_every_s=0.0):
    state = {'beat': time.monotonic(), 'reported': 0.0, 'n': 0}

    async def beat():
        while True:
            state['beat'] = time.monotonic()
            await asyncio.sleep(0.01)

    def watch():
        while True:
            time.sleep(0.05)
            now = time.monotonic()
            late = now - state['beat']
            if late > threshold_s and state['beat'] != state['reported']:
                state['reported'] = state['beat']
                state['n'] += 1
                frame = sys._current_frames().get(loop_thread_id)
                stack = ''.join(traceback.format_stack(frame, limit=6)) if frame else '?'
                logger.warning("event loop blocked %.2f s (stall #%d), loop thread at:\n%s",
                               late, state['n'], stack)

    threading.Thread(target=watch, name='loop_stall_watch', daemon=True).start()
    return beat()
