"""Quantify how much MainThread time a given frame accounts for in a py-spy raw profile."""
import collections
import sys

path = sys.argv[1]
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
needles = sys.argv[3:] or ["_is_standalone_test_running"]

main_total = 0
idle = 0
hits = collections.Counter()

for line in open(path):
    line = line.strip()
    if not line:
        continue
    try:
        stack_str, count = line.rsplit(" ", 1)
        count = int(count)
    except ValueError:
        continue
    if "MainThread" not in stack_str:
        continue
    main_total += count
    if "select (selectors.py" in stack_str:
        idle += count
    for n in needles:
        if n in stack_str:
            hits[n] += count

active = main_total - idle
print(f"MainThread samples: {main_total}  (idle in select: {idle}, active: {active})")
print(f"wall duration: {duration}s -> {duration/main_total*1000:.3f} ms per sample\n")
for n in needles:
    c = hits[n]
    secs = c / main_total * duration
    print(f"{n}:")
    print(f"  {c} samples = {100.0*c/main_total:.1f}% of MainThread, "
          f"{100.0*c/active:.1f}% of its ACTIVE time")
    print(f"  ~{secs:.2f}s of {duration:.0f}s wall")
    print(f"  at 1 scan/s that is ~{secs/duration*1000:.0f} ms of blocking per second\n")
