"""Summarise a py-spy `--format raw` folded-stack profile.

Usage: python3 analyze_pyspy_raw.py prof_raw.txt [thread_substring]

py-spy raw lines are `frame;frame;...;frame COUNT`, where the first frame carries
the process/thread label. With `--idle` the sample set includes threads blocked in
syscalls, which is exactly what you need to find something that BLOCKS the event
loop rather than something that merely burns CPU.
"""
import collections
import sys


def parse(path):
    per_thread = collections.Counter()
    stacks = collections.defaultdict(collections.Counter)
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            stack_str, count = line.rsplit(" ", 1)
            count = int(count)
        except ValueError:
            continue
        frames = stack_str.split(";")
        thread = frames[0]
        per_thread[thread] += count
        stacks[thread][tuple(frames[1:])] += count
    return per_thread, stacks


def main():
    path = sys.argv[1]
    want = sys.argv[2] if len(sys.argv) > 2 else None
    per_thread, stacks = parse(path)
    total = sum(per_thread.values())
    print(f"total samples: {total}\n")
    print("=== samples per thread ===")
    for th, c in per_thread.most_common(15):
        print(f"{c:8d}  {100.0*c/total:5.1f}%  {th}")

    targets = [t for t in per_thread if (want is None or want in t)]
    if want:
        targets = [t for t in targets if want in t]
    for th in sorted(targets, key=lambda t: -per_thread[t])[:2]:
        tt = per_thread[th]
        print(f"\n=== {th}: top leaf frames ({tt} samples) ===")
        leaves = collections.Counter()
        for frames, c in stacks[th].items():
            if frames:
                leaves[frames[-1]] += c
        for leaf, c in leaves.most_common(20):
            print(f"{c:8d}  {100.0*c/tt:5.1f}%  {leaf}")

        print(f"\n=== {th}: top full stacks ===")
        for frames, c in stacks[th].most_common(8):
            print(f"\n  {c} samples ({100.0*c/tt:.1f}%)")
            for f in frames[-14:]:
                print(f"      {f}")


if __name__ == "__main__":
    main()
