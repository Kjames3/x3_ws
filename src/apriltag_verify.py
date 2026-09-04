#!/usr/bin/env python3
"""Identify the AprilTags physically mounted in the apartment.

Laptop-only: takes ordinary photos (phone is fine) and reports which family
and ID each tag actually is. Answers the question the print-scale forensic
could not -- the mounted tags do not match any local source file, so the
family on the wall is unverified.

    python3 src/apriltag_verify.py ~/tag_photos/*.jpg

Photograph one tag per frame, roughly square-on, tag filling ~1/3 of frame.
"""
import sys, glob
import cv2
from pupil_apriltags import Detector

# tag25h9 is in ~/Downloads and is a plausible source for the mounted tags.
# It has only 35 codes and is false-positive prone -- if any wall tag comes
# back as this family, it must be replaced, not just resized.
FAMILIES = ["tag36h11", "tag25h9", "tag16h5"]


def main(paths):
    if not paths:
        print(__doc__)
        return 1
    dets = {f: Detector(families=f, quad_decimate=1.0) for f in FAMILIES}
    found = {}
    for path in paths:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"{path}: unreadable")
            continue
        hits = []
        for fam, det in dets.items():
            for r in det.detect(img):
                px = max(r.corners.max(0) - r.corners.min(0))
                hits.append((fam, r.tag_id, r.hamming, r.decision_margin, px))
        if not hits:
            print(f"{path}: NO TAG FOUND")
            continue
        for fam, tid, ham, margin, px in hits:
            flag = "" if fam == "tag36h11" and ham == 0 else "   <-- CHECK"
            print(f"{path}: {fam} id={tid} hamming={ham} "
                  f"margin={margin:.1f} size={px:.0f}px{flag}")
            found.setdefault((fam, tid), []).append(path)

    print(f"\n{len(found)} distinct tags across {len(paths)} photos")
    dupes = {k: v for k, v in found.items() if len(v) > 1}
    if dupes:
        print("Repeated IDs (expected if you shot a tag twice, a problem if not):")
        for (fam, tid), files in dupes.items():
            print(f"  {fam} id={tid}: {len(files)} photos")
    bad = [k for k in found if k[0] != "tag36h11"]
    if bad:
        print(f"\nWRONG FAMILY on {len(bad)} tag(s): {bad}")
        print("Replace these -- low Hamming distance means false detections.")
    return 0


if __name__ == "__main__":
    args = [p for a in sys.argv[1:] for p in (glob.glob(a) or [a])]
    sys.exit(main(args))
