"""One person, one box: group person boxes that mostly cover the same person.

Seg models split a close person into overlapping partial boxes (left half,
right half, whole body), each of which would become its own track. Shared by
the live OAK driver and the offline scorer (c3_seg_vs_pose.py) so both apply
the same rule. Pure Python; no numpy, ROS or DepthAI.
"""

# Boxes whose overlap covers this fraction of the SMALLER box are one person.
# Two people only merge when one is mostly hidden behind the other.
MERGE_CONTAIN = 0.6


def contain(a, b):
    """Intersection area over the smaller box's area (boxes are xyxy)."""
    iw = min(a[2], b[2]) - max(a[0], b[0])
    ih = min(a[3], b[3]) - max(a[1], b[1])
    if iw <= 0 or ih <= 0:
        return 0.0
    small = min((a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1]))
    return iw * ih / small if small > 0 else 0.0


def group_overlapping(boxes, threshold=MERGE_CONTAIN):
    """Index groups of xyxy boxes that overlap by >= threshold, transitively.

    Groups and their members keep input order (first member = lowest index).
    """
    parent = list(range(len(boxes)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            if contain(boxes[i], boxes[j]) >= threshold:
                ri, rj = find(i), find(j)
                parent[max(ri, rj)] = min(ri, rj)
    groups = {}
    for i in range(len(boxes)):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


def union_box(boxes):
    return [min(b[0] for b in boxes), min(b[1] for b in boxes),
            max(b[2] for b in boxes), max(b[3] for b in boxes)]
