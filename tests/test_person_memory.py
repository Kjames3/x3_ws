"""Tests for person_memory.PersonMemory: the CBF keeps people the tracker loses or relabels.

The scenarios are the ones found in the lab logs of 2026-10-22: a walker closing on the
robot until the camera loses them at 0.65 m, a person coming back under a new track id,
and the phantom-approach case the original mover guards were written for.

Coordinates: estimates carry the camera frame (z = depth ahead, x = lateral, positive to
the right); movers come out in the base frame (x forward, y left), like _cbf_movers.
"""
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src'))
from person_memory import PersonMemory, HOLD_S  # noqa: E402


def est(tid, z, x=0.0, vx=0.0, vy=0.0, person_frames=20, jump_age=20, age=0, category="person"):
    """One estimator track.  vx/vy are the base-frame velocity, as the estimator reports."""
    return {"id": tid, "z": z, "x": x, "vx": vx, "vy": vy, "speed": math.hypot(vx, vy),
            "category": category, "age": age, "person_frames": person_frames,
            "jump_age": jump_age}


def run_walker(mem, t0=0.0, z0=1.7, speed=1.0, until_z=0.65, dt=0.1, tid=1):
    """Feed a trusted walker approaching head-on until the camera loses them."""
    t, z = t0, z0
    out = []
    while z > until_z:
        out = mem.update([est(tid, z, vx=-speed)], t)
        z -= speed * dt
        t += dt
    return t, z + speed * dt, out


def test_trusted_walker_is_reported_like_before():
    mem = PersonMemory()
    t, z, out = run_walker(mem)
    assert out and out[0][1][0] < -0.9       # velocity passed through, approaching
    assert abs(out[0][0][0] - z) < 0.2


def test_option1_walker_lost_up_close_is_held_then_released():
    """15:06:30 in the lab: closing at 1.1 m/s, lost at 0.65 m, 10 s of nothing."""
    mem = PersonMemory()
    t, z, _ = run_walker(mem, speed=1.1)
    # the tracker now returns nothing for a while
    released = False
    for k in range(1, 25):
        out = mem.update([], t + 0.1 * k)
        gap = 0.1 * k
        if gap <= HOLD_S - 1e-9:
            assert len(out) == 1, f"ghost missing at gap {gap:.1f} s"
        else:
            assert out == [], f"ghost kept past the hold at gap {gap:.1f} s"
            released = True
    assert released, "the release branch was never exercised"
    assert mem.stats["ghost_started"] == 1


def test_option1_ghost_stops_running_on_its_velocity():
    mem = PersonMemory()
    t, z, _ = run_walker(mem, speed=1.0)
    p_early = mem.update([], t + 0.1)[0][0]
    p_late = mem.update([], t + 1.0)[0][0]
    p_later = mem.update([], t + 1.4)[0][0]
    assert p_late[0] < p_early[0]            # it moved on a little
    assert p_later == p_late                  # then froze (no unbounded extrapolation)
    # and its velocity has faded to almost nothing
    v = mem.update([], t + 1.4)[0][1]
    assert math.hypot(*v) < 0.1


def test_option2_new_id_inherits_the_walkers_velocity():
    """127 of 142 long dropouts came back under a different id; the new id is untrusted
    for ~10 frames and used to vanish from the CBF."""
    mem = PersonMemory()
    t, z, _ = run_walker(mem, speed=1.0, until_z=1.0)
    # 0.4 s gap, then the same person shows up as id 99 with a fresh, untrusted track
    t2 = t + 0.4
    out = mem.update([est(99, z - 0.4, vx=0.0, person_frames=1, jump_age=0)], t2)
    assert len(out) == 1
    (px, py), (vx, vy) = out[0]
    assert vx < -0.5, "velocity from before the re-label was dropped"
    assert mem.stats["relinked"] == 1
    # the carried velocity fades while the new track stays untrusted
    out_later = mem.update([est(99, z - 0.5, vx=0.0, person_frames=3, jump_age=2)], t2 + 1.5)
    assert abs(out_later[0][1][0]) < abs(vx)


def test_phantom_jump_does_not_inherit_velocity():
    """A track that jumps far (a bag pulled to a walker's depth) is a new entry: no
    carried velocity, so no invented 1 m/s approach."""
    mem = PersonMemory()
    t, z, _ = run_walker(mem, speed=1.0, until_z=1.2)
    out = mem.update([est(5, 0.2 + 1.7, x=1.2, vx=-1.0, person_frames=1, jump_age=0)], t + 0.1)
    # 1.7 m ahead, 1.2 m aside: well outside the gate of the walker 1.2 m ahead
    assert all(abs(v[0]) < 1e-9 for _, v in out)


def test_untrusted_stranger_is_static_after_a_short_tenure_and_only_close():
    mem = PersonMemory()
    assert mem.update([est(7, 1.2, person_frames=1, jump_age=0)], 0.0) == []
    out = mem.update([est(7, 1.2, person_frames=2, jump_age=1)], 0.4)
    assert len(out) == 1 and out[0][1] == (0.0, 0.0)
    far = PersonMemory()
    far.update([est(8, 1.7, person_frames=1, jump_age=0)], 0.0)
    assert far.update([est(8, 1.7, person_frames=3, jump_age=2)], 0.5) == []


def test_standing_trusted_person_is_left_to_the_lidar():
    mem = PersonMemory()
    for k in range(20):
        assert mem.update([est(3, 1.0, vx=0.03)], 0.1 * k) == []
    # and when they vanish there is no ghost either
    assert mem.update([], 2.2) == []


def test_other_categories_and_far_people_are_ignored():
    mem = PersonMemory()
    assert mem.update([est(1, 1.0, vx=-1.0, category="dynamic")], 0.0) == []
    assert mem.update([est(2, 3.0, vx=-1.0)], 0.1) == []


def test_two_people_keep_their_own_velocities():
    mem = PersonMemory()
    out = []
    for k in range(12):
        t = 0.1 * k
        out = mem.update([est(1, 1.5 - 0.1 * k, x=-0.6, vx=-1.0),
                          est(2, 1.4, x=0.6, vx=-0.6)], t)
    vs = sorted(round(v[0], 1) for _, v in out)
    assert vs == [-1.0, -0.6]
    # one of them vanishes: only its ghost remains alongside the other
    out = mem.update([est(2, 1.4, x=0.6, vx=-0.6)], 1.3)
    assert len(out) == 2


def test_old_estimates_not_matched_this_frame_do_not_count_as_observed():
    mem = PersonMemory()
    t, z, _ = run_walker(mem, speed=1.0)
    out = mem.update([est(1, z - 0.1, vx=-1.0, age=3)], t + 0.1)   # coasted, age > 0
    assert len(out) == 1 and mem.stats["ghost_started"] == 1       # treated as a ghost
