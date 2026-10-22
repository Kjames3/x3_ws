"""Tests for people_buffer (social buffer cloud + speed cap) and PersonMemory.people.

Coordinates are the base frame: x forward, y left.  Estimates for PersonMemory use the
camera convention of the estimator (z = depth ahead, x = lateral, positive right).
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src'))
from people_buffer import (CAP_FAR_M, CAP_FLOOR, CAP_NEAR_M, FREE_SPEED, MAX_POINTS,  # noqa: E402
                           RADIUS_M, UNKNOWN_EXTRA_M, Z_M, cap_velocity, people_cloud,
                           speed_cap)
from person_memory import PersonMemory  # noqa: E402


def person(x, y, vx=0.0, vy=0.0, trusted=True, ghost=False):
    return {"pos": (x, y), "vel": (vx, vy), "trusted": trusted, "ghost": ghost}


def est(tid, z, x=0.0, vx=0.0, vy=0.0, person_frames=20, jump_age=20, age=0):
    return {"id": tid, "z": z, "x": x, "vx": vx, "vy": vy, "speed": math.hypot(vx, vy),
            "category": "person", "age": age, "person_frames": person_frames,
            "jump_age": jump_age}


# ------------------------------------------------------------------ the cloud
def test_no_people_no_points():
    c = people_cloud([])
    assert c.shape == (0, 3) and c.dtype == np.float32


def test_standing_person_is_a_filled_disk_at_the_right_height():
    c = people_cloud([person(1.0, 0.2)])
    d = np.hypot(c[:, 0] - 1.0, c[:, 1] - 0.2)
    assert d.max() == pytest.approx(RADIUS_M, abs=1e-4)
    assert (d < RADIUS_M / 2).any() and (np.abs(c[:, 2] - Z_M) < 1e-6).all()
    # filled: no 5 cm gap anywhere from the centre outwards
    assert any(abs(r - RADIUS_M / 3) < 0.02 for r in d) and any(abs(r - 2 * RADIUS_M / 3) < 0.02 for r in d)


def test_untrusted_velocity_means_more_room_not_a_prediction():
    c = people_cloud([person(1.0, 0.0, vx=-1.0, trusted=False)])
    d = np.hypot(c[:, 0] - 1.0, c[:, 1])
    assert d.max() == pytest.approx(RADIUS_M + UNKNOWN_EXTRA_M, abs=1e-4)
    assert c[:, 0].min() > 1.0 - (RADIUS_M + UNKNOWN_EXTRA_M) - 1e-4     # nothing placed ahead of them


def test_trusted_walker_gets_the_places_they_are_heading():
    c = people_cloud([person(1.2, 0.0, vx=-1.0)])
    for cx in (1.2, 0.7, 0.2):      # now, +0.5 s, +1.0 s
        assert (np.hypot(c[:, 0] - cx, c[:, 1]) < 0.05).any(), f"nothing marked near x={cx}"
    assert c[:, 0].min() < 0.0       # the 1 s disk reaches past the robot's own position


def test_a_slow_trusted_person_is_not_predicted():
    c = people_cloud([person(1.0, 0.0, vx=-0.05)])
    assert c[:, 0].min() > 1.0 - RADIUS_M - 1e-4


def test_cloud_is_capped_and_keeps_the_nearest_people():
    crowd = [person(0.8 + 0.3 * k, (-1) ** k * 0.5, vx=-1.0) for k in range(30)]
    c = people_cloud(crowd)
    assert len(c) <= MAX_POINTS
    assert (np.hypot(c[:, 0] - 0.8, c[:, 1] - 0.5) < 0.05).any()      # the nearest is in


# ----------------------------------------------------------------- speed cap
def test_speed_cap_shape():
    assert speed_cap([]) == FREE_SPEED
    assert speed_cap([(CAP_FAR_M + 0.5, 0.0)]) == FREE_SPEED
    assert speed_cap([(CAP_NEAR_M, 0.0)]) == CAP_FLOOR == speed_cap([(0.2, 0.0)])
    ds = np.linspace(0.3, 2.0, 40)
    caps = [speed_cap([(d, 0.0)]) for d in ds]
    assert all(b >= a - 1e-9 for a, b in zip(caps, caps[1:]))          # never drops as they get further
    assert speed_cap([(1.0, 0.0), (3.0, 0.0)]) == speed_cap([(1.0, 0.0)])   # the nearest decides


def test_approaching_a_person_is_capped_and_direction_is_kept():
    vx, vy = cap_velocity(0.26, 0.10, [(0.8, 0.0)])
    assert math.hypot(vx, vy) == pytest.approx(speed_cap([(0.8, 0.0)]), abs=1e-9)
    assert vy / vx == pytest.approx(0.10 / 0.26, abs=1e-9)


def test_passing_beside_a_person_is_capped_too():
    vx, vy = cap_velocity(0.0, 0.25, [(0.0, 0.6)])           # moving along... toward them: capped
    assert math.hypot(vx, vy) == pytest.approx(speed_cap([(0.0, 0.6)]))
    vx, vy = cap_velocity(0.25, 0.0, [(0.0, 0.6)])           # going straight past, 0.6 m to the side
    assert math.hypot(vx, vy) == pytest.approx(speed_cap([(0.0, 0.6)]))


def test_a_retreat_is_never_slowed():
    assert cap_velocity(-0.45, 0.0, [(0.4, 0.0)]) == (-0.45, 0.0)
    assert cap_velocity(0.0, -0.40, [(0.0, 0.5)]) == (0.0, -0.40)


def test_far_people_and_zero_velocity_are_left_alone():
    assert cap_velocity(0.26, 0.0, [(2.5, 0.0)]) == (0.26, 0.0)
    assert cap_velocity(0.0, 0.0, [(0.3, 0.0)]) == (0.0, 0.0)
    assert cap_velocity(0.26, 0.0, []) == (0.26, 0.0)


# ------------------------------------------------------- PersonMemory.people
def test_people_lists_standing_people_that_the_mover_list_leaves_out():
    mem = PersonMemory()
    for k in range(10):
        movers = mem.update([est(1, 1.4, x=-0.3, vx=0.02)], 0.1 * k)
    assert movers == []
    assert len(mem.people) == 1 and mem.people[0]["trusted"] and not mem.people[0]["ghost"]
    assert mem.people[0]["pos"] == pytest.approx((1.4, 0.3))


def test_people_lists_a_ghost_then_drops_it():
    mem = PersonMemory()
    for k in range(12):
        mem.update([est(1, 1.6 - 0.1 * k, vx=-1.0)], 0.1 * k)
    mem.update([], 1.3)
    assert len(mem.people) == 1 and mem.people[0]["ghost"]
    mem.update([], 3.5)
    assert mem.people == []


def test_people_far_away_are_listed_by_position_only():
    mem = PersonMemory()
    mem.update([est(5, 2.8, x=0.4, vx=-1.0)], 0.0)
    assert len(mem.people) == 1
    assert mem.people[0]["vel"] == (0.0, 0.0) and mem.people[0]["pos"] == pytest.approx((2.8, -0.4))
    mem.update([est(6, 4.2)], 0.1)
    assert mem.people == []                      # beyond the buffer range


def test_an_untrusted_newcomer_is_listed_only_after_a_moment():
    mem = PersonMemory()
    mem.update([est(9, 1.2, person_frames=1, jump_age=0)], 0.0)
    assert mem.people == []
    mem.update([est(9, 1.2, person_frames=3, jump_age=2)], 0.4)
    assert len(mem.people) == 1 and mem.people[0]["trusted"] is False


# ------------------------------------------------------------ wiring guards
def test_both_costmaps_mark_the_topic_the_server_publishes():
    params = yaml.safe_load((REPO / 'src/yahboomcar_nav/params/nav2_params_x3.yaml').read_text())
    server = (REPO / 'src/server_x3.py').read_text()
    assert "'/people/obstacles'" in server and "people_cloud(" in server
    for name in ('local_costmap', 'global_costmap'):
        layer = params[name][name]['ros__parameters']['obstacle_layer']
        assert 'people' in layer['observation_sources'].split()
        src = layer['people']
        assert src['topic'] == '/people/obstacles' and src['data_type'] == 'PointCloud2'
        assert src['marking'] is True and src['clearing'] is False
        assert src['min_obstacle_height'] <= Z_M <= src['max_obstacle_height']


def test_speed_cap_is_applied_in_the_cbf_path():
    server = (REPO / 'src/server_x3.py').read_text()
    assert "people_cap_velocity(safe_vx, safe_vy" in server
