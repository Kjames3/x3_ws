"""Confirmed lidar tracks join the costmap people list (people_buffer.merge_lidar_people)."""
import time

from lidar_person_tracker import cbf_estimates
from people_buffer import merge_lidar_people, people_cloud


def _track(fwd, left, confirmed=True, lidar_age=0.0, trusted=False):
    return {'id': 'L1', 'confirmed': confirmed, 'lidar_age_s': lidar_age, 'stamp': time.time(),
            'fwd': fwd, 'left': left, 'vx': 0.0, 'vy': 0.0, 'speed': 0.0,
            'vel_trusted': trusted, 'hits': 5}


def _merge(people, tracks):
    return merge_lidar_people(people, cbf_estimates(tracks, [], time.time()))


def test_lidar_track_is_added_when_memory_has_nobody():
    out = _merge([], [_track(2.0, 0.5)])
    assert len(out) == 1 and out[0]['pos'] == (2.0, 0.5) and out[0]['trusted'] is False
    assert len(people_cloud(out))


def test_duplicate_of_a_camera_person_is_skipped():
    cam = [{'pos': (2.1, 0.5), 'vel': (0.0, 0.0), 'trusted': True}]
    assert len(_merge(cam, [_track(2.0, 0.5)])) == 1


def test_unconfirmed_or_coasting_tracks_are_ignored():
    assert _merge([], [_track(2.0, 0.5, confirmed=False), _track(3.0, 0.0, lidar_age=0.2)]) == []


def test_trusted_lidar_velocity_stays_trusted():
    assert _merge([], [_track(2.0, 0.0, trusted=True)])[0]['trusted'] is True
