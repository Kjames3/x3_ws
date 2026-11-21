import lidar_person_dataset as D


def test_livox_position_follows_the_chassis():
    assert D.livox_xyz_for('artifacts/person-lidar-2026-10-30/walkup-r1') == D.LIVOX_XYZ_OLD
    assert D.livox_xyz_for('artifacts/person-lidar-2026-11-09/apt-desk2-park30-r1') == D.LIVOX_XYZ_OLD   # before the 11-13 rebuild
    assert D.livox_xyz_for('artifacts/person-lidar-2026-11-20/seated-r1') == D.LIVOX_XYZ_V2
    assert D.livox_xyz_for('artifacts/person-lidar-2026-11-13/x') == D.LIVOX_XYZ_V2
