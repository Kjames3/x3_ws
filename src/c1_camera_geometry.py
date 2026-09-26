"""Explicit RGB sensor-mode geometry for the 1080P preview, not a fitted calibration.

The CAM_A preview (and the depth aligned to it) is the 1920x1080 ISP output
STRETCHED to a non-16:9 grid such as 480x640. ``getCameraIntrinsics(CAM_A, w, h)``
assumes one uniform scale, so on that grid it returns fy ~= fx and a wrong cy.
Map the EEPROM intrinsics through the sensor's real crop instead.
"""
import numpy as np

# EEPROM calibration size -> (crop x, crop y) of the 3840x2160 region that the
# 1080P mode 2x-bins. Refuse other EEPROM geometries rather than guess.
#   IMX214 (OAK-D Lite): centered 3840x2160 crop of 4208x3120.
#     https://docs.luxonis.com/hardware/sensors/IMX214
#   IMX378 (OAK-D Pro W): EEPROM is already calibrated at 3840x2160 (read from
#     the device 2026-09-25), which is exactly the region 1080P bins.
_1080P_CROPS = {
    ('IMX214', (4208, 3120)): (184, 480),
    ('IMX378', (3840, 2160)): (0, 0),
}


def rgb_1080_crop(sensor_name, native_size):
    key = (sensor_name.upper(), tuple(int(v) for v in native_size))
    if key not in _1080P_CROPS:
        raise ValueError(f'unqualified RGB sensor/calibration geometry: {sensor_name} {native_size}')
    return _1080P_CROPS[key]


def rgb_1080_preview_intrinsics(native_k, native_size, preview_size, sensor_name):
    """Crop to 3840x2160, then 2x binning, then stretch to preview_size.

    Binning and preview scale compose to preview/crop size.
    """
    ox, oy = rgb_1080_crop(sensor_name, native_size)
    w, h = preview_size
    k = np.asarray(native_k, dtype=float)
    if k.shape != (3, 3) or not np.isfinite(k).all() or min(w, h) <= 0:
        raise ValueError('invalid intrinsics or preview dimensions')
    crop = np.array([[1., 0., -ox], [0., 1., -oy], [0., 0., 1.]])
    return np.diag([w / 3840, h / 2160, 1.]) @ crop @ k


# Kept for callers written against the Lite.
imx214_1080_preview_intrinsics = rgb_1080_preview_intrinsics
