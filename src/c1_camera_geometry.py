"""Explicit RGB sensor-mode geometry for the 1080P preview, not a fitted calibration.

The CAM_A preview (and the depth aligned to it) is the 1920x1080 ISP output
CENTRE-CROPPED to the preview aspect and scaled uniformly (depthai's
setPreviewKeepAspectRatio defaults to True), so a 480x640 preview keeps only
the middle 810x1080 of the frame and fx == fy. ``getCameraIntrinsics(CAM_A,
w, h)`` models neither the crop nor the uniform scale and returns fx and fy
~2.4x too small. Map the EEPROM intrinsics through the real crop instead.
Verified 2026-09-25 on the OAK-D Pro W: a 30.1 cm board at 0.57 m spans
~377 depth columns (fx ~676 predicts 357 plus stereo edge fattening; the
uniform-scale fx 285.6 predicts 151).
"""
import numpy as np

# EEPROM calibration size -> (x, y) of the 3840x2160 region that the 1080P
# mode 2x-bins. Refuse other EEPROM geometries rather than guess.
#   IMX214 (OAK-D Lite): centered 3840x2160 crop of 4208x3120.
#     https://docs.luxonis.com/hardware/sensors/IMX214
#   IMX378 (OAK-D Pro W): EEPROM is already calibrated at 3840x2160 (read from
#     the device 2026-09-25), which is exactly the region 1080P bins.
_1080P_CROPS = {
    ('IMX214', (4208, 3120)): (184, 480),
    ('IMX378', (3840, 2160)): (0, 0),
}


def rgb_1080_preview_crop(sensor_name, native_size, preview_size):
    """Return ``(x, y, w, h)`` of the native-pixel region the preview shows."""
    key = (sensor_name.upper(), tuple(int(v) for v in native_size))
    if key not in _1080P_CROPS:
        raise ValueError(f'unqualified RGB sensor/calibration geometry: {sensor_name} {native_size}')
    w, h = preview_size
    if min(w, h) <= 0:
        raise ValueError('invalid preview dimensions')
    ox, oy = _1080P_CROPS[key]
    # Largest centred region of the 3840x2160 mode with the preview's aspect.
    scale = max(w / 3840, h / 2160)
    cw, ch = w / scale, h / scale
    return ox + (3840 - cw) / 2, oy + (2160 - ch) / 2, cw, ch


def rgb_1080_preview_intrinsics(native_k, native_size, preview_size, sensor_name):
    """Crop 3840x2160, 2x bin, centre-crop to the preview aspect, scale uniformly."""
    x, y, cw, ch = rgb_1080_preview_crop(sensor_name, native_size, preview_size)
    k = np.asarray(native_k, dtype=float)
    if k.shape != (3, 3) or not np.isfinite(k).all():
        raise ValueError('invalid intrinsics')
    s = preview_size[0] / cw
    crop = np.array([[1., 0., -x], [0., 1., -y], [0., 0., 1.]])
    return np.diag([s, s, 1.]) @ crop @ k


# Kept for callers written against the Lite.
imx214_1080_preview_intrinsics = rgb_1080_preview_intrinsics
