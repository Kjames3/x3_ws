"""
oakd_ros_publisher.py — Publish OakDCamera streams onto ROS2 topics.

The OAK-D Lite is driven in-process by `oakd_driver.OakDCamera` (DepthAI opens
the device exclusively, so no second process can grab it). That makes its
streams invisible to `ros2 bag record`. This module bridges the gap: it polls
the driver's cached getters from a background thread and republishes them as
standard sensor_msgs, so the OAK data can be bagged / viewed in RViz / Foxglove
alongside /scan, /odom and the Astra depth.

Topics (all under /oak):
    /oak/depth/image_raw    sensor_msgs/Image        16UC1, millimetres
    /oak/depth/camera_info  sensor_msgs/CameraInfo   depth-aligned intrinsics
    /oak/points             sensor_msgs/PointCloud2  xyz, optical frame (optional)
    /oak/left/image_raw     sensor_msgs/Image        mono8  (optional)
    /oak/right/image_raw    sensor_msgs/Image        mono8  (optional)
    /oak/imu                sensor_msgs/Imu          6-axis, no orientation
    /oak/detections         vision_msgs/Detection3DArray  (optional)

Publishing is poll-based and de-duplicated: a frame is only sent when the
driver has produced a new one, so a publish rate above the device rate costs
nothing but a few wakeups. Nothing here touches the driver's capture path.

Usage:
    pub = OakRosPublisher(node, oak, rate_hz=10.0)
    pub.start()
    ...
    pub.stop()
"""

import array
import logging
import threading
import time

import numpy as np

import oakd_cloud

logger = logging.getLogger(__name__)

# Frame ids follow the URDF (yahboomcar_X3.urdf.xacro, "OAK-D Lite (2026)" block)
# and the depthai_ros naming convention.
DEPTH_FRAME = "oak_rgb_camera_optical_frame"   # depth is aligned to CAM_A
LEFT_FRAME = "oak_left_camera_optical_frame"
RIGHT_FRAME = "oak_right_camera_optical_frame"
IMU_FRAME = "oak_imu_frame"

# Depth is stored by the driver as float32 metres; ROS convention for integer
# depth images is 16UC1 millimetres (same as the Astra's /camera/depth/image_raw,
# so downstream preprocessing handles both cameras identically). Anything beyond
# uint16 range (65.535 m) is unusable stereo noise and is clamped to 0 = invalid.
_MAX_DEPTH_MM = 65535


def _as_uint8_array(arr):
    """Pack `arr` into an array.array('B') for an Image.data field.

    Image.data is uint8[]. rosidl's generated setter early-returns ONLY for an
    array.array with typecode 'B'; bytes (and numpy arrays) fall through to an
    `if __debug__` block that validates every element individually — ~600k
    checks per image, which is what throttled the publish loop. A uint16 depth
    frame is reinterpreted, not converted: little-endian on aarch64, matching
    the is_bigendian = 0 we advertise.
    """
    return array.array('B', np.ascontiguousarray(arr).tobytes())


class OakRosPublisher:
    def __init__(self, node, oak, rate_hz=10.0, publish_stereo=True,
                 publish_detections=True, publish_images=True,
                 publish_cloud=False, cloud_rate_hz=5.0,
                 cloud_stride=oakd_cloud.DEFAULT_STRIDE,
                 cloud_z_min=oakd_cloud.DEFAULT_Z_MIN,
                 cloud_z_max=oakd_cloud.DEFAULT_Z_MAX,
                 cloud_max_points=oakd_cloud.DEFAULT_MAX_POINTS):
        self._node = node
        self._oak = oak
        self._period = 1.0 / max(rate_hz, 0.1)
        self._publish_stereo = publish_stereo and publish_images
        self._publish_detections = publish_detections
        self._publish_images = publish_images

        # The costmap needs a few Hz, not the full depth rate — each cloud costs
        # a raytrace pass through the voxel layer, so publishing at 30 Hz would
        # burn CPU in Nav2 for no extra obstacle information.
        self._publish_cloud = publish_cloud
        self._cloud_period = 1.0 / max(cloud_rate_hz, 0.1)
        self._cloud_stride = cloud_stride
        self._cloud_z_min = cloud_z_min
        self._cloud_z_max = cloud_z_max
        self._cloud_max_points = cloud_max_points
        self._last_cloud_t = 0.0

        self._running = False
        self._thread = None

        # De-dup state: the driver swaps in a fresh ndarray per frame, so object
        # identity is a free "is this new?" test (no pixel comparison).
        self._last_depth_obj = None
        self._last_left_obj = None
        self._last_right_obj = None
        self._last_imu_ts = None
        self._last_info_shape = None

        from sensor_msgs.msg import CameraInfo, Image, Imu
        from rclpy.qos import qos_profile_sensor_data
        self._Image = Image
        self._Imu = Imu
        self._CameraInfo = CameraInfo

        self._depth_pub = self._info_pub = None
        if publish_images:
            self._depth_pub = node.create_publisher(Image, "/oak/depth/image_raw",
                                                    qos_profile_sensor_data)
            self._info_pub = node.create_publisher(CameraInfo, "/oak/depth/camera_info",
                                                   qos_profile_sensor_data)
        self._cloud_pub = None
        if publish_cloud:
            from sensor_msgs.msg import PointCloud2
            self._cloud_pub = node.create_publisher(PointCloud2, "/oak/points",
                                                    qos_profile_sensor_data)
        self._imu_pub = node.create_publisher(Imu, "/oak/imu", qos_profile_sensor_data)
        self._left_pub = self._right_pub = None
        if self._publish_stereo:
            self._left_pub = node.create_publisher(Image, "/oak/left/image_raw",
                                                   qos_profile_sensor_data)
            self._right_pub = node.create_publisher(Image, "/oak/right/image_raw",
                                                    qos_profile_sensor_data)

        # vision_msgs ships with a full Humble desktop install but not with
        # ros-humble-ros-base; degrade to "no detections topic" rather than
        # taking the whole publisher down with an ImportError.
        self._det_pub = None
        if publish_detections:
            try:
                from vision_msgs.msg import (Detection3D, Detection3DArray,
                                             ObjectHypothesisWithPose)
                self._Detection3D = Detection3D
                self._Detection3DArray = Detection3DArray
                self._ObjectHypothesisWithPose = ObjectHypothesisWithPose
                self._det_pub = node.create_publisher(Detection3DArray,
                                                      "/oak/detections", 10)
            except ImportError as e:
                logger.warning(f"OakRosPublisher: vision_msgs unavailable ({e}); "
                               "/oak/detections disabled")

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="oak-ros-pub",
                                        daemon=True)
        self._thread.start()
        logger.info(f"OakRosPublisher: publishing /oak/* at {1.0 / self._period:.0f} Hz "
                    f"(images {'on' if self._publish_images else 'off'}, "
                    f"stereo {'on' if self._publish_stereo else 'off'}, "
                    f"detections {'on' if self._det_pub is not None else 'off'}, "
                    f"cloud {f'on @ {1.0 / self._cloud_period:.0f} Hz' if self._cloud_pub is not None else 'off'})")

    def stop(self):
        self._running = False
        t = self._thread
        if t is not None:
            t.join(timeout=2.0)
        self._thread = None
        logger.info("OakRosPublisher: stopped")

    # ------------------------------------------------------------------ worker
    def _run(self):
        # Achieved rate vs configured rate, logged on a ~30 s window. A gap
        # between the two means _publish_once is not keeping up (GIL contention
        # with the capture/asyncio threads, or a slow serialisation path).
        win_n, win_t, win_work = 0, time.monotonic(), 0.0
        while self._running:
            t0 = time.monotonic()
            try:
                self._publish_once()
            except Exception as e:
                logger.error(f"OakRosPublisher: publish failed: {e}")
                time.sleep(1.0)     # don't spin hot on a persistent failure
            work = time.monotonic() - t0
            win_n += 1
            win_work += work
            if t0 - win_t >= 30.0:
                elapsed = time.monotonic() - win_t
                logger.info(f"OakRosPublisher: {win_n / elapsed:.1f} Hz achieved "
                            f"(target {1.0 / self._period:.0f}), "
                            f"{1000.0 * win_work / win_n:.0f} ms/publish")
                win_n, win_t, win_work = 0, time.monotonic(), 0.0
            sleep = self._period - work
            if sleep > 0:
                time.sleep(sleep)

    def _publish_once(self):
        stamp = self._node.get_clock().now().to_msg()

        raw = self._oak.get_raw_depth_frame()
        if raw is not None and raw is not self._last_depth_obj:
            self._last_depth_obj = raw
            if self._depth_pub is not None:
                self._depth_pub.publish(self._depth_msg(raw, stamp))
                self._publish_camera_info(raw.shape, stamp)
            if self._cloud_pub is not None:
                self._publish_cloud_once(raw, stamp)

        if self._publish_stereo:
            left, right = self._oak.get_stereo_frames()
            if left is not None and left is not self._last_left_obj:
                self._last_left_obj = left
                self._left_pub.publish(self._mono_msg(left, stamp, LEFT_FRAME))
            if right is not None and right is not self._last_right_obj:
                self._last_right_obj = right
                self._right_pub.publish(self._mono_msg(right, stamp, RIGHT_FRAME))

        imu = self._oak.get_imu()
        if imu is not None and imu.get("ts") != self._last_imu_ts:
            self._last_imu_ts = imu.get("ts")
            self._imu_pub.publish(self._imu_msg(imu, stamp))

        if self._det_pub is not None and getattr(self._oak, "spatial_active", False):
            self._det_pub.publish(
                self._detections_msg(self._oak.get_spatial_detections(), stamp))

    # ------------------------------------------------------------------ builders
    def _depth_msg(self, raw_m, stamp):
        # metres (float32) -> millimetres (uint16). Negative/NaN and out-of-range
        # samples become 0, the ROS convention for "no reading".
        mm = np.nan_to_num(raw_m * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)
        # Zero, don't clamp: a 65 m "reading" from a 0.1 m baseline is noise, and
        # clamping would hand the training pipeline a wall of fake 65.535 m pixels.
        mm[(mm < 0.0) | (mm > _MAX_DEPTH_MM)] = 0.0
        mm = mm.astype(np.uint16)

        msg = self._Image()
        msg.header.stamp = stamp
        msg.header.frame_id = DEPTH_FRAME
        msg.height, msg.width = mm.shape[:2]
        msg.encoding = "16UC1"
        msg.is_bigendian = 0
        msg.step = msg.width * 2
        # Assign a uint8 ndarray, NOT bytes: rosidl's generated setter for a
        # uint8[] field validates bytes/list input element-by-element under
        # `if __debug__` (~600k checks per image), which throttled the publish
        # loop to ~1 Hz. The ndarray branch returns early and skips all of it.
        msg.data = _as_uint8_array(mm)
        return msg

    def _mono_msg(self, frame, stamp, frame_id):
        # DepthAI mono getCvFrame() is already single-channel uint8; be tolerant
        # of a BGR frame in case the pipeline config changes.
        if frame.ndim == 3:
            frame = frame[:, :, 0]
        frame = np.ascontiguousarray(frame, dtype=np.uint8)

        msg = self._Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = msg.width
        msg.data = _as_uint8_array(frame)   # ndarray fast path — see _depth_msg
        return msg

    def _imu_msg(self, sample, stamp):
        msg = self._Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = IMU_FRAME
        accel = sample.get("accel", {})
        gyro = sample.get("gyro", {})
        msg.linear_acceleration.x = float(accel.get("x", 0.0))
        msg.linear_acceleration.y = float(accel.get("y", 0.0))
        msg.linear_acceleration.z = float(accel.get("z", 0.0))
        msg.angular_velocity.x = float(gyro.get("x", 0.0))
        msg.angular_velocity.y = float(gyro.get("y", 0.0))
        msg.angular_velocity.z = float(gyro.get("z", 0.0))
        # The OAK-D Lite has no magnetometer and the driver reads raw accel/gyro
        # only — no fused orientation. -1 in element 0 is the ROS convention for
        # "this field is not supplied" and stops consumers using the zero quat.
        msg.orientation_covariance[0] = -1.0
        return msg

    def _publish_cloud_once(self, raw_m, stamp):
        now = time.monotonic()
        if now - self._last_cloud_t < self._cloud_period:
            return
        self._last_cloud_t = now

        scaled = oakd_cloud.scale_intrinsics(
            self._oak.get_depth_intrinsics(), raw_m.shape)
        if scaled is None:
            return      # device not up yet / readCalibration failed
        fx, fy, cx, cy = scaled

        pts = oakd_cloud.depth_to_points(
            raw_m, fx, fy, cx, cy,
            stride=self._cloud_stride,
            z_min=self._cloud_z_min,
            z_max=self._cloud_z_max,
            max_points=self._cloud_max_points)
        # An empty cloud is still worth publishing: the voxel layer needs an
        # observation to raytrace-clear stale obstacles with. Suppressing it
        # would leave phantom furniture in the costmap after the robot turns
        # away.
        self._cloud_pub.publish(
            oakd_cloud.make_cloud_msg(pts, stamp, DEPTH_FRAME))

    def _publish_camera_info(self, shape, stamp):
        h, w = shape[:2]
        # Depth-frame intrinsics, not CAM_A's: in the stereo-only fallback depth
        # is aligned to a mono socket at mono resolution, where CAM_A's numbers
        # do not apply.
        scaled = oakd_cloud.scale_intrinsics(
            self._oak.get_depth_intrinsics(), shape)
        if scaled is None:
            return      # device not up yet / readCalibration failed
        fx, fy, cx, cy = scaled

        if self._last_info_shape != (h, w):
            self._last_info_shape = (h, w)
            logger.info(f"OakRosPublisher: depth {w}x{h}, intrinsics "
                        f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

        msg = self._CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = DEPTH_FRAME
        msg.height, msg.width = h, w
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5           # depth is rectified on-device
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(msg)

    def _detections_msg(self, detections, stamp):
        msg = self._Detection3DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = DEPTH_FRAME
        for det in detections:
            xyz = det.get("xyz_m")
            if xyz is None:
                continue        # no depth behind the box — nothing to place in 3D
            d = self._Detection3D()
            d.header = msg.header
            hyp = self._ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(det.get("label", ""))
            hyp.hypothesis.score = float(det.get("conf", 0.0))
            hyp.pose.pose.position.x = float(xyz["x"])
            hyp.pose.pose.position.y = float(xyz["y"])
            hyp.pose.pose.position.z = float(xyz["z"])
            hyp.pose.pose.orientation.w = 1.0
            d.results.append(hyp)
            d.bbox.center.position.x = float(xyz["x"])
            d.bbox.center.position.y = float(xyz["y"])
            d.bbox.center.position.z = float(xyz["z"])
            d.bbox.center.orientation.w = 1.0
            # The driver only recovers a point, not an extent; a nominal
            # person-sized box keeps RViz/Foxglove markers visible.
            d.bbox.size.x = 0.5
            d.bbox.size.y = 0.5
            d.bbox.size.z = 1.7
            msg.detections.append(d)
        return msg
