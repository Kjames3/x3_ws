import cv2
import depthai as dai
import numpy as np
import sys
import time
import collections


class FPSCounter:
    """Rolling FPS estimate from the last N frame arrival timestamps."""
    def __init__(self, window=30):
        self.timestamps = collections.deque(maxlen=window)

    def tick(self):
        self.timestamps.append(time.monotonic())

    def add(self, ts):
        """Record an externally-supplied timestamp (seconds)."""
        self.timestamps.append(ts)

    def fps(self):
        if len(self.timestamps) < 2:
            return 0.0
        span = self.timestamps[-1] - self.timestamps[0]
        if span <= 0:
            return 0.0
        return (len(self.timestamps) - 1) / span


def draw_overlay(frame, lines):
    """Draw a small semi-transparent banner with text lines at the top-left.

    Works on both single-channel (mono/depth) and BGR frames.
    """
    if frame.ndim == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    pad = 6
    line_h = 20
    box_h = pad * 2 + line_h * len(lines)
    box_w = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
                for t in lines) + pad * 2

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (box_w, box_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

    y = pad + line_h - 6
    for t in lines:
        cv2.putText(frame, t, (pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 1, cv2.LINE_AA)
        y += line_h
    return frame


# USB link speed cap:
#   "SUPER" = USB3 (full bandwidth — needed for high color/stereo FPS)
#   "HIGH"  = USB2 (more electrically robust; fall back here if you get
#             firmware errorId 9001 "Device has crashed" errors)
USB_SPEED = "SUPER"
_USB_SPEEDS = {
    "SUPER": dai.UsbSpeed.SUPER,
    "HIGH": dai.UsbSpeed.HIGH,
}

# Depth visualization range (meters). Pixels beyond this are clipped/black.
DEPTH_MIN_M = 0.2
DEPTH_MAX_M = 6.0
MONO_FPS = 30

# Color (center IMX214) camera resolution presets: name -> (SensorResolution, fps).
# NOTE: the sensor mode can't be changed on a live device, so switching resolution
# means rebuilding the pipeline and reconnecting. A future GUI/keyboard button would
# just restart the pipeline with a different COLOR_RESOLUTION key.
COLOR_RESOLUTIONS = {
    "4K":    (dai.ColorCameraProperties.SensorResolution.THE_4_K,   20),
    "1080P": (dai.ColorCameraProperties.SensorResolution.THE_1080_P, 30),
    "720P":  (dai.ColorCameraProperties.SensorResolution.THE_720_P,  30),
}
COLOR_RESOLUTION = "1080P"   # active preset

# The window is resizable; 4K frames are larger than most screens, so the display
# window opens at this size while the captured frame stays full resolution.
COLOR_WINDOW_SIZE = (1280, 720)


def main():
    print("Initializing DepthAI Pipeline for OAK-D Lite...")

    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    depth = pipeline.create(dai.node.StereoDepth)
    imu = pipeline.create(dai.node.IMU)
    camRgb = pipeline.create(dai.node.ColorCamera)
    # On-device MJPEG encoder so high-res color fits through the USB2 (HIGH) link.
    videoEnc = pipeline.create(dai.node.VideoEncoder)

    xoutLeft = pipeline.create(dai.node.XLinkOut)
    xoutRight = pipeline.create(dai.node.XLinkOut)
    xoutDepth = pipeline.create(dai.node.XLinkOut)
    xoutImu = pipeline.create(dai.node.XLinkOut)
    xoutRgb = pipeline.create(dai.node.XLinkOut)

    xoutLeft.setStreamName('left')
    xoutRight.setStreamName('right')
    xoutDepth.setStreamName('depth')
    xoutImu.setStreamName('imu')
    xoutRgb.setStreamName('rgb')

    # Properties
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    monoLeft.setFps(MONO_FPS)

    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    monoRight.setFps(MONO_FPS)

    # Depth properties
    depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    depth.setDepthAlign(dai.CameraBoardSocket.CAM_B)
    depth.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

    # Color camera properties (center IMX214, CAM_A)
    color_res, color_fps = COLOR_RESOLUTIONS[COLOR_RESOLUTION]
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(color_res)
    camRgb.setFps(color_fps)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # MJPEG-encode the full-res color stream on the device to fit USB2 bandwidth.
    videoEnc.setDefaultProfilePreset(color_fps, dai.VideoEncoderProperties.Profile.MJPEG)

    # IMU properties
    # Enable ACCELEROMETER_RAW at 500Hz and GYROSCOPE_RAW at 400Hz
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # Linking
    monoLeft.out.link(depth.left)
    monoRight.out.link(depth.right)

    monoLeft.out.link(xoutLeft.input)
    monoRight.out.link(xoutRight.input)
    # Metric depth map (uint16, millimeters) so we can report min/max/points.
    depth.depth.link(xoutDepth.input)
    imu.out.link(xoutImu.input)
    # Color -> on-device MJPEG encoder -> host.
    camRgb.video.link(videoEnc.input)
    videoEnc.bitstream.link(xoutRgb.input)

    # Connect to device and start pipeline. Link speed is controlled by USB_SPEED
    # at the top of the file (SUPER = USB3, HIGH = USB2 fallback).
    try:
        with dai.Device(pipeline, maxUsbSpeed=_USB_SPEEDS[USB_SPEED]) as device:
            print("Connected to OAK-D device!")
            usb_speed = device.getUsbSpeed().name
            print(f"USB link speed: {usb_speed}  (requested {USB_SPEED})")
            if usb_speed not in ("SUPER", "SUPER_PLUS") and USB_SPEED == "SUPER":
                print("WARNING: requested USB3 but negotiated USB2 — check the cable "
                      "and that you're on a USB3 (blue/SS) port.")

            # Fetch output queues
            qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
            qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)
            qDepth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            qImu = device.getOutputQueue(name="imu", maxSize=4, blocking=False)
            qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

            # Resizable window so 4K frames are viewable without filling the screen.
            cv2.namedWindow("Color Camera (RGB)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Color Camera (RGB)", *COLOR_WINDOW_SIZE)

            fpsLeft = FPSCounter()
            fpsRight = FPSCounter()
            fpsDepth = FPSCounter()
            fpsColor = FPSCounter()
            # Wider window for the IMU since it runs at hundreds of Hz; rate is
            # computed from on-device report timestamps, not loop arrival time.
            rateImu = FPSCounter(window=200)

            print("\nOpening camera viewer... Press 'q' on any window to quit.")

            while True:
                inLeft = qLeft.tryGet()
                inRight = qRight.tryGet()
                inDepth = qDepth.tryGet()
                inImu = qImu.tryGet()
                inRgb = qRgb.tryGet()

                # Display Color Camera (MJPEG bitstream decoded on host)
                if inRgb is not None:
                    frame = cv2.imdecode(inRgb.getData(), cv2.IMREAD_COLOR)
                    if frame is not None:
                        fpsColor.tick()
                        h, w = frame.shape[:2]
                        frame = draw_overlay(frame, [
                            f"Color (IMX214) [{COLOR_RESOLUTION}]",
                            f"Res: {w}x{h}   FPS: {fpsColor.fps():4.1f}",
                        ])
                        cv2.imshow("Color Camera (RGB)", frame)

                # Display Left Mono Camera
                if inLeft is not None:
                    fpsLeft.tick()
                    frame = inLeft.getCvFrame()
                    h, w = frame.shape[:2]
                    frame = draw_overlay(frame, [
                        "Left Stereo (OV7251 mono)",
                        f"Res: {w}x{h}   FPS: {fpsLeft.fps():4.1f}",
                    ])
                    cv2.imshow("Left Stereo Camera", frame)

                # Display Right Mono Camera
                if inRight is not None:
                    fpsRight.tick()
                    frame = inRight.getCvFrame()
                    h, w = frame.shape[:2]
                    frame = draw_overlay(frame, [
                        "Right Stereo (OV7251 mono)",
                        f"Res: {w}x{h}   FPS: {fpsRight.fps():4.1f}",
                    ])
                    cv2.imshow("Right Stereo Camera", frame)

                # Display Depth Map (metric, millimeters)
                if inDepth is not None:
                    fpsDepth.tick()
                    depthFrame = inDepth.getFrame()  # uint16, mm
                    h, w = depthFrame.shape[:2]
                    valid = depthFrame > 0
                    n_pts = int(valid.sum())
                    total = depthFrame.size
                    if n_pts > 0:
                        dmin = depthFrame[valid].min() / 1000.0
                        dmax = depthFrame[valid].max() / 1000.0
                    else:
                        dmin = dmax = 0.0

                    # Colorize valid depths within the display range.
                    vis = np.clip(depthFrame, DEPTH_MIN_M * 1000, DEPTH_MAX_M * 1000)
                    vis = ((vis - DEPTH_MIN_M * 1000) *
                           (255.0 / ((DEPTH_MAX_M - DEPTH_MIN_M) * 1000))).astype(np.uint8)
                    vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
                    vis[~valid] = 0  # invalid/no-return pixels stay black

                    vis = draw_overlay(vis, [
                        "Depth (aligned to Left)",
                        f"Res: {w}x{h}   FPS: {fpsDepth.fps():4.1f}",
                        f"Range: {dmin:4.2f}-{dmax:4.2f} m",
                        f"Valid pts: {n_pts} ({100.0 * n_pts / total:4.1f}%)",
                    ])
                    cv2.imshow("Depth Camera", vis)

                # Display IMU Data
                if inImu is not None:
                    # Feed every report's on-device timestamp so the rate
                    # reflects the true sensor Hz, not the viewer loop speed.
                    for packet in inImu.packets:
                        rateImu.add(packet.acceleroMeter.getTimestampDevice().total_seconds())

                    imuData = inImu.packets[-1]  # Get latest packet
                    accel = imuData.acceleroMeter
                    gyro = imuData.gyroscope

                    # Create a black image window to display text
                    imu_frame = np.zeros((200, 400, 3), dtype=np.uint8)
                    cv2.putText(imu_frame, f"Accel X: {accel.x:7.2f} m/s^2", (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(imu_frame, f"Accel Y: {accel.y:7.2f} m/s^2", (15, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(imu_frame, f"Accel Z: {accel.z:7.2f} m/s^2", (15, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    cv2.putText(imu_frame, f"Gyro X: {gyro.x:7.2f} rad/s", (210, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(imu_frame, f"Gyro Y: {gyro.y:7.2f} rad/s", (210, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(imu_frame, f"Gyro Z: {gyro.z:7.2f} rad/s", (210, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    cv2.putText(imu_frame, f"IMU rate: {rateImu.fps():5.1f} Hz", (15, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    cv2.imshow("IMU Data", imu_frame)

                if cv2.waitKey(1) == ord('q'):
                    break

    except RuntimeError as e:
        print(f"\nFailed to connect to OAK-D device: {e}")
        print("Please check your USB connection or pipeline configuration.")
        sys.exit(1)


if __name__ == "__main__":
    main()
