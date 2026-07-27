#!/usr/bin/env python3
"""
bench_all.py — measure the SIMULTANEOUS delivered rates of the OAK-D Lite's
mono L/R + depth + IMU, at the target config (mono 100 fps, depth on, IMU 500/400).

Needs exclusive access to the OAK, so free it first:
    sudo systemctl stop x3_server        # if the server is running
    (and Ctrl-C the webrtc PoC if it's running)

Run:  python3 bench_all.py
"""
import time
import depthai as dai

MONO_FPS = 100

p = dai.Pipeline()
ml = p.create(dai.node.MonoCamera); mr = p.create(dai.node.MonoCamera)
st = p.create(dai.node.StereoDepth); imu = p.create(dai.node.IMU)
xl = p.create(dai.node.XLinkOut); xl.setStreamName("l")
xr = p.create(dai.node.XLinkOut); xr.setStreamName("r")
xd = p.create(dai.node.XLinkOut); xd.setStreamName("d")
xi = p.create(dai.node.XLinkOut); xi.setStreamName("i")

for m, sock in ((ml, dai.CameraBoardSocket.CAM_B), (mr, dai.CameraBoardSocket.CAM_C)):
    m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    m.setBoardSocket(sock)
    m.setFps(MONO_FPS)

st.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
st.setLeftRightCheck(True)

imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(20)

ml.out.link(st.left); mr.out.link(st.right)
ml.out.link(xl.input); mr.out.link(xr.input)
st.depth.link(xd.input); imu.out.link(xi.input)

with dai.Device(p, maxUsbSpeed=dai.UsbSpeed.SUPER) as dev:
    print("USB link:", dev.getUsbSpeed().name, " (mono requested", MONO_FPS, "fps)")
    ql = dev.getOutputQueue("l", 8, False); qr = dev.getOutputQueue("r", 8, False)
    qd = dev.getOutputQueue("d", 8, False); qi = dev.getOutputQueue("i", 40, False)
    time.sleep(1.0)  # warmup
    cl = cr = cd = ci = 0
    t0 = time.time()
    while time.time() - t0 < 6.0:
        if ql.tryGet() is not None: cl += 1
        if qr.tryGet() is not None: cr += 1
        if qd.tryGet() is not None: cd += 1
        pk = qi.tryGet()
        if pk is not None: ci += len(pk.packets)
        time.sleep(0.0003)
    dt = time.time() - t0
    print(f"SIMULTANEOUS:  left {cl/dt:5.0f} fps | right {cr/dt:5.0f} fps | "
          f"depth {cd/dt:5.0f} fps | imu {ci/dt:5.0f} Hz")
