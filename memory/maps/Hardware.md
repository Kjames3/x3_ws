---
title: Hardware
description: Map of content — robot, compute, sensors, mounts
tags: [moc, hardware]
---

# Hardware

Back to [[Home]].

## Platform

| Part | Detail |
|---|---|
| Robot | Yahboom X3, 4-wheel mecanum drive |
| Compute | Jetson Orin Nano, JetPack 6.2 / Ubuntu 22.04 |
| Controller | Rosmaster board on `/dev/ttyCH341USB0` — motors, IMU, battery, OLED |
| Lidar | YDLidar X3 on `/dev/ttyUSB0`, TOF, 512000 baud, 8 Hz |
| Camera | Orbbec Astra Pro (RGB + depth) |
| Camera | OAK-D Lite — see [[project_oakd_lite]] |

Low-level serial protocol lives in `src/Rosmaster_Lib.py` (Yahboom official).
Hardware abstraction — motors, lidar, camera, OLED, mecanum kinematics — is
`src/drivers_x3.py`.

## Known hardware issues

- **Lidar mounted too low.** At `laser_joint` z = 0.11 m the beam clips the chassis,
  producing fixed self-returns around +24° and −18° at 0.25–0.43 m. Currently worked
  around with scan/CBF filter params; the real fix is the riser bracket + URDF update.
  → [[project_lidar_mount]], `LIDAR_MOUNT_PLAN.md`
- **No hardware video encoder on the Orin Nano.** Rules out the obvious "just use NVENC"
  fix for camera streaming cost. → [[project_jetson_cpu_profile]]
- **OAK-D USB enumeration.** USB3 + 1080P works; USB2 fallback needed when the device
  crashes on 9001. → [[project_oakd_lite]]

## Related

- [[Software-Architecture]] — how the drivers are consumed
- [[Perception]] — what the cameras feed
- [[Build-and-Deploy]] — getting code onto the machine
