# C3 arm C on yolo26n-pose boxes vs yolo26n boxes (2026-09-28)

Same 18 held-out captures (r2+r3), same frozen arm C config (floor 0.05 m,
accel 1.5), same YOLO11x reference. Only arm C's person boxes changed:
`c3-detections-yolo26n-pose.json` (models/yolo26n-pose.pt, host, conf 0.35).
Reproduce:

    python3 src/c3_replay.py detect RUN --model models/yolo26n-pose.pt --name c3-detections-yolo26n-pose.json
    python3 src/c3_score.py score artifacts/c3-captures/*-r[23] --output artifacts/c3-eval-pose-2026-09-28 \
        --detections c3-detections-yolo26n-pose.json --config-c '{"accel_sigma_mps2": 1.5, "meas_sigma_floor_m": 0.05}'

| Arm C | yolo26n | yolo26n-pose |
|---|---:|---:|
| cross recall 0.5–1.8 / 1.8–3 / 3–4 m | 88.6 / 68.0 / 87.6 % | 87.3 / 70.6 / 91.4 % |
| approach, stand recall | 99–100 % | same |
| startstop recall | 87–93 % | 87–94 % |
| cross walk speed ratio 3–4 m | 0.93 | **0.55** (under-reads) |
| cross non-person outputs | 435 | 79 |
| stand non-person outputs | 834 | 11 |
| **empty-room non-person outputs** | **0** | **1428** (static, 0 false motion) |
| go / stop delay (median) | 0.27 / 0.25 s | 0.32 / 0.27 s (1 stop missed) |

- Recall is equal or slightly better. In replay, C3 keeps people 87–100 % of the
  time with pose boxes, so the ~50 % ring coverage seen live on 2026-09-28 is
  in the live path, not the model.
- **Phantom person:** in empty-r2 the pose model calls a white towel draped over
  a chair a person in 1433/1434 frames (conf 0.66). Static, so no false motion,
  but a persistent false person track. The yolo26n detector does not.
- The 3–4 m crossing speed under-read (ratio 0.55) needs a look before adoption.
- Host .pt inference, not the FP16 blob; one room.
