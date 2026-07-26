# July 2026 Improvement Ideas & Enhancement Log

This document serves as the active improvement ideas log and architectural roadmap for the EE244 Computational Learning Project (Predictive Local Planning via Onboard Velocity Estimation) for **July 2026**.

> [!NOTE]
> For historical context and prior ideas generated during June 2026, refer to the centralized archive in the [`ideas/`](file:///home/kamren/x3_ws/ideas) directory:
> - **Architectural Ideas (1–240)**: [ideas/June_architectural_ideas.md](file:///home/kamren/x3_ws/ideas/June_architectural_ideas.md)
> - **Performance & Efficiency (1–120)**: [ideas/June_performance_ideas.md](file:///home/kamren/x3_ws/ideas/June_performance_ideas.md)
> - **June ROI Analysis**: [ideas/June_roi_analysis.md](file:///home/kamren/x3_ws/ideas/June_roi_analysis.md)

---

## 1. Prioritization & ROI Summary Matrix

Use this matrix to track active ideas and their ROI tier at a glance.

| Idea ID | Date Logged | Domain | Title | ROI Tier | Status |
| :---: | :---: | :--- | :--- | :---: | :---: |
| *e.g., J-01* | *2026-07-26* | *Architecture* | *Template entry: OAK-D Lite Domain Gap Self-Training* | *High* | *Planned* |

---

## 2. Architecture & Algorithmic Enhancements
*Focus areas: Nav2/MPPI integration, Kalman filtering, state machine transitions, predictive costmap layers, and domain adaptation.*

<!-- Add new architectural ideas below this line -->

---

## 3. Performance & Execution Efficiency
*Focus areas: TensorRT acceleration, zero-copy shared memory IPC, asyncio event loop optimization, and memory allocation reduction.*

<!-- Add new performance ideas below this line -->

---

## 4. Sensor Fusion & Hardware Integration
*Focus areas: OAK-D vs. Astra Pro depth calibration, YDLidar X3 mounting/scan-matching, IMU slip compensation, and motor driver telemetry.*

<!-- Add new hardware/sensor ideas below this line -->

---

## 5. Log & Prioritization Guidelines

To maintain document readability and prevent log bloat:
1. **Categorize Immediately**: Place new ideas under their respective domain section (Sections 2–4).
2. **Assign an ROI Tier**: Grade each idea based on implementation effort vs. runtime impact:
   - **High ROI**: Low investment (<2 hours), significant gains in CPU/RAM, safety, or accuracy.
   - **Medium ROI**: Moderate effort (half-day to 1 day), solid architecture or navigation benefits.
   - **Low ROI**: High effort or minor edge-case improvements.
3. **Monthly Archiving**: At the end of each month, move completed or historical ideas to the `ideas/` archive folder.
