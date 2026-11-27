# Surroundings methods research — 2026-11-27

Branch `overnight/surroundings-research-2026-11-27` (base `4857e5e`; not merged). Research only: no runtime code changed, no robot access, no `PROGRESS.md` edit.

**Question:** what should replace or improve the browser GUI's current voxel-derived white-box surroundings, with better furniture orientation and curved person paths, without breaking the AGX Orin / ROS 2 Humble budget?

**Short answer:** do not chase photorealism first. Build a layered, local 3D scene: static shell from a bounded TSDF/ESDF or the current voxel grid, furniture as semantic objects with geometric heading evidence and confidence, and people as separate dynamic tracks with sampled predicted paths. The highest-value prototype is nvblox-style local reconstruction for geometry, paired with a conservative furniture-heading prior and the already-plumbed trajectory shadow path.

## 1. Current approach and constraints

`src/web/reality-structure.js` already converts base-footprint voxel cells into wall segments and object stand-ins. Constants are tuned for 0.2 m cells, 1.2 m walls, 0.09 m line tolerance, 6 minimum inliers, 0.45 m gap splitting, 0.6 m minimum segments, 14 segments, 0.55 fill ratio, 15° snapping, and 0.4/0.3 m merge offsets. Object positions are OAK detections; nearby lidar voxels determine size/orientation. Tracks persist in odom frame.

`src/web/reality-lookalike.js` consumes colored OAK points in odom, with per-object box pose and people masking. `src/web/reality-mode.js` already accepts a scene-space `[x, z]` path per person and draws a widening uncertainty ribbon; the current path is constant velocity. `src/traj_predictor.py` already defines the adapter wanted here: `observe(track_id, t, x, y, vx, vy)`, `predict(track_id)`, optional sigma, absolute ODOM/world path, gap 0.35 s, stale 2.0 s, with CV fallback.

Compute matters. `notes/plans/overnight_perf_2026-11-26.md` measured `server_x3.py` at 521% CPU, mostly 11 OpenBLAS spin-yield threads, with only 0–15% GR3D use. That leaves GPU headroom for reconstruction, but not much tolerance for more unthrottled Python/Numpy work. `notes/plans/livox_fastlio_2026-10-29.md` showed local FAST-LIO at 10 Hz, ~57% of one core, 167 MB RSS, timestamp age 66 ms median / 78 ms p95, and stationary excursions of 3.2 mm / 0.033°. It is isolated from Nav2/CBF and not yet a moving-accuracy result.

The GUI target is also not full photorealism: the scene is a dark room with white, low-texture objects, and the display is three.js. Geometry, headings, and uncertainty are more useful than appearance. The platform baseline is consistent: ROS 2 Humble targets Ubuntu 22.04 on amd64/aarch64, and JetPack 6.2 ships an Ubuntu 22.04 rootfs plus TensorRT and cuDNN ([ROS install](https://docs.ros.org/en/humble/Installation.html), [JetPack 6.2](https://docs.nvidia.com/jetson/jetpack/6.2/introduction/index.html)).

## 2. Method comparison

Numbers below are from their own papers or docs; they are not a common benchmark.

| Family | What it gives | Fit to X3 | Verdict |
|---|---|---|---|
| OctoMap | Occupancy voxels, `.bt/.ot`, `OccupancyGrid` projection | ROS 2 Humble CI; `PointCloud2` input; static cumulative log-odds; no time decay/dynamic model | **Prototype as a validation layer**, not the final look |
| Voxblox | CPU TSDF/ESDF, mesh, free-space distance for planning | Strong idea, but official repo is ROS 1; ROS 2 PR #413 is unmerged and omits ESDF node | **Skip port**; use nvblox or a small custom 2.5D TSDF |
| Isaac ROS nvblox | GPU TSDF/color/mesh/ESDF, 2D slices, people/dynamic modes | ROS 2 Humble on Jetson Orin; documented AGX Orin timings; local rolling map; requires good pose and 30 Hz depth imager | **Best geometry prototype** |
| S-Graphs+ | Four-layer graph: keyframes, walls, rooms, floors | ROS 2 Humble tested; planar indoor semantics; 3D LiDAR; heavy SLAM | **Skip for GUI now**; useful only if wall/room topology becomes a goal |
| FAST-LIO2 | Consistent local cloud + odometry | ROS 2 Humble port exists; local FAST-LIO already running in shadow; dynamic objects can contaminate maps | **Use as pose/cloud input only after moving validation** |
| VoteNet / 3DETR / PointPillars / CenterPoint | 3D boxes, classes, heading/yaw, velocity in CenterPoint | Geometry-only detection is plausible for white furniture; desktop numbers exist, but none of the official repos ship ROS 2 Humble nodes or Jetson benchmarks | **Conditional prototype** if OAK boxes stay poor; CenterPoint+TensorRT is the strongest detector path |
| Scan2CAD / NOCS / CHIP / HouseCat6D | CAD alignment or category-level 6D pose | Scan2CAD predicts 9DoF with 20° rotation tolerance; CHIP studies texture-unreliable chairs; NOCS avoids exact CAD but has small object classes | **Use as evidence for a heading module**, not a full runtime dependency yet |
| ConceptGraphs / OpenScene | Open-vocabulary 3D scene graphs and per-point CLIP features | Good long-term semantics; needs RGB-D, pose, foundation-model features, graph association; OpenScene feature files are hundreds of GB | **Skip now**; too heavy for this GUI problem |
| 3D Gaussian splatting / NeRF | Photorealistic novel views | Original training wants CUDA 7.0+, 24 GB VRAM and ~7 h on an A6000; browser streaming saves 32.9% bandwidth on average but still moves tens to hundreds of MB; Orin NX inference 2–9 FPS | **Skip for control GUI**; optional offline showroom only |
| Social LSTM / GAN / Trajectron++ / TrajMamba / social force | Curved or multimodal person forecasts | Current GUI already has a path hook and scoring tool; scene/map-heavy models cost too much | **Prototype TrajMamba-style velocity model in shadow** |

## 3. Geometry and occupancy

### OctoMap

OctoMap's ROS 2 `octomap_mapping` repo is on the `ros2` branch, uses `ament_cmake`/`rclcpp`, and CI includes Humble, Jazzy, Kilted, and Rolling ([repo](https://api.github.com/repos/OctoMap/octomap_mapping), [README](https://raw.githubusercontent.com/OctoMap/octomap_mapping/ros2/README.md), [CI](https://raw.githubusercontent.com/OctoMap/octomap_mapping/ros2/.github/workflows/ros2_ci_build_and_test.yml)). Relevant ROS nodes and interfaces:

- `octomap_server_node`, `static`, `multilayer`, `tracking`, `color`, and `saver` executables ([CMakeLists](https://raw.githubusercontent.com/OctoMap/octomap_mapping/ros2/octomap_server/CMakeLists.txt)).
- `cloud_in` as `sensor_msgs/PointCloud2`, `frame_id=map`, `base_frame_id=base_footprint`; TF needed at cloud stamp for ground filtering ([server source](https://raw.githubusercontent.com/OctoMap/octomap_mapping/ros2/octomap_server/src/octomap_server.cpp)).
- Publishes `octomap_binary`, `octomap_full`, occupied-cell centers, marker arrays, and `projected_map` as `nav_msgs/OccupancyGrid`; services include `~/clear_bbox` and `~/reset` ([server source](https://raw.githubusercontent.com/OctoMap/octomap_mapping/ros2/octomap_server/src/octomap_server.cpp), [header](https://raw.githubusercontent.com/OctoMap/octomap_mapping/ros2/octomap_server/include/octomap_server/octomap_server.hpp)).
- Defaults include `resolution=0.05`, `occupancy_min_z/max_z`, `filter_speckles=false`, `filter_ground_plane=false`, `compress_map=true`, `incremental_2D_projection=false`, `publish_free_space=false`, sensor model hit 0.7 / miss 0.4 / min 0.12 / max 0.97 ([server source](https://raw.githubusercontent.com/OctoMap/octomap_mapping/ros2/octomap_server/src/octomap_server.cpp)).

This is useful as a **sanity layer**: feed the Livox cloud, inspect `projected_map` and occupied-cell centers, and compare wall/furniture geometry with the current voxel pipeline. It is not a dynamics solution: insertion is cumulative log-odds raycasting with no time decay; removal requires later free observations or explicit clear/reset calls. `tracking_octomap_server` only publishes changed keys, not a semantic dynamic model ([tracking source](https://raw.githubusercontent.com/OctoMap/octomap_mapping/ros2/octomap_server/src/tracking_octomap_server.cpp)).

### Voxblox

Voxblox is the conceptual reference for TSDF/ESDF, but the official repo is ROS 1: `package.xml` uses `catkin`, `CMakeLists.txt` uses `catkin_simple`, and there is no official `ros2` branch ([repo](https://api.github.com/repos/ethz-asl/voxblox), [package.xml](https://raw.githubusercontent.com/ethz-asl/voxblox/master/voxblox_ros/package.xml), [CMakeLists](https://raw.githubusercontent.com/ethz-asl/voxblox/master/voxblox_ros/CMakeLists.txt)). Issue #364 "ROS2 Support" remains open; an ETH collaborator suggested nvblox as ROS 2 native and faster on Jetson ([issue](https://api.github.com/repos/ethz-asl/voxblox/issues/364), [comments](https://api.github.com/repos/ethz-asl/voxblox/issues/364/comments)). PR #413 ports only `tsdf_server_node` and the RViz plugin, with ESDF server explicitly left as future work ([PR](https://api.github.com/repos/ethz-asl/voxblox/pulls/413)).

Its published CPU numbers are still informative: Voxblox beat OctoMap by 3–37× in the listed KITTI cases but used 2–3× the RAM; mesh/integrator timings scale from 56 ms / 49 MB at 20 cm to 527 ms / 609 MB at 2 cm on an i7-4810MQ ([performance docs](https://voxblox.readthedocs.io/en/latest/pages/Performance.html)). Do not spend the night porting it.

### nvblox

Isaac ROS nvblox is the best fit for a local 3D upgrade. It takes depth images and/or 3D LiDAR plus pose, reconstructs on GPU, and outputs mesh and 2D cost/distance slices ([concept docs](https://nvidia-isaac-ros.github.io/v/release-3.1/concepts/scene_reconstruction/nvblox/index.html), [technical details](https://nvidia-isaac-ros.github.io/v/release-3.1/concepts/scene_reconstruction/nvblox/technical_details.html)). Default layers are TSDF, color, mesh, and ESDF; ESDF can be 3D or 2D, mesh and ESDF update at 5 Hz by default, and ESDF slice height/radius are configurable ([parameters](https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/parameters.html)).

Compatibility is direct: documented for ROS 2 Humble on Jetson Orin / JetPack 6.0, with Debian install `ros-humble-isaac-ros-nvblox`, source clone `-b release-3.1`, and Isaac ROS 3.0 adding JetPack 6.0 / Ubuntu 22.04 / CUDA 12.2 support ([nvblox docs](https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_nvblox/index.html), [meta-package](https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html), [releases](https://nvidia-isaac-ros.github.io/v/release-3.1/releases/index.html)).

AGX Orin core timings at 5 cm voxels were TSDF 0.6–0.8 ms, color 1.1–1.2 ms, meshing 1.1–2.5 ms, and ESDF 1.7 ms on Replica/Redwood ([nvblox docs](https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_nvblox/index.html)). Newer nvblox work reports local 1 cm reconstruction at 30 Hz on AGX Orin in an 8 m³ workspace ([release notes](https://github.com/nvidia-isaac/nvblox/blob/public/release-notes.md)).

Caveats:

- Camera requirements are strict: 30 Hz imager, ±2 ms jitter, ±100 µs stereo offset ([nvblox docs](https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_nvblox/index.html)). The OAK-D Pro W needs a measured check before assuming it qualifies.
- It needs a pose source; the docs use visual SLAM or another estimator. The isolated FAST-LIO output is a candidate after moving validation, not an accuracy assumption.
- It is intended as **local mapping**. NVIDIA maintainers say compute scales with observed updates, not total map size, but memory grows; map clearing and radius cropping were needed, and loop closures/global pose corrections are not handled inside nvblox ([issue #96](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/issues/96)).
- People/dynamic modes use segmentation or freespace violation to separate dynamic occupancy; Dynablox-based dynamic reconstruction cannot detect objects moving slower than odometry drift ([technical details](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/technical_details.html)).

**Prototype shape:** run at 3–5 cm voxels, bounded to a 4–6 m radius, mesh + ESDF slice only, no automatic Nav2/CBF wiring. Export a decimated triangle mesh or 2.5D slice to the GUI and overlay it on the existing walls/objects. Compare screenshots and timing against the current voxel renderer before replacing anything.

### S-Graphs+ and FAST-LIO2

S-Graphs+ is a ROS 2 hierarchical SLAM graph with keyframes, wall planes, rooms, and floors, extracted from 3D LiDAR and odometry via sequential RANSAC and room/floor segmentation ([paper](https://arxiv.org/abs/2212.11770), [repo](https://github.com/snt-arg/lidar_situational_graphs), [install docs](https://github.com/snt-arg/lidar_situational_graphs/blob/35dd3561/docs/1.installation.md)). It matches structured indoor walls much better than furniture. Adopt only if the GUI later needs persistent rooms/walls as a map.

FAST-LIO2 ROS 2 supports Ubuntu 22.04 / Humble and includes loop closure, PGO, relocalization, and HBA ([repo](https://github.com/liangheming/FASTLIO2_ROS2)). For X3 it should remain a shadow/local-pose input because dynamic people can contaminate the map and the mounted-sensor validation is stationary only ([local note](notes/plans/livox_fastlio_2026-10-29.md)).

## 4. Objects and furniture heading

The current heading problem is not solved by "more voxels" alone. The useful signal is part geometry plus relational priors:

- chair: backrest height, seat plane, leg cluster, and facing toward a table;
- table/couch: support plane and long-axis extent;
- all objects: nearby wall axes and owner memory across observations.

VoteNet shows heading can be predicted from geometry only: its outputs include center, heading, scale, class, and objectness, with 12 heading bins and no color images ([paper](https://arxiv.org/abs/1904.09664), [CVF](https://openaccess.thecvf.com/content_ICCV_2019/html/Qi_Deep_Hough_Voting_for_3D_Object_Detection_in_Point_Clouds_ICCV_2019_paper.html)).

3DETR is the indoor RGB-D option: model-zoo pretrained checkpoints report ScanNet AP25/AP50 65.0/47.0 (`3DETR-m`) and 62.1/37.9 (`3DETR`), but inference was 170 ms versus VoteNet's 132 ms on a V100 at batch 8, and the tested stack is PyTorch 1.9 / CUDA 10.2 / Python 3.6 ([paper](https://arxiv.org/abs/2109.08141), [repo](https://github.com/facebookresearch/3detr)). PointPillars reports 62 Hz and 59.20 KITTI moderate 3D mAP on an i7 + 1080 Ti, with TensorRT giving a 45.5% speedup, but the official repo is archived/unmaintained and targets Ubuntu 16.04/18.04 with PyTorch 0.4.1 ([paper](https://arxiv.org/abs/1812.05784), [repo](https://github.com/nutonomy/second.pytorch)). CenterPoint is the strongest detector path: it emits center, offset, height, size, yaw, velocity, and optional second-stage refinement; nuScenes pillar detection reports 31 FPS at mAP 50.3 / NDS 60.2, Waymo two-frame two-stage reports mAPH 71.9 at 11 FPS, and a TensorRT port reports RTX 3080 FP16 RPN inference of 6.89 ms plus 2.37 ms postprocess ([paper](https://arxiv.org/abs/2006.11275), [repo](https://github.com/tianweiy/CenterPoint), [TensorRT repo](https://github.com/Abraham423/CenterPointTensorRT)). None of these ships a ROS 2 Humble node or Jetson benchmark; treat them as fallback detector ports, not drop-ins.

The furniture-specific literature supports a cautious approach:

- Scan2CAD aligns CAD models to noisy RGB-D scans and outputs 9DoF pose; its benchmark tolerates 20 cm translation, 20° rotation, and 20% scale, with top classes chairs, tables, and cabinets ([CVF](https://openaccess.thecvf.com/content_CVPR_2019/html/Avetisyan_Scan2CAD_Learning_CAD_Model_Alignment_in_RGB-D_Scans_CVPR_2019_paper.html), [repo](https://github.com/skanti/Scan2CAD), [benchmark docs](https://kaldir.vc.in.tum.de/scan2cad_benchmark/documentation)).
- NOCS estimates 6D pose and size for unseen instances without exact CAD at train/test time, but its classes are bottles, bowls, cameras, cans, laptops, and mugs ([paper](https://arxiv.org/abs/1901.02970), [repo](https://github.com/hughw19/NOCS_CVPR2019)).
- CHIP is closer to the visual problem: 7 wooden chairs, 77,811 RGB-D frames, texture-less CAD models, "unreliable textures", occlusion, and pose variation; baselines still struggle ([paper](https://arxiv.org/abs/2506.09699), [repo](https://github.com/tev-fbk/CHIP), [dataset](https://huggingface.co/datasets/FBK-TeV/CHIP)).
- HouseCat6D includes objects without texture and active-stereo depth, but its household categories are small items such as bottles, cups, shoes, and teapots ([paper](https://arxiv.org/abs/2212.10428), [CVF](https://openaccess.thecvf.com/content/CVPR2024/html/Jung_HouseCat6D_-_A_Large-Scale_Multi-Modal_Category_Level_6D_Object_Perception_CVPR_2024_paper.html)).
- T-LESS and the IKEA Object State Dataset are useful proxies for textureless/same-colour objects, but neither is a drop-in dynamic furniture pose system ([T-LESS](https://arxiv.org/abs/1701.05498), [IKEA](https://arxiv.org/abs/2111.08614), [repo](https://github.com/mxllmx/IKEAObjectStateDataset)).

No primary source found tests exactly dark, untextured white furniture under robot motion with precise yaw recovery. Treat any heading estimate as uncertain until measured locally.

**Prototype shape:** extend the existing furniture fitter with a heading confidence. Use back-height/seat-plane evidence first; snap only to the nearest wall axis when evidence is weak; keep a symmetric box or hide the heading when confidence is low. Add relations to nearby objects only after logging shows chairs/tables are repeatedly misoriented. Persist the heading in odom and update it with a slow filter, not a hard per-frame snap.

## 5. Scene graphs, Gaussian splatting, and NeRF

ConceptGraphs builds open-vocabulary 3D scene graphs by associating 2D foundation-model outputs across views ([repo](https://github.com/concept-graphs/concept-graphs), [paper](https://arxiv.org/abs/2309.16650)). The newer `ali-dev` path targets streamlined real-time RGB-D input, but this still adds feature extraction, association, and graph maintenance. Its published Jackal deployment is ROS 1 Noetic with a Velodyne VLP-16 and `move_base`, not ROS 2 or Jetson ([Jackal repo](https://github.com/sachaMorin/concept_graphs_jackal)). OpenScene stores per-point CLIP-space features for zero-shot segmentation and retrieval; the supplement reports ~24 h A100 training and 0.1 s inference, but released feature sets are huge (ScanNet OpenSeg fused train/val 234.8 GB) and fusion needs an NVIDIA GPU with >30 GB memory ([paper](https://arxiv.org/abs/2211.15654), [repo](https://github.com/pengsongyou/openscene), [install](https://raw.githubusercontent.com/pengsongyou/openscene/main/installation.md)). These are the right direction if the GUI later needs "the red mug is on the left table", not the next fix for white furniture headings.

3D Gaussian splatting is a rendering format, not a control map. The original implementation trains on a CUDA capability 7.0+ GPU with 24 GB VRAM for paper quality, uses CUDA SDK 11.8 on Ubuntu 22.04, takes COLMAP `SIMPLE_PINHOLE` or `PINHOLE` input, and needs about 7 hours on an A6000 for full evaluation. Its released pre-trained models are 14 GB ([project](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/), [paper](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/3d_gaussian_splatting_high.pdf), [README](https://raw.githubusercontent.com/graphdeco-inria/gaussian-splatting/main/README.md)). That is offline work, not something to run on the robot.

Browser rendering exists, but it is its own engineering project. SGSS forks the WebGL2 viewer, partitions the scene into cuboids, prioritizes visible Gaussians, and aborts/reissues HTTP `Range` requests as the view changes. Its test setup used 100/150/200 Mbit/s links with 20 ms RTT and reported 32.9% less network transfer on average. The dolly scenes still downloaded 53.3-241.6 MB at 200 Mbit/s. The garden scene has over 5.8 million Gaussians and 1.4 GB; SH level 3 spends 192 of 236 bytes per Gaussian on coefficients. In the WebGL2 test on an RTX 3090, SH3 stayed under 15 FPS above 3.5M Gaussians, and SH1 reached 60 FPS only below 3M ([paper](https://yaoliu-yl.github.io/publications/mmsys25-sgss.pdf), [repo](https://github.com/symmru/SGSS), [SH tradeoffs](https://yaoliu-yl.github.io/publications/Internet_Computing_25.pdf)). Spark is the active Three.js path with `.PLY`, `.SPZ`, `.SPLAT`, `.KSPLAT`, and `.SOG` support. WebGPU WebSplatter needs Chrome 113+, Safari 26, or Firefox 141+ ([Spark](https://sparkjs.dev/), [WebSplatter](https://github.com/websplatter/WebSplatter)).

Jetson numbers are inference experiments, not a ready deployment. StreamingGS reports original 3DGS at 2-9 FPS on Orin NX, with projection and sorting taking about 90% of the work, then uses a custom 32 nm accelerator for up to 45.7x speedup ([paper](https://arxiv.org/html/2506.09070v1)). SeeLe reports AGX Orin baseline 20.79/41.97/23.94 FPS rising to 59.67/127.80/86.58 on Mip-NeRF360, Tanks&Temples, and Deep Blending ([paper](https://arxiv.org/html/2503.05168v3)). LS-Gaussian targets 90 FPS on AGX Orin with a custom 16 nm accelerator ([paper](https://arxiv.org/html/2507.21572v2)). These need more build work than the current GUI has budget for.

Dynamic scenes are the bigger mismatch. 4DGS uses a deformation field and reaches 82 FPS at 800x800 on an RTX 3090, but large motion, missing background points, and imprecise poses remain hard. Deformable 3D Gaussians handle non-intense motion and avoid quantitative HyperNeRF results because the poses were inaccurate. Dynamic 3D Gaussians report 850 FPS after 2 hours on one RTX 3090, but track only scene parts visible in the initial frame and need multiple cameras ([4DGS](https://openaccess.thecvf.com/content/CVPR2024/html/Wu_4D_Gaussian_Splatting_for_Real-Time_Dynamic_Scene_Rendering_CVPR_2024_paper.html), [Deformable](https://arxiv.org/html/2309.13101v1), [Dynamic3DGaussians](https://github.com/JonathonLuiten/Dynamic3DGaussians)). A person walking into the room is exactly this case.

Recommendation: skip for the live GUI. Capture a room offline for a separate showroom mode if appearance matters. Do not gate navigation or CBF display on a splat stream.

## 6. Person trajectory prediction

The cheapest high-value work is to judge the current CV and GRU baselines before adding a larger model. The GUI's path hook is already in place (`src/web/reality-mode.js`, `src/web/scene-view.js`), and `traj_shadow.py` plus `server_x3.py --traj-shadow` log predictions without acting. `src/tools/perception/traj_shadow_score.py` can score mean/median FDE versus CV at 0.5/1/2/3 s, skip gaps >0.5 s, separate moving tracks >0.3 m/s, and check 2σ coverage. `src/tools/perception/train_traj_gru.py` supports run-grouped folds and optional sigma.

Primary-source options:

| Method | Evidence | Fit |
|---|---|---|
| Social Force | Langevin motion from desired velocity, other agents, borders/attractions ([APS](https://journals.aps.org/pre/abstract/10.1103/PhysRevE.51.4282)) | Useful non-learned baseline, not a learned predictor |
| Social LSTM | Social pooling; original paper reports 0.27/0.61 on its metric, later standard L2 reproduction 0.72/1.54 ([paper](https://openaccess.thecvf.com/content_cvpr_2016/papers/Alahi_Social_LSTM_Human_CVPR_2016_paper.pdf), [Trajectron++](https://arxiv.org/html/2001.03093v5)) | Skip; expensive per-step pooling |
| Social GAN | Best-of-20 0.39/0.58 at 8 steps, 0.78/1.18 at 12 steps; 0.04/0.05 s on P100 ([paper](https://openaccess.thecvf.com/content_cvpr_2018/papers/Gupta_Social_GAN_Socially_CVPR_2018_paper.pdf)) | Conditional only if K modes can be displayed/selected |
| Trajectron++ | ETH/UCY ML 0.37/0.91, best-of-20 0.19/0.41; CPU inference up to 1.2 s ([paper](https://arxiv.org/html/2001.03093v5)) | Skip; map/dynamics burden |
| CAR-Net / SoPhie | Need scene/top-view images ([CAR-Net](https://www.ecva.net/papers/eccv_2018/papers_ECCV/papers/Amir_Sadeghian_CAR-Net_Clairvoyant_Attentive_ECCV_2018_paper.pdf), [SoPhie](https://arxiv.org/abs/1806.01482)) | Skip; no such scene tensor |
| ScePT / ASTRA / EMP | Scene/map-aware multimodal forecasting ([ScePT](https://arxiv.org/html/2206.13387v1), [ASTRA](https://arxiv.org/html/2501.09878v1), [EMP](https://arxiv.org/html/2409.16154v2)) | Skip; too heavy or wrong map domain |
| TrajMamba | 262K params, 2 s observation / 3 s prediction at 10 Hz, ETH/UCY avg ADE/FDE 0.29/0.51, deployed on Jetson Orin ([paper](https://arxiv.org/html/2610.33916v1), [repo](https://github.com/basheeraltawil/NIT-TrajMamba)) | **Best prototype**; per-person velocity sequence matches the current API |
| Res-GCNN | ~6.4K params, ADE 0.37 / FDE 0.65, tiny graph model ([paper](https://arxiv.org/abs/2011.09214)) | Conditional second if neighbor interaction matters |

**Prototype shape:** keep the numpy GRU as the control, add a TrajMamba-style per-track velocity model in shadow, and ensure it returns the same absolute ODOM path plus sigma. The GUI expects 16 scene `[x, z]` points over 2.0 s, so resample before display. Promotion gate: beat CV on moving-subset mean and median FDE at 0.5/1/2/3 s, remain no worse at 3 s, and meet coverage if probabilistic. No controller/CBF promotion without a separate decision.

## 7. Ranked next steps

1. **Capture a local baseline set first.** Log Livox clouds, OAK boxes/points, pose, current furniture yaw, person tracks, and GUI screenshots. Run the existing trajectory shadow scoring before training anything. Without this, "better" has no local definition.
2. **Prototype bounded nvblox geometry in shadow.** Use the OAK depth or Livox cloud plus validated pose; export only a decimated mesh or 2D slice to the browser at 3–5 Hz. Do not touch Nav2/CBF. Check 30 Hz camera timing, GPU/CPU cost, and memory over a 10-minute run.
3. **Fix furniture heading with evidence and confidence.** Extend the current back/seat logic with wall-axis priors, object relations, and an explicit "unknown heading" state. Build a small labelled set from the baseline captures and measure yaw error against the current fitter.
4. **Add a lightweight 2.5D TSDF fallback if nvblox fails camera-rate checks.** The existing lidar voxel pipeline already supplies cells; fuse them into a local height-bounded surface grid and render top-down polygons. This is less beautiful but cheap and predictable.
5. **Prototype TrajMamba-style shadow prediction.** Reuse `TrajPredictor` adapter, scorer, and 16-point GUI path. Add Res-GCNN only if neighbour interactions show up in the failures.
6. **Keep S-Graphs+, ConceptGraphs/OpenScene, 3DETR/PointPillars/CenterPoint runtime use, Scan2CAD runtime use, and 3DGS/NeRF on the "later" shelf.** Each needs a concrete local failure that the top five do not address.

## 8. What not to do

- Do not replace AMCL/Nav2 maps with a global TSDF before solving pose corrections and memory growth.
- Do not let dynamic people enter the static shell; keep them in a separate layer with finite lifetime.
- Do not force a furniture heading from a single noisy frame; store uncertainty and update slowly in odom.
- Do not route any new prediction into CBF or motion control from the GUI path experiment.
- Do not feed a photorealistic splat stream into safety decisions.

## Sources

Primary sources used above:

- Platform: https://docs.ros.org/en/humble/Installation.html , https://docs.nvidia.com/jetson/jetpack/6.2/introduction/index.html
- OctoMap: https://github.com/OctoMap/octomap_mapping/blob/ros2/octomap_server/launch/octomap_mapping.launch.xml , https://api.github.com/repos/OctoMap/octomap_mapping , https://raw.githubusercontent.com/OctoMap/octomap_mapping/ros2/README.md , https://raw.githubusercontent.com/OctoMap/octomap_mapping/ros2/octomap_server/src/octomap_server.cpp
- Voxblox: https://github.com/ethz-asl/voxblox/ , https://voxblox.readthedocs.io/en/latest/pages/Performance.html , https://voxblox.readthedocs.io/en/latest/pages/Using-Voxblox-for-Planning.html , https://api.github.com/repos/ethz-asl/voxblox/issues/364 , https://api.github.com/repos/ethz-asl/voxblox/pulls/413
- Isaac ROS nvblox: https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_nvblox/index.html , https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html , https://nvidia-isaac-ros.github.io/v/release-3.1/concepts/scene_reconstruction/nvblox/technical_details.html , https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/parameters.html , https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/issues/96 , https://github.com/nvidia-isaac/nvblox/blob/public/release-notes.md , https://nvidia-isaac-ros.github.io/v/release-3.1/releases/index.html
- S-Graphs+: https://arxiv.org/abs/2212.11770 , https://github.com/snt-arg/lidar_situational_graphs , https://snt-arg.github.io/lidar_situational_graphs/ , https://github.com/snt-arg/lidar_situational_graphs/blob/35dd3561/docs/1.installation.md
- FAST-LIO2 ROS 2: https://github.com/liangheming/FASTLIO2_ROS2
- 3D detection: https://arxiv.org/abs/1904.09664 , https://arxiv.org/abs/2109.08141 , https://github.com/facebookresearch/3detr , https://arxiv.org/abs/1812.05784 , https://github.com/nutonomy/second.pytorch , https://arxiv.org/abs/2006.11275 , https://github.com/tianweiy/CenterPoint , https://github.com/Abraham423/CenterPointTensorRT , https://arxiv.org/html/2601.12638
- Furniture/CAD/6D pose: https://github.com/skanti/Scan2CAD/ , https://openaccess.thecvf.com/content_CVPR_2019/html/Avetisyan_Scan2CAD_Learning_CAD_Model_Alignment_in_RGB-D_Scans_CVPR_2019_paper.html , https://kaldir.vc.in.tum.de/scan2cad_benchmark/documentation , https://arxiv.org/abs/1901.02970 , https://github.com/hughw19/NOCS_CVPR2019 , https://arxiv.org/abs/2506.09699 , https://github.com/tev-fbk/CHIP , https://arxiv.org/abs/2212.10428 , https://arxiv.org/abs/1701.05498 , https://arxiv.org/abs/2111.08614
- Scene graphs: https://github.com/concept-graphs/concept-graphs , https://arxiv.org/abs/2309.16650 , https://github.com/sachaMorin/concept_graphs_jackal , https://arxiv.org/abs/2211.15654 , https://github.com/pengsongyou/openscene , https://raw.githubusercontent.com/pengsongyou/openscene/main/installation.md
- Gaussian splatting: https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/ , https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/3d_gaussian_splatting_high.pdf , https://raw.githubusercontent.com/graphdeco-inria/gaussian-splatting/main/README.md , https://doi.org/10.1145/3712676.3714437 , https://github.com/symmru/SGSS , https://yaoliu-yl.github.io/publications/mmsys25-sgss.pdf , https://yaoliu-yl.github.io/publications/Internet_Computing_25.pdf , https://github.com/antimatter15/splat , https://github.com/mkkellogg/GaussianSplats3D , https://github.com/sparkjsdev/spark , https://sparkjs.dev/ , https://arxiv.org/html/2602.03207 , https://websplatter.github.io/ , https://github.com/websplatter/WebSplatter , https://arxiv.org/html/2506.09070v1 , https://doi.org/10.1109/DAC63849.2025.11132470 , https://arxiv.org/html/2507.21572v2 , https://ieeexplore.ieee.org/document/11240950 , https://arxiv.org/html/2503.05168v3 , https://openaccess.thecvf.com/content/CVPR2026/papers/Zhu_Seele_A_Unified_Acceleration_Framework_for_Real-Time_Gaussian_Splatting_on_CVPR_2026_paper.pdf , https://doi.org/10.1145/3712676.3714445 , https://people.cs.nycu.edu.tw/~chuang/pubs/pdf/2025mmsys.pdf , https://github.com/AIINS-NTHU/LTS-DASH-Streaming-System-for-3DGS , https://openaccess.thecvf.com/content/CVPR2024/html/Wu_4D_Gaussian_Splatting_for_Real-Time_Dynamic_Scene_Rendering_CVPR_2024_paper.html , https://github.com/hustvl/4DGaussians , https://arxiv.org/html/2309.13101v1 , https://arxiv.org/pdf/2308.09713 , https://dynamic3dgaussians.github.io/ , https://github.com/JonathonLuiten/Dynamic3DGaussians , https://eecs.site.nthu.edu.tw/var/file/5/1005/img/5540/285252546.pdf
- Trajectory prediction: https://journals.aps.org/pre/abstract/10.1103/PhysRevE.51.4282 , https://openaccess.thecvf.com/content_cvpr_2016/papers/Alahi_Social_LSTM_Human_CVPR_2016_paper.pdf , https://openaccess.thecvf.com/content_cvpr_2018/papers/Gupta_Social_GAN_Socially_CVPR_2018_paper.pdf , https://arxiv.org/html/2001.03093v5 , https://www.ecva.net/papers/eccv_2018/papers_ECCV/papers/Amir_Sadeghian_CAR-Net_Clairvoyant_Attentive_ECCV_2018_paper.pdf , https://arxiv.org/abs/1806.01482 , https://arxiv.org/html/2206.13387v1 , https://arxiv.org/html/2501.09878v1 , https://arxiv.org/html/2409.16154v2 , https://arxiv.org/html/2610.33916v1 , https://github.com/basheeraltawil/NIT-TrajMamba , https://arxiv.org/abs/2011.09214

Branch `overnight/surroundings-research-2026-11-27`; HEAD at research start `4857e5e`; one-line result: prototype bounded nvblox geometry, confidence-scored furniture headings, and TrajMamba-style shadow prediction; skip 3DGS/NeRF and heavy scene graphs for now.
