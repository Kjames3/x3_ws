# August 2026 Improvement Ideas & Enhancement Log

This document serves as the active improvement ideas log and architectural roadmap for the EE244 Computational Learning Project (Predictive Local Planning via Onboard Velocity Estimation) for **August 2026**.

---

## 1. Prioritization & ROI Summary Matrix

| Idea ID | Date Logged | Domain | Title | ROI Tier | Status |
| :---: | :---: | :--- | :--- | :---: | :---: |
| **A-01** | 2026-08-01 | Architecture | Global Optimal Bipartite Matching for Cross-Path Tracking | **High** | Logged (Iter 1) |
| **A-02** | 2026-08-01 | Performance | Connected Components for Zero-Overhead Depth Blob Extraction | **High** | Logged (Iter 1) |
| **A-03** | 2026-08-01 | Performance | Chunked Serial Payload Reading for Reduced Syscall Overhead | **High** | Logged (Iter 1) |
| **A-04** | 2026-08-01 | Performance | Vectorized Struct Unpacking for Telemetry Parsing | **High** | Logged (Iter 1) |
| **A-05** | 2026-08-01 | Architecture | Thread-Safe Serial Writes to Prevent Packet Corruption | **High** | Logged (Iter 2) |
| **A-06** | 2026-08-01 | Code Quality | Graceful Thread Termination and Non-Blocking I/O | **High** | Logged (Iter 2) |
| **A-07** | 2026-08-01 | Performance | Eliminate O(N^2) List Insertions in Window Padding | **High** | Logged (Iter 2) |
| **A-08** | 2026-08-01 | Code Quality | Refactor Duplicated UART Transmission Logic into a Helper Method | **High** | Logged (Iter 3) |
| **A-09** | 2026-08-01 | Architecture | Parameterize Hardcoded Camera Intrinsics and Constants | **Medium** | Logged (Iter 3) |
| **A-10** | 2026-08-01 | Performance | INT8/FP16 Quantization for the Velocity MLP Model | **High** | Logged (Iter 3) |
| **A-11** | 2026-08-01 | Performance | Migrate MLP Inference to ONNX Runtime / TensorRT | **High** | Logged (Iter 4) |
| **A-12** | 2026-08-01 | Architecture | Decouple OpenCV Visualization from the Inference Loop | **High** | Logged (Iter 4) |
| **A-13** | 2026-08-01 | Code Quality | Eliminate Bare Except Clauses and Implement Robust UART Error Handling | **High** | Logged (Iter 4) |
| **A-14** | 2026-08-01 | Architecture | Use Event-Driven Synchronization for Asynchronous UART Reads | **High** | Logged (Iter 5) |
| **A-15** | 2026-08-01 | Code Quality | Remove Redundant Runtime JIT Tracing | **High** | Logged (Iter 5) |
| **A-16** | 2026-08-01 | Architecture | Embed Scaling Operations Directly into the Model Graph | **High** | Logged (Iter 5) |
| **A-17** | 2026-08-01 | Performance | Optimize OpenCV Morphological Operations with Separable/Smaller Kernels | **Medium** | Logged (Iter 6) |
| **A-18** | 2026-08-01 | Performance | Vectorize Sliding Window Features using NumPy Instead of Python Loops | **High** | Logged (Iter 6) |
| **A-19** | 2026-08-01 | Performance | Optimize Checksum Validation using Native sum() in UART parsing | **High** | Logged (Iter 6) |
| **A-20** | 2026-08-01 | Performance | Avoid Pulling Colorized depth_frame if Visualization is Disabled | **Medium** | Logged (Iter 7) |
| **A-21** | 2026-08-01 | Architecture | Prioritize Tracking Proximal Obstacles When at Tracking Capacity | **High** | Logged (Iter 7) |
| **A-22** | 2026-08-01 | Performance | Use Command Queue Instead of Blind time.sleep Post-Write Delays | **High** | Logged (Iter 7) |
| **A-23** | 2026-08-01 | Code Quality | Eliminate Redundant cv2.boundingRect Computations in Blob Extraction | **Medium** | Logged (Iter 8) |
| **A-24** | 2026-08-01 | Architecture | Extrapolate State for Coasting Tracks instead of Re-evaluating Stale History | **High** | Logged (Iter 8) |
| **A-25** | 2026-08-01 | Performance | Vectorize Kinematic Stop-Trigger Gating in Feature Extraction | **Medium** | Logged (Iter 8) |

---

## 2. Architecture & Algorithmic Enhancements

### Idea A-01: Global Optimal Bipartite Matching for Cross-Path Tracking
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Moderate coding effort, significantly improves tracking during crowded scenes)
- **Problem:** `ObstacleTracker.update()` currently uses greedy nearest-neighbor matching by iterating over active tracks and popping the closest unmatched detection. In crowded scenes where pedestrians cross paths, greedy matching often results in identity swaps or lost tracks because the match order depends on dictionary iteration order rather than globally minimizing the total distance across all track-detection pairs. Furthermore, `det_coords` is repeatedly converted to a NumPy array inside the track loop.
- **Proposed Solution:** Compute a full pairwise distance matrix between all active tracks and all new detections simultaneously. Use `scipy.optimize.linear_sum_assignment` (the Hungarian algorithm) to find the globally optimal bipartite matching that minimizes the sum of squared distances.
- **Expected Benefit:** Eliminates tracking ID swaps during cross-path scenarios, stabilizes velocity histories for closely interacting pedestrians, and reduces CPU overhead by vectorizing the distance matrix computation.

### Idea A-05: Thread-Safe Serial Writes to Prevent Packet Corruption
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 2)
- **ROI Tier:** **High ROI** (Low coding effort, ensures control stability)
- **Problem:** The `Rosmaster` class in `src/Rosmaster_Lib.py` provides numerous methods (e.g., `set_motor`, `set_beep`, `set_uart_servo`) that construct a command byte array and call `self.ser.write(cmd)`. However, there is no thread synchronization mechanism protecting the serial port. If the robot's main control loop (running in one thread) sends motor commands while a separate UI or sensor thread sends a beep or servo command simultaneously, the bytes of the two packets can interleave over the UART connection. This will result in corrupted frames, failed checksums, and dropped commands at the microcontroller.
- **Proposed Solution:** Introduce a `threading.Lock()` (e.g., `self._tx_lock = threading.Lock()`) in the `__init__` method. Wrap every `self.ser.write(cmd)` call within a `with self._tx_lock:` block to guarantee atomic packet transmission.
- **Expected Benefit:** Eliminates sporadic packet loss and undefined behavior caused by race conditions during concurrent hardware control.

### Idea A-09: Parameterize Hardcoded Camera Intrinsics and Constants
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 3)
- **ROI Tier:** **Medium ROI** (Moderate coding effort, improves portability)
- **Problem:** In `src/velocity_estimator.py`, variables like camera focal length (`fx = 277.0`), center coordinates (`cx`, `cy`), algorithm fallbacks (`Z = 1.0`), and detection areas (`MIN_BLOB_AREA = 500`) are hardcoded inside methods like `_extract_depth_centroids`. This makes it difficult to reuse the code on different robot platforms equipped with varying camera hardware (e.g., switching from Astra Pro to OAK-D) without modifying the source code directly.
- **Proposed Solution:** Refactor the code to load camera intrinsics and algorithm constants from a configuration file (e.g., a YAML config) or via ROS parameters. Inject these parameters through the `VelocityEstimator` constructor.
- **Expected Benefit:** Increases modularity and allows seamless deployment across heterogeneous robot fleets without hardcoded camera-specific values.

### Idea A-12: Decouple OpenCV Visualization from the Inference Loop
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 4)
- **ROI Tier:** **High ROI** (Low coding effort, decouples execution dependencies)
- **Problem:** In `src/velocity_estimator.py`, the `cv2.imshow` and `cv2.waitKey(1)` calls are executed directly inside the main `_inference_loop` when the `DISPLAY` environment variable is set. This means the critical velocity estimation rate (10Hz) is artificially throttled by the host's X11/Wayland UI rendering pipeline, causing variable latency and potentially missed LiDAR/Depth frames if the UI lags.
- **Proposed Solution:** Move the image rendering into an entirely separate thread or publish the annotated frame as a ROS Image topic. Have the main inference loop only push the latest annotated frame into a `queue.Queue` of size 1 (dropping old frames), and let the consumer thread handle `cv2.imshow`.
- **Expected Benefit:** Isolates the real-time velocity estimation loop from GUI rendering delays, ensuring a stable and fast 10Hz inference rate regardless of the system's graphics load.

### Idea A-14: Use Event-Driven Synchronization for Asynchronous UART Reads
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (Moderate coding effort, improves latency and CPU usage)
- **Problem:** Methods like `get_motion_pid()` and `get_uart_servo_value()` in `src/Rosmaster_Lib.py` implement blocking busy-wait loops (`time.sleep(0.001)`) to poll global class variables updated by the `__receive_data` thread. This leads to wasted CPU cycles, potential race conditions if two threads request data simultaneously, and unpredictable latency depending on when the OS schedules the Python thread.
- **Proposed Solution:** Implement proper thread synchronization primitives such as `threading.Event()` or `queue.Queue()`. When requesting data, the calling thread should wait on a specific `Event` or read from a response queue. The `__receive_data` thread will set the event or push to the queue upon parsing the specific response packet.
- **Expected Benefit:** Eliminates CPU-burning busy waits, guarantees thread safety for concurrent UART reads, and provides deterministic data retrieval latency.

### Idea A-16: Embed Scaling Operations Directly into the Model Graph
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (Low coding effort on training side, streamlines architecture)
- **Problem:** The `VelocityEstimator` loads scaling parameters from a separate `scaler_params.json` file and uses NumPy to explicitly scale input features (`features_scaled = (features_batch - self.scaler_X_mean) * self.scaler_X_inv_scale`) and inverse-scale the model's raw predictions back into meters per second. This necessitates shipping a fragile `.json` file alongside the TorchScript model and induces redundant conversions between NumPy arrays and PyTorch tensors.
- **Proposed Solution:** Embed the data scaling and inverse scaling operations directly inside the PyTorch model's `forward()` pass before exporting it to TorchScript (or ONNX). The model itself should accept unscaled meters and output unscaled velocity in m/s.
- **Expected Benefit:** Eliminates the dependency on `scaler_params.json`, simplifies the Python inference loop, avoids CPU-bound NumPy operations, and makes the model artifact completely self-contained.

### Idea A-21: Prioritize Tracking Proximal Obstacles When at Tracking Capacity
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 7)
- **ROI Tier:** **High ROI** (Low coding effort, improves collision avoidance reliability)
- **Problem:** When creating new tracks, `ObstacleTracker.update()` blindly iterates through the `unmatched` detections list and accepts new obstacles until the `MAX_OBSTACLES` limit is reached. In a crowded environment, it is highly possible that the algorithm will fill up the tracked slots with people 4.0 meters away while actively ignoring a new obstacle that just stepped in front of the robot at 0.5 meters.
- **Proposed Solution:** Before the loop that creates new tracks for unmatched detections, sort the `unmatched` list in ascending order of their Euclidean distance from the robot (e.g., using their Z depth or `hypot(x, y)`). 
- **Expected Benefit:** Ensures the estimator prioritizes tracking the most imminent and dangerous collision threats when constrained by compute limits.

### Idea A-24: Extrapolate State for Coasting Tracks instead of Re-evaluating Stale History
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 8)
- **ROI Tier:** **High ROI** (Moderate coding effort, prevents hallucinated velocities during occlusions)
- **Problem:** If a tracked obstacle is briefly occluded, its track `age` increments, but the tracking algorithm does not drop it immediately. However, the `_inference_loop` blindly pushes the track's stale history into the MLP again to estimate velocity. Because the history hasn't received new coordinate points, calculating differentials on this stale history produces physically unrealistic or zero-velocity predictions that jolt the local planner.
- **Proposed Solution:** If a track's `age > 0` (it was not seen this frame), skip the MLP inference. Instead, perform a simple linear kinematic extrapolation (or use a Kalman filter step) based on the last known valid `vx` and `vy`. Feed the newly extrapolated coordinate back into the `history_global` buffer so the track smoothly coasts.
- **Expected Benefit:** Maintains smooth tracking and velocity estimates when pedestrians are temporarily hidden by pillars or other objects, avoiding jerky, sudden zero-velocity updates.

---

## 3. Performance & Execution Efficiency

### Idea A-02: Connected Components for Zero-Overhead Depth Blob Extraction
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Low coding effort, reduces image processing latency)
- **Problem:** In `_extract_depth_centroids`, `cv2.findContours` is used to identify obstacle blobs, followed by `cv2.moments` and `cv2.boundingRect` in a Python loop to find the centroid. For complex or noisy depth masks, contour tracing is CPU-intensive. Additionally, cropping the contour mask (`cv2.drawContours`) allocates a new NumPy array and performs rasterization for every detected object, driving up garbage collection overhead.
- **Proposed Solution:** Replace `cv2.findContours` and the contour-masking loop with `cv2.connectedComponentsWithStats`. This single highly-optimized C++ call directly returns the bounding boxes, areas, and centroids for all blobs in one pass without tracing perimeters. Use the bounding box directly to slice the depth frame.
- **Expected Benefit:** Reduces vision processing latency by 30-50% during crowded scenes and eliminates per-obstacle NumPy array allocations and rasterization calls.

### Idea A-03: Chunked Serial Payload Reading for Reduced Syscall Overhead
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Low coding effort, frees up kernel resources)
- **Problem:** The serial receive thread `__receive_data` in `src/Rosmaster_Lib.py` reads data one byte at a time using `self.ser.read()`. Each call triggers a kernel syscall and Python GIL context switch. Worse, each byte is wrapped in a new `bytearray(self.ser.read())[0]` object. For a 20-byte payload, this causes 20 syscalls and 20 temporary object allocations per packet, capping the maximum effective telemetry parsing rate and wasting CPU cycles.
- **Proposed Solution:** Once the header and `ext_len` are parsed, read the entire remaining payload in a single block: `payload = self.ser.read(ext_len - 2)`. Alternatively, read all available bytes into a circular bytearray buffer and parse packets out of the buffer synchronously.
- **Expected Benefit:** Drastically reduces kernel syscall overhead and Python object allocations, allowing the Jetson's CPU to spend more time on Nav2 and inference rather than serial I/O.

### Idea A-04: Vectorized Struct Unpacking for Telemetry Parsing
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Low coding effort, accelerates UART data ingestion)
- **Problem:** In `__parse_data` of `src/Rosmaster_Lib.py`, data extraction is performed by repeatedly slicing the `ext_data` list and constructing a new `bytearray` for every single field: `struct.unpack('h', bytearray(ext_data[0:2]))[0]`. For IMU data (`FUNC_REPORT_ICM_RAW`), this happens 9 times per packet, creating massive object churn and slowing down telemetry ingestion.
- **Proposed Solution:** Convert the entire payload to `bytes` once: `data_bytes = bytes(ext_data)`. Then unpack all related fields simultaneously using `struct.unpack_from`. For example, `vx, vy, vz, voltage = struct.unpack_from('<hhhB', data_bytes, 0)`.
- **Expected Benefit:** Accelerates UART parsing by over 5x, eliminates hundreds of temporary `bytearray` allocations per second, and prevents the serial thread from becoming a bottleneck during high-frequency IMU telemetry (100Hz).

### Idea A-06: Graceful Thread Termination and Non-Blocking I/O
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 2)
- **ROI Tier:** **High ROI** (Low coding effort, prevents hangs)
- **Problem:** The `__receive_data` thread in `src/Rosmaster_Lib.py` runs an infinite `while True:` loop with blocking `self.ser.read()` calls. If the main program attempts to exit, this daemon thread will abruptly terminate, potentially leaving the serial port in an unpredictable state or causing resource leaks. Additionally, there is no way to gracefully stop the background thread during a controlled shutdown.
- **Proposed Solution:** Configure the `serial.Serial` instance with a read timeout (e.g., `timeout=0.1`). Replace `while True:` with `while self._running:`, and check if `self.ser.in_waiting > 0` or rely on the timeout to yield control periodically. Add a `stop()` method to cleanly toggle the flag, wait for the thread to join, and close the serial port.
- **Expected Benefit:** Prevents zombie processes, ensures clean hardware disengagement on shutdown, and avoids freezing the Python interpreter during program termination.

### Idea A-07: Eliminate O(N^2) List Insertions in Window Padding
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 2)
- **ROI Tier:** **High ROI** (Low coding effort, speeds up feature building)
- **Problem:** In `_build_window_features` in `src/velocity_estimator.py`, if a track has fewer than `WINDOW_SIZE` (10) frames, the history is padded using `hist.insert(0, ...)` inside a `while` loop. Because `hist` is a Python list at this stage, `insert(0, ...)` forces all existing elements to shift in memory, resulting in an O(N^2) operation. While `WINDOW_SIZE` is small, this operation occurs at 10Hz for every new obstacle, accumulating unnecessary CPU overhead.
- **Proposed Solution:** Use `collections.deque` with `appendleft()` (which has O(1) time complexity) or pad the list using list multiplication/concatenation: `hist = [pad_value] * pad_length + hist`.
- **Expected Benefit:** Eliminates O(N^2) memory shifting overhead, improving the feature extraction speed for newly tracked objects.

### Idea A-08: Refactor Duplicated UART Transmission Logic into a Helper Method
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 3)
- **ROI Tier:** **High ROI** (Low coding effort, improves code maintainability)
- **Problem:** In `src/Rosmaster_Lib.py`, every hardware control method (e.g., `set_motor`, `set_beep`, `set_pwm_servo`) independently calculates the packet checksum, appends it, writes to the serial port, conditionally prints debug info, and calls `time.sleep()`. This results in excessive code duplication and makes the file harder to maintain. If threading logic (Idea A-05) needs to be added, it must currently be injected into 20+ different methods.
- **Proposed Solution:** Consolidate the transmission logic into a single private helper method `def _send_command(self, cmd, debug_name):` that handles checksumming, serial writes, locks, and logging. Update all hardware methods to call this helper instead of repeating the same logic.
- **Expected Benefit:** Significantly reduces code redundancy and minimizes the potential for synchronization bugs by centralizing UART transmission logic.

### Idea A-10: INT8/FP16 Quantization for the Velocity MLP Model
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 3)
- **ROI Tier:** **High ROI** (Low effort if supported by PyTorch Mobile/TensorRT, speeds up inferences)
- **Problem:** The `velocity_mlp.torchscript` model in `src/velocity_estimator.py` uses FP32 precision during inference. On embedded platforms like the Jetson Orin Nano, running simple MLP models in FP32 fails to take advantage of the hardware's efficient half-precision (FP16) or integer (INT8) compute capabilities, resulting in unnecessary memory bandwidth consumption and higher latency.
- **Proposed Solution:** Convert or quantize the TorchScript model to FP16 or INT8 (e.g., using PyTorch's dynamic quantization `torch.quantization.quantize_dynamic` or exporting to TensorRT). Make the `VelocityEstimator` load the optimized model variant.
- **Expected Benefit:** Drastically reduces model size in memory and lowers inference time, freeing up more CPU/GPU cycles for path planning (Nav2) without noticeable loss of prediction accuracy.

### Idea A-11: Migrate MLP Inference to ONNX Runtime / TensorRT
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 4)
- **ROI Tier:** **High ROI** (Moderate coding effort, drastic latency reduction)
- **Problem:** The `velocity_estimator.py` currently loads a TorchScript model and uses the PyTorch CPU backend to run inference. Loading PyTorch inside Python introduces a massive memory footprint (often >300MB) and initialization overhead. For a lightweight MLP running on an embedded Jetson device, executing via PyTorch is inefficient and slower than necessary.
- **Proposed Solution:** Export the trained PyTorch MLP to ONNX format (`torch.onnx.export`). Replace the `torch` imports in `velocity_estimator.py` with `onnxruntime` (or TensorRT bindings) and execute the model using `onnxruntime.InferenceSession()`.
- **Expected Benefit:** Cuts Python memory consumption drastically and reduces inference execution time from milliseconds to microseconds, maximizing Jetson resource utilization for other robotic tasks.

### Idea A-13: Eliminate Bare Except Clauses and Implement Robust UART Error Handling
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 4)
- **ROI Tier:** **High ROI** (Low coding effort, improves system reliability)
- **Problem:** Throughout `src/Rosmaster_Lib.py`, methods like `set_motor` or `set_car_run` wrap their entire logic in `try...except:` blocks that catch all exceptions and simply execute `pass` after printing an error string. This silent failure pattern hides critical serial communication drops from the higher-level navigation system, potentially leaving the robot stuck in its last motor state if the UART connection dies.
- **Proposed Solution:** Change the bare `except:` clauses to catch specific exceptions like `except serial.SerialException as e:`. Utilize Python's `logging` module to report the actual error traceback. More importantly, allow critical connection exceptions to propagate (or raise a custom exception) so the parent ROS node can gracefully halt the robot or attempt an automatic reconnection to the microcontroller.
- **Expected Benefit:** Ensures transparent hardware error reporting, preventing dangerous scenarios where the navigation stack thinks the robot is moving but the commands are silently failing over serial.

### Idea A-15: Remove Redundant Runtime JIT Tracing
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (Trivial coding effort, fixes potential bugs)
- **Problem:** In `velocity_estimator.py`, after loading the TorchScript model via `torch.jit.load()`, the script attempts to trace it again at runtime using `torch.jit.trace(self._model, dummy_input)`. Re-tracing an already compiled ScriptModule is completely redundant, slows down initialization, and can strip away dynamic control flow logic or cause unpredictable behavior in the computation graph.
- **Proposed Solution:** Delete the entire `torch.jit.trace` block inside `_load_model()`. The model is already optimized when exported and saved.
- **Expected Benefit:** Speeds up the node initialization time and prevents obscure PyTorch execution bugs caused by tracing a loaded artifact.

### Idea A-17: Optimize OpenCV Morphological Operations with Separable/Smaller Kernels
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 6)
- **ROI Tier:** **Medium ROI** (Trivial coding effort, lowers image processing cost)
- **Problem:** In `velocity_estimator.py`'s `_extract_depth_centroids`, a 5x5 elliptical structural element is used twice (for `MORPH_OPEN` and `MORPH_CLOSE`) over the entire depth frame mask. Full-frame morphological operations with large non-separable kernels are relatively expensive on low-end CPUs and are likely overkill considering `connectedComponentsWithStats` already filters out small noise blobs.
- **Proposed Solution:** Shrink the kernel size to 3x3, use a computationally cheaper rectangular kernel `cv2.MORPH_RECT`, or drop the morphology completely and rely strictly on the `MIN_BLOB_AREA` parameter during the connected components extraction phase.
- **Expected Benefit:** Reclaims CPU cycles during the dense computer vision pre-processing pipeline, keeping the estimator lightweight and fast.

### Idea A-18: Vectorize Sliding Window Features using NumPy Instead of Python Loops
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 6)
- **ROI Tier:** **High ROI** (Low coding effort, improves feature extraction speed)
- **Problem:** Building the `(1, 40)` sliding window feature vector in `_build_window_features` (inside `velocity_estimator.py`) is currently achieved using a Python `for` loop that iterates over the tracker history, computes translation offsets byte-by-byte, applies bounds clipping, and appends to a standard list before finally casting to a NumPy array.
- **Proposed Solution:** Convert the entire history deque to a NumPy array in one fell swoop: `arr = np.array(hist)`. Then calculate positional differences using `np.diff(arr, axis=0)`, apply translation normalization via broadcasting, and use `np.clip` directly on the entire resulting matrix before calling `.flatten()`.
- **Expected Benefit:** Replaces slow Python loops with optimized C-level NumPy array operations, vastly reducing the overhead per detected obstacle at every frame.

### Idea A-19: Optimize Checksum Validation using Native sum() in UART parsing
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 6)
- **ROI Tier:** **High ROI** (Low coding effort, accelerates UART receive thread)
- **Problem:** In `__receive_data` inside `Rosmaster_Lib.py`, the packet checksum is calculated using a `while` loop that reads bytes one by one and computes `check_sum = check_sum + value`. Doing sequential arithmetic inside a Python byte loop is notoriously slow. When combined with block reading (Idea A-03), this loop becomes an unnecessary bottleneck.
- **Proposed Solution:** After implementing block reading (`payload = self.ser.read(ext_len - 2)`), validate the packet using Python's highly optimized built-in `sum()` function: `check_sum = (ext_len + ext_type + sum(payload[:-1])) % 256`.
- **Expected Benefit:** Significantly speeds up the UART telemetry ingestion thread, preventing packet drops or pipeline saturation when receiving high-frequency IMU data.

### Idea A-20: Avoid Pulling Colorized depth_frame if Visualization is Disabled
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 7)
- **ROI Tier:** **Medium ROI** (Trivial coding effort, reduces USB bandwidth/driver CPU load)
- **Problem:** The `_inference_loop` in `velocity_estimator.py` fetches both `depth_frame` (colored, scaled) and `raw_depth_frame` (uint16 raw values) every single cycle from the camera driver. The BGR colorized `depth_frame` is completely ignored during algorithmic extraction if `raw_depth_frame` is available. Generating this colored depth frame wastes the camera node's CPU cycles (via `cv2.applyColorMap`) and USB bandwidth.
- **Proposed Solution:** Only request `depth_frame` from the camera driver conditionally if `DISPLAY` is actually set in the environment variables and visualization is required.
- **Expected Benefit:** Reduces host CPU consumption on BGR mapping scaling operations and decreases the overhead in retrieving unused frames.

### Idea A-22: Use Command Queue Instead of Blind time.sleep Post-Write Delays
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 7)
- **ROI Tier:** **High ROI** (Moderate coding effort, drastically reduces control pipeline lag)
- **Problem:** Every time a hardware command (like `set_motor`) is called in `Rosmaster_Lib.py`, it calculates the payload, calls `self.ser.write()`, and then blindly blocks the calling thread with `time.sleep(self.__delay_time)` (usually 2ms). If a ROS navigation node sends 10 consecutive commands (e.g., servo updates, beeps, motor updates), it locks the thread for 20ms+, accumulating latency in the real-time path planning pipeline.
- **Proposed Solution:** Offload UART writes to a dedicated transmission thread fed by a thread-safe `queue.Queue`. The transmission thread can pull commands and enforce the 2ms hardware breather between writes, returning execution instantly to the main Python/ROS thread.
- **Expected Benefit:** Unblocks the main navigation process instantly after dispatching commands, preventing hardware I/O constraints from slowing down obstacle avoidance logic.

### Idea A-23: Eliminate Redundant cv2.boundingRect Computations in Blob Extraction
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 8)
- **ROI Tier:** **Medium ROI** (Trivial coding effort, reduces bounding box calculation overhead)
- **Problem:** Inside the contour iteration loop of `_extract_depth_centroids`, `cv2.boundingRect(cnt)` is called twice for the exact same obstacle contour. First, it is called to compute a reference depth to scale the minimum area filter, and then it is called again 10 lines later to crop the raw depth frame.
- **Proposed Solution:** Cache the result of the first `cv2.boundingRect(cnt)` call into local variables and reuse those variables for the cropping phase instead of recalculating the bounding rectangle geometry.
- **Expected Benefit:** Provides a free micro-optimization in the vision processing pipeline by avoiding redundant C++ module calls per contour.

### Idea A-25: Vectorize Kinematic Stop-Trigger Gating in Feature Extraction
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 8)
- **ROI Tier:** **Medium ROI** (Low coding effort, aligns with Idea A-18 refactor)
- **Problem:** The kinematic stop-trigger algorithm iterates over the last 3 tracker frames and manually computes point-to-point displacement using `math.hypot(dx, dy)` in a Python loop to determine if an object is fully stationary. This is needlessly slow compared to native matrix math.
- **Proposed Solution:** Alongside the NumPy vectorization planned in Idea A-18, replace the `is_stopped` loop with a single line of NumPy math: `is_stopped = np.all(np.linalg.norm(np.diff(arr[-3:], axis=0), axis=1) < 0.01)`.
- **Expected Benefit:** Ensures the entire feature preparation step in `_build_window_features` runs purely in C space, reducing Python interpretation overhead.
