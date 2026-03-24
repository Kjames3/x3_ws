/* =================================================================
   Application State
   ================================================================= */
const state = {
    ws: null,
    connected: false,
    detectionEnabled: false,
    depthEnabled: false,
    lidarEnabled: false,
    autoDriveEnabled: false, // Local toggle tracking
    isAutoDriving: false,    // Server state
    isDemoMode: false,       // Demo cycling mode
    gamepadIndex: null,
    lastLeftPower: 0,
    lastRightPower: 0,
    lastVx: 0,
    lastVy: 0,
    lastOmega: 0,
    sessionStartTime: 0,
    sessionTimerInterval: null,

    // Data Buffer for Render Loop
    latestData: {
        readout: null,        // Motor positions, power, image
        robotPose: null,
        targetPose: null,
        trajectory: null,
        lidarPoints: null,
        detections: [],
        fps: { cam: 0, yolo: 0 },
        battery: null
    },

    // Flags for dirty checking (optional optimization)
    needsLidarUpdate: false,
    needs3DUpdate: false,

    // Nav Metrics tracking
    trackedModelName: null,
    movingFrameCount: 0,
    movingDetectionCount: 0,

    // Logging
    logThrottle: 0
};

const DEFAULT_PORT = 8081;

// =================================================================
// DOM Elements
// =================================================================
const elements = {
    // Connection
    connectBtn: document.getElementById('connect-btn'),
    disconnectBtn: document.getElementById('disconnect-btn'),
    statusDot: document.getElementById('status-dot'),
    statusText: document.getElementById('status-text'),
    controlArea: document.getElementById('control-area'),
    robotIp: document.getElementById('robot-ip'),

    // Navigation Metrics
    navMetricsOverlay: document.getElementById('nav-metrics-overlay'),
    metricModel: document.getElementById('metric-model'),
    metricLabel: document.getElementById('metric-label'),
    metricFrames: document.getElementById('metric-frames'),
    metricRatio: document.getElementById('metric-ratio'),
    metricLatency: document.getElementById('metric-latency'),

    // Camera & Detection
    cameraFeed: document.getElementById('camera-feed'),
    cameraPlaceholder: document.getElementById('camera-placeholder'),
    depthFeed: document.getElementById('depth-feed'),
    depthPanel: document.getElementById('depth-panel'),
    depthPlaceholder: document.getElementById('depth-placeholder'),
    depthToggle: document.getElementById('depth-toggle'),
    detectionToggle: document.getElementById('detection-toggle'),
    detectionPanel: document.getElementById('detection-panel'),
    detectionCount: document.getElementById('detection-count'),
    detectionList: document.getElementById('detection-list'),
    autoDriveBtn: document.getElementById('auto-drive-btn'),
    autoDriveWrapper: document.getElementById('auto-drive-wrapper'),
    demoModeBtn: document.getElementById('demo-mode-btn'),
    demoBanner: document.getElementById('demo-banner'),
    demoBannerRole: document.getElementById('demo-banner-role'),
    demoBannerModel: document.getElementById('demo-banner-model'),

    // Power Stats
    powerStatsPanel: document.getElementById('power-stats'),
    statVolts: document.getElementById('stat-volts'),
    statAmps: document.getElementById('stat-amps'),
    statWatts: document.getElementById('stat-watts'),
    statUptime: document.getElementById('stat-uptime'),
    statTimeRemaining: document.getElementById('stat-time-remaining'),
    timeRemainingContainer: document.getElementById('time-remaining-container'),

    // Motor Readouts (Status Section)
    m1Pos: document.getElementById('m1-pos'),
    m1Power: document.getElementById('m1-power'),
    m2Pos: document.getElementById('m2-pos'),
    m2Power: document.getElementById('m2-power'),
    m3Pos: document.getElementById('m3-pos'),
    m3Power: document.getElementById('m3-power'),
    m4Pos: document.getElementById('m4-pos'),
    m4Power: document.getElementById('m4-power'),

    // FPS
    fpsDisplay: document.getElementById('fps-display'),
    fpsCamera: document.getElementById('fps-camera'),
    fpsDetection: document.getElementById('fps-detection'),
    fpsDetectionWrapper: document.getElementById('fps-detection-wrapper'),
    fpsCameraInline: document.getElementById('fps-camera-inline'),
    fpsYoloInline: document.getElementById('fps-yolo-inline'),

    // Position Section
    robotX: document.getElementById('robot-x'),
    robotY: document.getElementById('robot-y'),
    robotTheta: document.getElementById('robot-theta'),
    startX: document.getElementById('start-x'),
    startY: document.getElementById('start-y'),
    startMsg: document.getElementById('start-msg'),
    targetX: document.getElementById('target-x'),
    targetY: document.getElementById('target-y'),
    targetDist: document.getElementById('target-dist'),

    // Power (Detailed Section)
    powerVoltage: document.getElementById('power-voltage'),
    powerCurrent: document.getElementById('power-current'),
    powerWatts: document.getElementById('power-watts'),
    powerBatteryPct: document.getElementById('power-battery-pct'),
    powerTimeRemaining: document.getElementById('power-time-remaining'),

    // Lidar
    lidarToggle: document.getElementById('lidar-toggle'),
    lidarCanvas: document.getElementById('lidar-canvas'),
    lidarCtx: document.getElementById('lidar-canvas') ? document.getElementById('lidar-canvas').getContext('2d') : null,

    // Motors Controls
    leftSlider: document.getElementById('left-slider'),
    leftSliderValue: document.getElementById('left-slider-value'),
    leftFill: document.getElementById('left-fill'),
    leftThumb: document.getElementById('left-thumb'),

    rightSlider: document.getElementById('right-slider'),
    rightSliderValue: document.getElementById('right-slider-value'),
    rightFill: document.getElementById('right-fill'),
    rightThumb: document.getElementById('right-thumb'),

    stopBtn: document.getElementById('stop-btn'),

    // Image Capture
    captureBtn: document.getElementById('capture-btn'),
    captureCount: document.getElementById('capture-count'),
    downloadImagesBtn: document.getElementById('download-images-btn'),
    blurSweepBtn: document.getElementById('blur-sweep-btn'),
    goldenSetBtn: document.getElementById('golden-set-btn'),

    // WSAD / D-Pad
    btnW: document.getElementById('btn-w'),
    btnA: document.getElementById('btn-a'),
    btnS: document.getElementById('btn-s'),
    btnD: document.getElementById('btn-d'),
    keyboardToggle: document.getElementById('keyboard-toggle'),

    // Controller
    controllerName: document.getElementById('controller-name'),
    controllerStatus: document.getElementById('controller-status'),
    gamepadIndicator: document.getElementById('gamepad-indicator'),
    gamepadStatusText: document.getElementById('gamepad-status-text'),

    // 3D Viewport
    viewportContainer: document.getElementById('viewport-3d-container'),
    viewportCanvas: document.getElementById('viewport-3d-canvas'),
    navPhaseDisplay: document.getElementById('nav-phase-display'),

    // Mode badge & Gazebo launcher
    modeBadge: document.getElementById('mode-badge'),
    modeBadgeLabel: document.getElementById('mode-badge-label'),
    launchGazeboBtn: document.getElementById('launch-gazebo-btn'),
};

// =================================================================
// Class Filter Toggle Helper
// =================================================================
function updateClassFilterBtn(btn, allClasses) {
    if (allClasses) {
        btn.textContent = '🌐 All Classes';
        btn.style.color = '#4ade80';
        btn.style.borderColor = '#4ade80';
    } else {
        btn.textContent = '🥫 Cans Only';
        btn.style.color = 'var(--accent-warning, #facc15)';
        btn.style.borderColor = 'var(--accent-warning, #facc15)';
    }
}

function updateLabelToggleBtn(btn, labelsOn) {
    if (labelsOn) {
        btn.textContent = '🏷️ Labels On';
        btn.style.color = '#60a5fa';   // blue
        btn.style.borderColor = '#60a5fa';
    } else {
        btn.textContent = '🏷️ Labels Off';
        btn.style.color = '#6b7280';   // grey
        btn.style.borderColor = '#6b7280';
    }
}

// =================================================================
// Initialization
// =================================================================
document.addEventListener('DOMContentLoaded', () => {
    // Initialize 3D Viewport with delay to ensure layout
    setTimeout(init3DViewport, 100);

    // Initialize logic
    const savedIP = localStorage.getItem('viam_robot_ip');
    if (savedIP && elements.robotIp) {
        elements.robotIp.value = savedIP;
    }

    const modelSelect = document.getElementById('model-select');
    if (modelSelect) {
        modelSelect.addEventListener('change', (e) => {
            const selectedModel = e.target.value;
            sendMessage({
                type: "set_model",
                model: selectedModel
            });
        });
    }

    // Class filter toggle — Cans Only ↔ All Classes
    const classFilterBtn = document.getElementById('class-filter-toggle');
    if (classFilterBtn) {
        let allClassesActive = false;
        classFilterBtn.addEventListener('click', () => {
            allClassesActive = !allClassesActive;
            sendMessage({ type: "set_classes", all_classes: allClassesActive });
            updateClassFilterBtn(classFilterBtn, allClassesActive);
        });
    }

    // Label overlay toggle — Labels On ↔ Labels Off
    const labelToggleBtn = document.getElementById('label-toggle');
    if (labelToggleBtn) {
        let labelsOn = true;
        labelToggleBtn.addEventListener('click', () => {
            labelsOn = !labelsOn;
            sendMessage({ type: "set_labels", show_labels: labelsOn });
            updateLabelToggleBtn(labelToggleBtn, labelsOn);
        });
    }

    // Data capture toolbar expand/collapse
    const dataToolsToggle = document.getElementById('data-tools-toggle');
    const cameraToolbarSecondary = document.getElementById('camera-toolbar-secondary');
    if (dataToolsToggle && cameraToolbarSecondary) {
        dataToolsToggle.addEventListener('click', () => {
            const isVisible = cameraToolbarSecondary.style.display !== 'none';
            cameraToolbarSecondary.style.display = isVisible ? 'none' : 'flex';
            dataToolsToggle.textContent = isVisible ? '📸 Data ▾' : '✖ Close';
        });
    }

    // Init Visuals
    updateVisuals(0, elements.leftFill, elements.leftThumb);
    updateVisuals(0, elements.rightFill, elements.rightThumb);

    // Start Render Loop
    requestAnimationFrame(renderLoop);
});

// =================================================================
// Render Loop (Centralized Update)
// =================================================================
function renderLoop(timestamp) {
    if (state.connected) {
        // 1. Poll Gamepad
        pollGamepad();

        // 2. Update UI from latest data
        updateUI();

        // 3. Draw Lidar
        if (state.lidarEnabled && state.needsLidarUpdate) {
            drawLidar(state.latestData.lidarPoints);
            state.needsLidarUpdate = false;
        }
    }

    // 4. Update 3D Scene (Always update controls/render if initialized)
    if (renderer3D && scene3D && camera3D) {
        controls3D.update();
        if (state.needs3DUpdate && state.latestData.robotPose) {
            update3DSceneContent();
            state.needs3DUpdate = false;
        }
        renderer3D.render(scene3D, camera3D);
    }

    requestAnimationFrame(renderLoop);
}

// =================================================================
// WebSocket Connection
// =================================================================
function connect() {
    if (state.ws && state.ws.readyState === WebSocket.OPEN) return;

    updateConnectionStatus('connecting');
    const serverUrl = getServerAddress();
    console.log(`Connecting to ${serverUrl}`);
    state.ws = new WebSocket(serverUrl);

    state.ws.onopen = () => {
        state.connected = true;
        state.sessionStartTime = Date.now();
        updateConnectionStatus('connected');
        if (elements.controlArea) elements.controlArea.classList.remove('disabled-overlay');

        if (elements.robotIp && elements.robotIp.value) {
            localStorage.setItem('viam_robot_ip', elements.robotIp.value);
        }

        // Start session timer
        if (state.sessionTimerInterval) clearInterval(state.sessionTimerInterval);
        state.sessionTimerInterval = setInterval(updateSessionTimer, 1000);
    };

    state.ws.onclose = () => {
        state.connected = false;
        updateConnectionStatus('disconnected');
        if (elements.controlArea) elements.controlArea.classList.add('disabled-overlay');
        state.ws = null;
    };

    state.ws.onerror = () => {
        state.connected = false;
        updateConnectionStatus('error');
        if (elements.controlArea) elements.controlArea.classList.add('disabled-overlay');
        state.ws = null;
    };

    state.ws.onmessage = (event) => {
        // P3: binary frame = raw JPEG camera image (no base64 round-trip)
        if (event.data instanceof Blob) {
            if (elements.cameraFeed) {
                const oldSrc = elements.cameraFeed.src;
                elements.cameraFeed.src = URL.createObjectURL(event.data);
                elements.cameraFeed.style.display = 'block';
                if (elements.cameraPlaceholder) elements.cameraPlaceholder.style.display = 'none';
                if (oldSrc.startsWith('blob:')) URL.revokeObjectURL(oldSrc);
            }
            return;
        }
        try {
            const data = JSON.parse(event.data);
            handleMessage(data);
        } catch (e) {
            console.error("Message parse error:", e);
        }
    };
}

function getServerAddress() {
    const hostInput = elements.robotIp;
    if (hostInput && hostInput.value && hostInput.value !== '192.168.1.X') {
        return `ws://${hostInput.value}:${DEFAULT_PORT}`;
    }
    return `ws://besto.local:${DEFAULT_PORT}`;
}

function updateConnectionStatus(status) {
    if (!elements.statusDot) return;
    elements.statusDot.className = 'status-dot';

    switch (status) {
        case 'connected':
            elements.statusDot.classList.add('connected');
            if (elements.statusText) elements.statusText.textContent = 'Connected';
            if (elements.connectBtn) {
                elements.connectBtn.textContent = 'Connected';
                elements.connectBtn.disabled = true;
            }
            if (elements.disconnectBtn) elements.disconnectBtn.style.display = 'inline-flex';
            break;
        case 'connecting':
            elements.statusDot.classList.add('connecting');
            if (elements.statusText) elements.statusText.textContent = 'Connecting...';
            if (elements.connectBtn) {
                elements.connectBtn.textContent = 'Connecting...';
                elements.connectBtn.disabled = true;
            }
            break;
        case 'error':
            if (elements.statusText) elements.statusText.textContent = 'Connection failed';
            if (elements.connectBtn) {
                elements.connectBtn.textContent = 'Retry';
                elements.connectBtn.disabled = false;
            }
            break;
        default:
            if (elements.statusText) elements.statusText.textContent = 'Disconnected';
            if (elements.connectBtn) {
                elements.connectBtn.textContent = 'Connect';
                elements.connectBtn.disabled = false;
            }
            if (elements.disconnectBtn) elements.disconnectBtn.style.display = 'none';
    }
}

function sendMessage(data) {
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send(JSON.stringify(data));
    }
}

// =================================================================
// Data Handling
// =================================================================
function handleMessage(data) {
    if (data.type === "hello") {
        const mode = data.mode; // "sim" | "ros2" | "direct"
        const label = elements.modeBadgeLabel;
        const badge = elements.modeBadge;
        const gazeboBtn = elements.launchGazeboBtn;
        if (!badge || !label) return;

        const styles = {
            sim:    { text: 'SIM',    bg: '#92400e', color: '#fde68a', border: '#f59e0b' },
            ros2:   { text: 'ROS2',   bg: '#1e3a5f', color: '#93c5fd', border: '#3b82f6' },
            direct: { text: 'DIRECT', bg: '#14532d', color: '#86efac', border: '#22c55e' },
        };
        const s = styles[mode] || styles.direct;
        label.textContent = s.text;
        label.style.background = s.bg;
        label.style.color = s.color;
        label.style.border = `1px solid ${s.border}`;
        badge.style.display = 'flex';

        if (gazeboBtn) {
            gazeboBtn.style.display = mode === 'sim' ? 'inline-block' : 'none';
        }
        return;
    }

    if (data.type === "launch_gazebo_result") {
        const btn = elements.launchGazeboBtn;
        if (data.success) {
            if (btn) { btn.textContent = '⏳ Launching…'; btn.disabled = true; }
            setTimeout(() => {
                if (btn) { btn.textContent = '🚀 Gazebo'; btn.disabled = false; }
            }, 15000);
        } else {
            alert(`Gazebo: ${data.msg}`);
        }
        return;
    }

    if (data.type === "readout") {
        // Update State Buffer
        state.latestData.readout = data;
        state.latestData.robotPose = data.robot_pose;
        state.latestData.targetPose = data.target_pose;
        state.latestData.trajectory = data.trajectory; // 3D Trajectory

        if (data.lidar_points) {
            state.latestData.lidarPoints = data.lidar_points;
            state.needsLidarUpdate = true;
        }

        if (data.detections !== undefined) {
            state.latestData.detections = data.detections;
        }

        if (data.fps_camera !== undefined) state.latestData.fps.cam = data.fps_camera;
        if (data.fps_detection !== undefined) state.latestData.fps.yolo = data.fps_detection;

        if (data.battery) state.latestData.battery = data.battery;

        // Sync flags
        state.detectionEnabled = data.detection_enabled;
        state.isAutoDriving = data.is_auto_driving;
        if (data.is_demo_mode !== undefined) {
            state.isDemoMode = data.is_demo_mode;
            updateDemoBannerFromReadout(data);
        }
        state.latestData.navPhase = data.nav_phase;
        state.latestData.autoDriveStart = data.auto_drive_start;
        state.latestData.power = data.power; // Power stats from INA219

        state.needs3DUpdate = true;

        // Telemetry Logging (Throttled ~1Hz at 20fps)
        state.logThrottle++;
        if (state.logThrottle % 20 === 0) {
            const rp = data.robot_pose;
            const tp = data.target_pose;
            const thetaDeg = (rp.theta * 180 / Math.PI).toFixed(1);

            console.groupCollapsed(`🤖 State: ${data.nav_phase || 'IDLE'}`);
            console.log(`📍 Pose:   X=${rp.x.toFixed(1)}, Y=${rp.y.toFixed(1)}, θ=${thetaDeg}°`);
            if (tp && tp.x !== null) {
                console.log(`🎯 Target: X=${tp.x.toFixed(1)}, Y=${tp.y.toFixed(1)}, Dist=${tp.distance_cm.toFixed(0)}cm`);
            } else {
                console.log(`🎯 Target: None`);
            }
            console.log(`⚙️ Motors: L=${data.left_power.toFixed(2)}, R=${data.right_power.toFixed(2)}`);
            console.log(`🔋 Power:  ${data.power ? data.power.voltage.toFixed(2) + 'V' : '--'}`);
            console.log(`⚙️ Motors: L=${data.left_power.toFixed(2)}, R=${data.right_power.toFixed(2)}`);
            console.groupEnd();
        }

        // Piped Server Logging
        if (data.latest_log && data.latest_log.time > (state.lastLogTime || 0)) {
            console.log(`%c🐍 SERVER: ${data.latest_log.msg}`, "color: #4ade80; font-weight: bold;");

            // Auto-reset Golden Set button if complete
            if (data.latest_log.msg.includes("Golden collection COMPLETE") || data.latest_log.msg.includes("STOPPED")) {
                if (elements.goldenSetBtn) {
                    elements.goldenSetBtn.textContent = '✨ Golden Set: Start';
                    window.goldenSetActive = false;
                }
            }

            state.lastLogTime = data.latest_log.time;
        }

    } else if (data.type === "capture_response") {
        const category = data.category || "saved";
        const dist = data.distance_cm ? ` (${Math.round(data.distance_cm)}cm)` : "";
        if (elements.captureCount) elements.captureCount.textContent = `${data.count} total • ${category}${dist}`;

    } else if (data.type === "model_changed") {
        const modelSelect = document.getElementById('model-select');
        const label = modelSelect ? modelSelect.options[modelSelect.selectedIndex].text : data.model;
        if (data.success) {
            console.log(`%c✅ Model loaded: ${label}`, "color: #4ade80; font-weight: bold;");
        } else {
            const reason = data.error || 'Unknown error';
            console.warn(`❌ Model FAILED to load: ${label}\n   Reason: ${reason}\n   Path tried: ${data.path}`);
        }
        // Flash the dropdown border green/red so there's a visible cue
        if (modelSelect) {
            modelSelect.style.transition = 'border-color 0.3s';
            modelSelect.style.borderColor = data.success ? '#4ade80' : '#f87171';
            setTimeout(() => { modelSelect.style.borderColor = ''; }, 2000);
        }
        // Sync the class-filter button to whatever the server now has active
        if (data.success && data.all_classes !== undefined) {
            const btn = document.getElementById('class-filter-toggle');
            if (btn) updateClassFilterBtn(btn, data.all_classes);
        }

    } else if (data.type === "demo_model_changed") {
        updateDemoBannerFromEvent(data);

    } else if (data.type === "demo_status") {
        console.log(`[DEMO] ${data.msg}`);

    } else if (data.type === "classes_updated") {
        // Server confirmed the class filter changed
        const btn = document.getElementById('class-filter-toggle');
        if (btn) updateClassFilterBtn(btn, data.all_classes);
        console.log(`%c🔍 Class filter: ${data.all_classes ? 'All COCO classes' : 'Cans only'}`, 'color: #60a5fa;');

    } else if (data.type === "labels_updated") {
        const btn = document.getElementById('label-toggle');
        if (btn) updateLabelToggleBtn(btn, data.show_labels);
        console.log(`%c🏷️ Labels: ${data.show_labels ? 'ON' : 'OFF'}`, 'color: #60a5fa;');

    } else if (data.type === "download_images_response") {
        handleDownloadResponse(data);

    } else if (data.type === "blur_dataset_response") {
        handleBlurResponse(data);
    }
}

function updateUI() {
    const data = state.latestData.readout;
    if (!data) return;

    // 1. Motor Readouts
    if (elements.m1Pos) elements.m1Pos.textContent = (data.m1_pos ?? 0);
    if (elements.m1Power) elements.m1Power.textContent = `${Math.round((data.m1_power ?? 0) * 100)}%`;
    if (elements.m2Pos) elements.m2Pos.textContent = (data.m2_pos ?? 0);
    if (elements.m2Power) elements.m2Power.textContent = `${Math.round((data.m2_power ?? 0) * 100)}%`;
    if (elements.m3Pos) elements.m3Pos.textContent = (data.m3_pos ?? 0);
    if (elements.m3Power) elements.m3Power.textContent = `${Math.round((data.m3_power ?? 0) * 100)}%`;
    if (elements.m4Pos) elements.m4Pos.textContent = (data.m4_pos ?? 0);
    if (elements.m4Power) elements.m4Power.textContent = `${Math.round((data.m4_power ?? 0) * 100)}%`;

    // 2. Camera image is now sent as a binary WebSocket frame (P3) — handled in onmessage

    // 2b. Depth Image
    if (data.depth_image && elements.depthFeed) {
        elements.depthFeed.src = "data:image/jpeg;base64," + data.depth_image;
        elements.depthFeed.style.display = 'block';
        if (elements.depthPlaceholder) elements.depthPlaceholder.style.display = 'none';
    }

    // 3. FPS
    if (elements.fpsDisplay) {
        elements.fpsDisplay.style.display = 'block';
        if (elements.fpsCamera) elements.fpsCamera.textContent = state.latestData.fps.cam.toFixed(1);
        if (elements.fpsCameraInline) elements.fpsCameraInline.textContent = state.latestData.fps.cam.toFixed(0);

        if (state.detectionEnabled) {
            if (elements.fpsDetectionWrapper) elements.fpsDetectionWrapper.style.display = 'inline';
            if (elements.fpsDetection) elements.fpsDetection.textContent = state.latestData.fps.yolo.toFixed(1);
            if (elements.fpsYoloInline) elements.fpsYoloInline.textContent = state.latestData.fps.yolo.toFixed(0);
        } else {
            if (elements.fpsDetectionWrapper) elements.fpsDetectionWrapper.style.display = 'none';
            if (elements.fpsYoloInline) elements.fpsYoloInline.textContent = '--';
        }
    }

    // 4. Nav Metrics Update
    if (state.detectionEnabled && data.active_model_name !== undefined) {
        if (elements.navMetricsOverlay) elements.navMetricsOverlay.style.display = 'flex';

        // Reset tracking if model changes
        if (state.trackedModelName !== data.active_model_name) {
            state.trackedModelName = data.active_model_name;
            state.movingFrameCount = 0;
            state.movingDetectionCount = 0;
        }

        // Track moving frames
        const isMoving = Math.abs(data.left_power) > 0.05 || Math.abs(data.right_power) > 0.05;
        if (isMoving) {
            state.movingFrameCount++;
            if (data.detections && data.detections.length > 0) {
                state.movingDetectionCount++;
            }
        }

        const ratio = state.movingFrameCount > 0 ? (state.movingDetectionCount / state.movingFrameCount) * 100 : 0;
        const detectingLabel = data.detections && data.detections.length > 0 ? data.detections[0].label : "--";
        const latency = data.inference_latency_ms !== undefined ? data.inference_latency_ms.toFixed(1) : "--";

        if (elements.metricModel) elements.metricModel.textContent = data.active_model_name;
        if (elements.metricLabel) elements.metricLabel.textContent = detectingLabel;
        if (elements.metricFrames) elements.metricFrames.textContent = state.movingFrameCount;
        if (elements.metricRatio) elements.metricRatio.textContent = ratio.toFixed(1) + "%";
        if (elements.metricLatency) elements.metricLatency.textContent = latency + " ms";
    } else {
        if (elements.navMetricsOverlay) elements.navMetricsOverlay.style.display = 'none';
    }

    // 5. Detections UI
    if (data.detections !== undefined) {
        updateDetectionsList(state.latestData.detections);
    }

    // 5. Buttons State
    if (state.detectionEnabled !== undefined && elements.detectionToggle) {
        elements.detectionToggle.classList.toggle('active', state.detectionEnabled);
        if (elements.detectionPanel) elements.detectionPanel.style.display = state.detectionEnabled ? 'block' : 'none';

        // Auto Drive Wrapper visibility
        if (elements.autoDriveWrapper) {
            state.detectionEnabled ? elements.autoDriveWrapper.classList.add('visible') : elements.autoDriveWrapper.classList.remove('visible');
        }
    }

    if (state.isAutoDriving !== undefined) {
        updateAutoDriveButton();
    }

    // 6. Position & Power
    updatePositionUI();
    updatePowerUI();
}

function updateDetectionsList(detections) {
    if (!elements.detectionCount || !elements.detectionList) return;
    elements.detectionCount.textContent = detections.length;

    if (detections.length === 0) {
        elements.detectionList.innerHTML = '<div class="no-detections">No objects detected</div>';
        return;
    }

    const html = detections.map(d => {
        const distInches = (d.distance_cm / 2.54).toFixed(1);
        const distFeet = (d.distance_cm / 30.48).toFixed(2);
        return `
        <div class="detection-item">
            <div class="detection-item-label">
                <span class="detection-badge">${d.label}</span>
            </div>
            <div class="detection-item-stats">
                <span title="Distance">${d.distance_cm.toFixed(0)}cm / ${distInches}in / ${distFeet}ft</span>
                <span title="Confidence">${(d.confidence * 100).toFixed(0)}%</span>
                <div class="confidence-bar" title="Confidence">
                    <div class="confidence-fill" style="width: ${d.confidence * 100}%"></div>
                </div>
            </div>
        </div>
    `;
    }).join('');

    elements.detectionList.innerHTML = html;
}

function updatePositionUI() {
    // Robot
    if (state.latestData.robotPose) {
        const rp = state.latestData.robotPose;
        if (elements.robotX) elements.robotX.textContent = `X: ${rp.x.toFixed(1)}`;
        if (elements.robotY) elements.robotY.textContent = `Y: ${rp.y.toFixed(1)}`;
        const thetaDeg = (rp.theta * 180 / Math.PI).toFixed(1);
        if (elements.robotTheta) elements.robotTheta.textContent = `θ: ${thetaDeg}°`;
    }

    // Target
    const tp = state.latestData.targetPose;
    if (tp && tp.x !== null) {
        if (elements.targetX) elements.targetX.textContent = `X: ${tp.x.toFixed(1)}`;
        if (elements.targetY) elements.targetY.textContent = `Y: ${tp.y.toFixed(1)}`;

        if (state.latestData.robotPose) {
            const dx = tp.x - state.latestData.robotPose.x;
            const dy = tp.y - state.latestData.robotPose.y;
            const dist = Math.sqrt(dx * dx + dy * dy);
            const distIn = (dist / 2.54).toFixed(1);
            if (elements.targetDist) elements.targetDist.textContent = `Dist: ${dist.toFixed(0)}cm (${distIn}in)`;
        }
    } else {
        if (elements.targetX) elements.targetX.textContent = 'X: --';
        if (elements.targetY) elements.targetY.textContent = 'Y: --';
        if (elements.targetDist) elements.targetDist.textContent = 'Dist: --';
    }

    // Start
    const sp = state.latestData.autoDriveStart;
    if (sp) {
        if (elements.startX) elements.startX.textContent = `X: ${sp.x.toFixed(1)}`;
        if (elements.startY) elements.startY.textContent = `Y: ${sp.y.toFixed(1)}`;
        if (elements.startMsg) {
            elements.startMsg.textContent = 'Auto-Drive Origin';
            elements.startMsg.style.color = 'var(--accent-green)';
        }
    }
}

function updatePowerUI() {
    const pwr = state.latestData.power;
    if (!pwr) return;

    if (elements.powerVoltage) elements.powerVoltage.textContent = pwr.voltage.toFixed(2) + ' V';
    if (elements.powerCurrent) elements.powerCurrent.textContent = pwr.current.toFixed(2) + ' A';
    if (elements.powerWatts) elements.powerWatts.textContent = pwr.power.toFixed(1) + ' W';

    if (elements.powerBatteryPct) {
        const pct = pwr.battery_pct;
        elements.powerBatteryPct.textContent = pct.toFixed(0) + '%';

        if (pct > 50) elements.powerBatteryPct.style.color = 'var(--accent-green)';
        else if (pct > 20) elements.powerBatteryPct.style.color = 'var(--accent-yellow)';
        else elements.powerBatteryPct.style.color = 'var(--accent-red)';
    }

    // Estimate Time
    if (elements.powerTimeRemaining && pwr.current > 0.1) {
        const BATTERY_CAPACITY_AH = 6.0;
        const remainingCapacity = (pwr.battery_pct / 100.0) * BATTERY_CAPACITY_AH;
        const hoursRemaining = remainingCapacity / pwr.current;
        const totalMinutes = Math.floor(hoursRemaining * 60);
        const hours = Math.floor(totalMinutes / 60);
        const mins = totalMinutes % 60;

        if (hours > 0) elements.powerTimeRemaining.textContent = `${hours}h ${mins}m`;
        else elements.powerTimeRemaining.textContent = `${mins} min`;

        if (totalMinutes > 60) elements.powerTimeRemaining.style.color = 'var(--accent-green)';
        else if (totalMinutes > 20) elements.powerTimeRemaining.style.color = 'var(--accent-yellow)';
        else elements.powerTimeRemaining.style.color = 'var(--accent-red)';
    } else if (elements.powerTimeRemaining) {
        elements.powerTimeRemaining.textContent = '--';
    }

    // Fallback Battery Voltage Logic (for header)
    if (state.latestData.battery) { // Legacy battery object
        const voltage = state.latestData.battery.voltage;
        if (elements.statVolts) elements.statVolts.textContent = voltage.toFixed(2) + ' V';
        if (elements.statAmps) elements.statAmps.textContent = state.latestData.battery.amps.toFixed(3) + ' A';
        if (elements.statWatts) elements.statWatts.textContent = state.latestData.battery.watts.toFixed(2) + ' W';

        // Simple linear check for header time remaining
        const LOW_VOLTAGE_THRESHOLD = 12.2;
        const CRITICAL_VOLTAGE = 11.8;

        if (voltage <= LOW_VOLTAGE_THRESHOLD && elements.statTimeRemaining) {
            const sub = voltage - CRITICAL_VOLTAGE;
            const range = LOW_VOLTAGE_THRESHOLD - CRITICAL_VOLTAGE;
            const pct = Math.max(0, sub) / range;
            const secs = Math.floor(pct * 150); // 2.5 mins
            const m = Math.floor(secs / 60);
            const s = secs % 60;
            elements.statTimeRemaining.textContent = `${m}:${s.toString().padStart(2, '0')}`;
            elements.statTimeRemaining.style.color = 'var(--accent-red)';
        } else if (elements.statTimeRemaining) {
            elements.statTimeRemaining.textContent = 'OK';
            elements.statTimeRemaining.style.color = 'var(--accent-green)';
        }
    }
}


function updateSessionTimer() {
    if (!state.connected || !elements.statUptime) return;
    const elapsed = Math.floor((Date.now() - state.sessionStartTime) / 1000);
    const mins = Math.floor(elapsed / 60);
    const secs = elapsed % 60;
    elements.statUptime.textContent = `${mins}:${secs.toString().padStart(2, '0')}`;
}

// =================================================================
// Three.js 3D Visualization (Optimized)
// =================================================================
let scene3D, camera3D, renderer3D, controls3D;
let robotMesh, targetMarker, startMarker, trajectoryLine;
// Reusable buffers
const MAX_TRAJECTORY_POINTS = 500;
let trajectoryGeometry;

function init3DViewport() {
    const container = elements.viewportContainer;
    const canvas = elements.viewportCanvas;

    if (!container || !canvas || typeof THREE === 'undefined') {
        console.warn('Three.js not loaded or container not found');
        return;
    }

    // Scene
    scene3D = new THREE.Scene();
    scene3D.background = new THREE.Color(0x1e293b);

    // Camera
    camera3D = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 1000);
    camera3D.position.set(0, 2, 2);
    camera3D.lookAt(0, 0, 0);

    // Renderer — wrap in try/catch: WebGL may be unavailable in some
    // environments (software renderer, remote desktop, headless browser).
    try {
        renderer3D = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    } catch (e) {
        console.warn('WebGL context creation failed — hiding 3D viewport.', e);
        const card = document.getElementById('viewport-3d-card');
        if (card) card.style.display = 'none';
        return;
    }
    renderer3D.setSize(container.clientWidth, container.clientHeight);
    renderer3D.setPixelRatio(window.devicePixelRatio);

    // Controls
    controls3D = new THREE.OrbitControls(camera3D, renderer3D.domElement);
    controls3D.enableDamping = true;
    controls3D.dampingFactor = 0.1;
    controls3D.maxPolarAngle = Math.PI / 2.1;

    // Grid
    const gridHelper = new THREE.GridHelper(5, 50, 0x475569, 0x334155);
    scene3D.add(gridHelper);

    // Lighting
    scene3D.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(2, 4, 2);
    scene3D.add(dirLight);

    // Robot (Box)
    const boxGeometry = new THREE.BoxGeometry(0.25, 0.08, 0.15);
    const robotMaterial = new THREE.MeshStandardMaterial({ color: 0x06b6d4, metalness: 0.3, roughness: 0.7 });
    robotMesh = new THREE.Mesh(boxGeometry, robotMaterial);
    robotMesh.position.set(0, 0.04, 0);
    scene3D.add(robotMesh);

    // Robot Arrow
    const arrowGeom = new THREE.ConeGeometry(0.03, 0.08, 8);
    const arrowMat = new THREE.MeshStandardMaterial({ color: 0xffffff });
    const arrow = new THREE.Mesh(arrowGeom, arrowMat);
    arrow.rotation.x = -Math.PI / 2;
    arrow.position.set(0, 0.04, 0.11);
    robotMesh.add(arrow);

    // Start Marker
    const startGeom = new THREE.CylinderGeometry(0.05, 0.05, 0.01, 16);
    const startMat = new THREE.MeshStandardMaterial({ color: 0x22c55e });
    startMarker = new THREE.Mesh(startGeom, startMat);
    startMarker.position.set(0, 0.005, 0);
    scene3D.add(startMarker);

    // Target Marker
    const targetGeom = new THREE.CylinderGeometry(0.08, 0.08, 0.2, 16);
    const targetMat = new THREE.MeshStandardMaterial({ color: 0xef4444, transparent: true, opacity: 0.7 });
    targetMarker = new THREE.Mesh(targetGeom, targetMat);
    targetMarker.visible = false;
    scene3D.add(targetMarker);

    // Trajectory Line (Pre-allocated BufferGeometry)
    trajectoryGeometry = new THREE.BufferGeometry();
    const positions = new Float32Array(MAX_TRAJECTORY_POINTS * 3); // 3 vertices per point
    trajectoryGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    trajectoryGeometry.setDrawRange(0, 0);
    const trajMat = new THREE.LineBasicMaterial({ color: 0x00ff88, linewidth: 2 });
    trajectoryLine = new THREE.Line(trajectoryGeometry, trajMat);
    scene3D.add(trajectoryLine);

    // Resize Handler
    window.addEventListener('resize', () => {
        if (container && camera3D && renderer3D) {
            camera3D.aspect = container.clientWidth / container.clientHeight;
            camera3D.updateProjectionMatrix();
            renderer3D.setSize(container.clientWidth, container.clientHeight);
        }
    });

    console.log('✓ 3D Viewport initialized');
}

function update3DSceneContent() {
    const robotPose = state.latestData.robotPose;
    const targetPose = state.latestData.targetPose;
    const trajectory = state.latestData.trajectory;
    const navPhase = state.latestData.navPhase;

    if (!robotMesh) return;

    // Update Robot
    robotMesh.position.x = robotPose.x / 100;
    robotMesh.position.z = robotPose.y / 100;
    robotMesh.rotation.y = -robotPose.theta;

    // Update Target
    if (targetPose && targetPose.x !== null) {
        targetMarker.position.x = targetPose.x / 100;
        targetMarker.position.z = targetPose.y / 100;
        targetMarker.visible = true;
    } else {
        targetMarker.visible = false;
    }

    // Update Trajectory (Zero Allocations)
    if (trajectory && trajectory.length > 0 && trajectoryGeometry) {
        const positions = trajectoryGeometry.attributes.position.array;
        let count = 0;
        const max = Math.min(trajectory.length, MAX_TRAJECTORY_POINTS);

        for (let i = 0; i < max; i++) {
            positions[count * 3] = trajectory[i].x / 100;     // x
            positions[count * 3 + 1] = 0.02;                  // y (height)
            positions[count * 3 + 2] = trajectory[i].y / 100; // z
            count++;
        }

        trajectoryGeometry.setDrawRange(0, count);
        trajectoryGeometry.attributes.position.needsUpdate = true;
    } else {
        trajectoryGeometry.setDrawRange(0, 0);
    }

    // Update Phase UI
    if (elements.navPhaseDisplay && navPhase) {
        elements.navPhaseDisplay.textContent = navPhase;
    }
}


// =================================================================
// Lidar Rendering
// =================================================================
function drawLidar(points) {
    if (!elements.lidarCtx) return;
    const canvas = elements.lidarCanvas;
    const ctx = elements.lidarCtx;
    const width = canvas.width;
    const height = canvas.height;
    const cx = width / 2;
    const cy = height / 2;
    const scale = 75; // pixels per meter

    // Clear
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, width, height);

    // Grid
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    [1, 2].forEach(r => {
        ctx.beginPath();
        ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
        ctx.stroke();
    });

    ctx.beginPath();
    ctx.moveTo(0, cy);
    ctx.lineTo(width, cy);
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, height);
    ctx.stroke();

    // Points — flat [x0,y0,x1,y1,…] format (P4)
    ctx.fillStyle = '#22c55e';
    for (let i = 0; i < points.length; i += 2) {
        const x = cx - (points[i + 1] * scale);
        const y = cy - (points[i]     * scale);
        ctx.fillRect(x - 1, y - 1, 2, 2);
    }

    // Robot Center
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
}


// =================================================================
// Controls (Sliders, Buttons)
// =================================================================
function updateVisuals(value, fill, thumb) {
    // Normalize -100 to 100 -> 0% to 100%
    const pct = (parseInt(value) + 100) / 200;
    const pctString = (pct * 100) + "%";

    if (thumb) thumb.style.bottom = pctString;

    if (value >= 0) {
        fill.style.bottom = "50%";
        fill.style.height = (value / 2) + "%";
        fill.style.backgroundColor = "var(--accent-green)";
    } else {
        const absVal = Math.abs(value);
        fill.style.bottom = pctString;
        fill.style.height = (absVal / 2) + "%";
        fill.style.backgroundColor = "var(--accent-cyan)";
    }
}

function updateMotor(motor, value) {
    if (!state.connected || state.isAutoDriving) return;
    sendMessage({
        type: "set_power",
        motor: motor,
        power: value / 100.0
    });
}

function autoDriveToggle() {
    if (!state.connected) return;
    const msgType = state.isAutoDriving ? "stop_auto_drive" : "start_auto_drive";
    sendMessage({ type: msgType });
}

function updateAutoDriveButton() {
    if (state.isAutoDriving) {
        elements.autoDriveBtn.textContent = "Stop Auto-Drive";
        elements.autoDriveBtn.classList.add("btn-danger");
        elements.autoDriveBtn.classList.remove("btn-primary");
    } else {
        elements.autoDriveBtn.textContent = "Start Auto-Drive";
        elements.autoDriveBtn.classList.remove("btn-danger");
        elements.autoDriveBtn.classList.add("btn-primary");
    }
}

function demoModeToggle() {
    if (!state.connected) return;
    if (state.isDemoMode) {
        sendMessage({ type: "stop_demo" });
        state.isDemoMode = false;
        updateDemoModeButton(false);
        hideDemoBanner();
    } else {
        sendMessage({ type: "start_demo", interval: 30 });
        state.isDemoMode = true;
        updateDemoModeButton(true);
    }
}

function updateDemoModeButton(active) {
    const btn = elements.demoModeBtn;
    if (!btn) return;
    if (active) {
        btn.textContent = "⏹ Stop Demo";
        btn.style.background = "#7c3aed";
        btn.style.color = "#fff";
        btn.style.borderColor = "#7c3aed";
    } else {
        btn.textContent = "🎬 Demo Mode";
        btn.style.background = "var(--bg-tertiary)";
        btn.style.color = "#a78bfa";
        btn.style.borderColor = "#a78bfa";
    }
}

function updateDemoBannerFromEvent(data) {
    const banner = elements.demoBanner;
    if (!banner) return;
    if (data.success) {
        banner.style.display = "flex";
        if (elements.demoBannerRole) {
            elements.demoBannerRole.textContent = data.role === "teacher" ? "TEACHER" : "STUDENT (KD)";
            elements.demoBannerRole.style.color = data.role === "teacher" ? "#fca5a5" : "#86efac";
        }
        if (elements.demoBannerModel) elements.demoBannerModel.textContent = data.display_name;
    }
}

function updateDemoBannerFromReadout(data) {
    if (data.is_demo_mode && data.demo_current_model) {
        if (elements.demoBanner) elements.demoBanner.style.display = "flex";
        updateDemoModeButton(true);
    } else if (!data.is_demo_mode) {
        hideDemoBanner();
        updateDemoModeButton(false);
    }
}

function hideDemoBanner() {
    if (elements.demoBanner) elements.demoBanner.style.display = "none";
}

// Event Listeners (Setup)
if (elements.connectBtn) elements.connectBtn.addEventListener('click', () => state.connected ? (state.ws.close()) : connect());
if (elements.launchGazeboBtn) elements.launchGazeboBtn.addEventListener('click', () => {
    if (state.ws && state.connected) state.ws.send(JSON.stringify({ type: 'launch_gazebo' }));
});
if (elements.disconnectBtn) elements.disconnectBtn.addEventListener('click', () => { if (state.ws) { state.ws.send(JSON.stringify({ type: "disconnect" })); state.ws.close(); } });

if (elements.leftSlider) {
    elements.leftSlider.addEventListener('input', (e) => {
        const val = e.target.value;
        elements.leftSliderValue.textContent = val;
        updateVisuals(val, elements.leftFill, elements.leftThumb);
        updateMotor('left', parseInt(val));
    });
}

if (elements.rightSlider) {
    elements.rightSlider.addEventListener('input', (e) => {
        const val = e.target.value;
        elements.rightSliderValue.textContent = val;
        updateVisuals(val, elements.rightFill, elements.rightThumb);
        updateMotor('right', parseInt(val));
    });
}

if (elements.autoDriveBtn) elements.autoDriveBtn.addEventListener('click', autoDriveToggle);
if (elements.demoModeBtn) elements.demoModeBtn.addEventListener('click', demoModeToggle);

if (elements.stopBtn) elements.stopBtn.addEventListener('click', () => {
    if (!state.connected) return;
    sendMessage({ type: "stop" });
    if (state.isAutoDriving) sendMessage({ type: "stop_auto_drive" });
    if (state.isDemoMode) {
        sendMessage({ type: "stop_demo" });
        state.isDemoMode = false;
        updateDemoModeButton(false);
        hideDemoBanner();
    }

    // Reset UI
    if (elements.leftSlider) { elements.leftSlider.value = 0; elements.leftSliderValue.textContent = "0"; updateVisuals(0, elements.leftFill, elements.leftThumb); }
    if (elements.rightSlider) { elements.rightSlider.value = 0; elements.rightSliderValue.textContent = "0"; updateVisuals(0, elements.rightFill, elements.rightThumb); }
});

if (elements.depthToggle) elements.depthToggle.addEventListener('click', () => {
    if (!state.connected) return;
    state.depthEnabled = !state.depthEnabled;
    elements.depthToggle.classList.toggle('active', state.depthEnabled);
    if (elements.depthPanel) elements.depthPanel.style.display = state.depthEnabled ? 'block' : 'none';
    sendMessage({ type: "toggle_depth", enabled: state.depthEnabled });
});

if (elements.detectionToggle) elements.detectionToggle.addEventListener('click', () => {
    if (!state.connected) return;
    const newState = !state.detectionEnabled;
    state.detectionEnabled = newState; // Optimistic
    sendMessage({ type: "toggle_detection", enabled: newState });
});

if (elements.lidarToggle) elements.lidarToggle.addEventListener('click', () => {
    state.lidarEnabled = !state.lidarEnabled;
    elements.lidarToggle.classList.toggle('active', state.lidarEnabled);
    if (state.connected) sendMessage({ type: "toggle_lidar", enabled: state.lidarEnabled });
    if (!state.lidarEnabled && elements.lidarCtx) {
        elements.lidarCtx.fillStyle = '#000';
        elements.lidarCtx.fillRect(0, 0, elements.lidarCanvas.width, elements.lidarCanvas.height);
    }
});

// Camera Actions
if (elements.captureBtn) elements.captureBtn.addEventListener('click', () => {
    if (!state.connected) return;
    sendMessage({ type: "capture_image" });
    elements.captureBtn.disabled = true;
    elements.captureBtn.textContent = '⏳';
    setTimeout(() => { elements.captureBtn.disabled = false; elements.captureBtn.textContent = '📸 Capture'; }, 500);
});

if (elements.downloadImagesBtn) elements.downloadImagesBtn.addEventListener('click', () => {
    if (!state.connected) return;
    const shouldClear = confirm("Do you want to DELETE these images from the robot after downloading?\n\nOK = Download & Delete\nCancel = Download Only");
    elements.downloadImagesBtn.disabled = true;
    elements.downloadImagesBtn.textContent = '⏳ Preparing...';
    sendMessage({ type: "download_images", clear: shouldClear });
});

if (elements.blurSweepBtn) elements.blurSweepBtn.addEventListener('click', () => {
    if (!state.connected) return;
    if (confirm("Ensure can is fixed distance. Robot will capture ~25 images. Start?")) {
        sendMessage({ type: "collect_blur_dataset" });
    }
});

window.goldenSetActive = false;
if (elements.goldenSetBtn) {
    elements.goldenSetBtn.addEventListener('click', () => {
        if (!state.connected) return;
        window.goldenSetActive = !window.goldenSetActive;
        if (window.goldenSetActive) {
            elements.goldenSetBtn.textContent = '⏹️ Golden Set: Stop';
            sendMessage({ type: "start_golden_collection" });
        } else {
            elements.goldenSetBtn.textContent = '✨ Golden Set: Start';
            sendMessage({ type: "stop_golden_collection" });
        }
    });
}

function handleDownloadResponse(data) {
    elements.downloadImagesBtn.disabled = false;
    elements.downloadImagesBtn.textContent = '💾 Download All';

    if (data.success && data.zip_data) {
        const byteCharacters = atob(data.zip_data);
        const byteNumbers = new Array(byteCharacters.length);
        for (let i = 0; i < byteCharacters.length; i++) byteNumbers[i] = byteCharacters.charCodeAt(i);
        const byteArray = new Uint8Array(byteNumbers);
        const blob = new Blob([byteArray], { type: 'application/zip' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = data.filename || 'training_images.zip';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    } else {
        alert(data.error || 'No images to download');
    }
}

function handleBlurResponse(data) {
    if (data.status === "started") {
        elements.blurSweepBtn.disabled = true;
        elements.blurSweepBtn.textContent = "📸 Sweeping...";
    } else if (data.status === "complete") {
        elements.blurSweepBtn.disabled = false;
        elements.blurSweepBtn.textContent = "🌊 Blur Sweep";
    } else if (data.status === "error") {
        elements.blurSweepBtn.disabled = false;
        elements.blurSweepBtn.textContent = "🌊 Blur Sweep";
        alert("Sweep failed: " + data.message);
    }
}

// =================================================================
// Gamepad & Input
// =================================================================
// =================================================================
// Gamepad Widget
// =================================================================
function showGamepadWidget(show) {
    const widget    = document.getElementById('gamepad-widget');
    const sliders   = document.querySelector('.sliders-container');
    if (!widget) return;
    if (show) {
        widget.style.display = 'flex';
        if (sliders) sliders.style.visibility = 'hidden';
    } else {
        widget.style.display = 'none';
        if (sliders) sliders.style.visibility = '';
    }
}

function drawGamepadWidget(axes, buttons) {
    const canvas = document.getElementById('gamepad-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);

    const R = 38;           // outer circle radius
    const DOT = 7;          // stick-dot radius
    const DEAD = 0.12;      // dead-zone fraction
    const centres = [
        { x: 60,  y: 56, axX: 0, axY: 1, label: 'ROT/STR' },
        { x: 160, y: 56, axX: 2, axY: 3, label: 'FWD/STR' },
    ];

    centres.forEach(({ x, y, axX, axY, label }) => {
        // Outer ring
        ctx.beginPath();
        ctx.arc(x, y, R, 0, Math.PI * 2);
        ctx.strokeStyle = '#334155';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Dead-zone ring
        ctx.beginPath();
        ctx.arc(x, y, R * DEAD * 3.5, 0, Math.PI * 2);
        ctx.strokeStyle = '#475569';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Stick dot
        const ax = axes[axX] || 0;
        const ay = axes[axY] || 0;
        const dx = ax * (R - DOT - 2);
        const dy = ay * (R - DOT - 2);
        ctx.beginPath();
        ctx.arc(x + dx, y + dy, DOT, 0, Math.PI * 2);
        ctx.fillStyle = '#60a5fa';
        ctx.fill();

        // Label
        ctx.fillStyle = '#64748b';
        ctx.font = '9px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(label, x, y + R + 13);
    });

    // Button pills
    const pillDefs = [
        { idx: 0, label: '✕ Stop',   color: '#f87171' },
        { idx: 2, label: '□ Detect', color: '#a78bfa' },
        { idx: 3, label: '△ Auto',   color: '#34d399' },
    ];
    const pillContainer = document.getElementById('gamepad-btns');
    if (pillContainer) {
        if (!pillContainer.children.length) {
            // Build pills once
            pillDefs.forEach(({ label }, i) => {
                const pill = document.createElement('span');
                pill.id = `gp-btn-${i}`;
                pill.style.cssText = 'padding: 0.15rem 0.5rem; border-radius: 999px; border: 1px solid #334155; background: var(--bg-secondary); color: #64748b; transition: all 0.1s;';
                pill.textContent = label;
                pillContainer.appendChild(pill);
            });
        }
        pillDefs.forEach(({ idx, color }, i) => {
            const pill = document.getElementById(`gp-btn-${i}`);
            if (!pill) return;
            const pressed = buttons[idx] && buttons[idx].pressed;
            pill.style.background = pressed ? color : 'var(--bg-secondary)';
            pill.style.color      = pressed ? '#0f172a' : '#64748b';
            pill.style.borderColor = pressed ? color : '#334155';
        });
    }
}

window.addEventListener("gamepadconnected", (e) => {
    state.gamepadIndex = e.gamepad.index;
    if (elements.controllerName) elements.controllerName.textContent = e.gamepad.id.substring(0, 30);
    if (elements.gamepadIndicator) elements.gamepadIndicator.classList.add('connected');
    if (elements.gamepadStatusText) elements.gamepadStatusText.textContent = '✓ Connected';
    showGamepadWidget(true);
});

window.addEventListener("gamepaddisconnected", (e) => {
    if (state.gamepadIndex === e.gamepad.index) {
        state.gamepadIndex = null;
        if (elements.controllerName) elements.controllerName.textContent = "No controller";
        if (elements.gamepadIndicator) elements.gamepadIndicator.classList.remove('connected');
        if (elements.gamepadStatusText) elements.gamepadStatusText.textContent = 'No Controller';
        showGamepadWidget(false);
    }
});

const buttonDebounce = { square: false, triangle: false };

function pollGamepad() {
    if (state.gamepadIndex === null || !state.connected) return;
    const gamepad = navigator.getGamepads()[state.gamepadIndex];
    if (!gamepad) return;

    // 1. E-Stop (X / Cross / Button 0)
    if (gamepad.buttons[0].pressed) {
        sendMessage({ type: "stop" });
        if (state.isAutoDriving) sendMessage({ type: "stop_auto_drive" });
        return;
    }

    // 2. Square (Toggle Detection)
    if (gamepad.buttons[2].pressed && !buttonDebounce.square) {
        buttonDebounce.square = true;
        const newState = !state.detectionEnabled;
        state.detectionEnabled = newState;
        sendMessage({ type: "toggle_detection", enabled: newState });
    } else if (!gamepad.buttons[2].pressed) {
        buttonDebounce.square = false;
    }

    // 3. Triangle (Toggle Auto-Drive)
    if (gamepad.buttons[3].pressed && !buttonDebounce.triangle) {
        buttonDebounce.triangle = true;
        if (state.detectionEnabled) {
            autoDriveToggle();
        }
    } else if (!gamepad.buttons[3].pressed) {
        buttonDebounce.triangle = false;
    }

    // 4. Joystick Drive (Holonomic Mecanum)
    // Right stick: forward/backward (vx) + strafe left/right (vy)
    // Left stick:  rotate in place (omega)
    // Both sticks can be used simultaneously.
    // Only if NOT auto-driving
    if (!state.isAutoDriving) {
        const deadzone = 0.1;

        // Right stick
        let rx = gamepad.axes[2]; // Strafe X
        let ry = gamepad.axes[3]; // Forward/Back Y

        // Left stick
        let lx = gamepad.axes[0]; // Rotation X

        if (Math.abs(rx) < deadzone) rx = 0;
        if (Math.abs(ry) < deadzone) ry = 0;
        if (Math.abs(lx) < deadzone) lx = 0;

        const vx    = -ry;   // Right stick Y up → forward (positive vx)
        const vy    = -rx;   // Right stick X right → strafe right (inverted payload)
        const omega = -lx;   // Left stick X right → rotate CW (negative omega = CW)

        const vxR    = Math.round(vx    * 100) / 100;
        const vyR    = Math.round(vy    * 100) / 100;
        const omegaR = Math.round(omega * 100) / 100;

        if (Math.abs(vxR - state.lastVx) > 0.02 ||
            Math.abs(vyR - state.lastVy) > 0.02 ||
            Math.abs(omegaR - state.lastOmega) > 0.02) {

            sendMessage({ type: "set_move", vx: vxR, vy: vyR, omega: omegaR });
            state.lastVx    = vxR;
            state.lastVy    = vyR;
            state.lastOmega = omegaR;

            // Mirror approximate forward power to UI sliders
            const approxPower = Math.max(-1, Math.min(1, vxR));
            if (elements.leftSlider) {
                elements.leftSlider.value = Math.round(approxPower * 100);
                updateVisuals(elements.leftSlider.value, elements.leftFill, elements.leftThumb);
            }
            if (elements.rightSlider) {
                elements.rightSlider.value = Math.round(approxPower * 100);
                updateVisuals(elements.rightSlider.value, elements.rightFill, elements.rightThumb);
            }
        }
    }

    // Draw live stick + button visualization
    drawGamepadWidget(gamepad.axes, gamepad.buttons);
}

// Keyboard Support
const keysPressed = {};
window.addEventListener('keydown', (e) => {
    if (!state.connected) return;
    const key = e.key.toLowerCase();

    // Space = Capture
    if (key === ' ' && elements.captureBtn && !elements.captureBtn.disabled) {
        e.preventDefault();
        elements.captureBtn.click();
        return;
    }

    // WASD requires keyboard toggle
    if (!elements.keyboardToggle || !elements.keyboardToggle.checked) return;
    if (state.isAutoDriving) return;

    if (keysPressed[key]) return;
    keysPressed[key] = true;

    if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
        updateKeyboardDrive();
    }
});

window.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    keysPressed[key] = false;

    if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
        updateKeyboardDrive();
    }
});

function updateKeyboardDrive() {
    if (state.isAutoDriving) return;

    const w = keysPressed['w'] || keysPressed['arrowup'];
    const s = keysPressed['s'] || keysPressed['arrowdown'];
    const a = keysPressed['a'] || keysPressed['arrowleft'];
    const d = keysPressed['d'] || keysPressed['arrowright'];

    if (elements.btnW) w ? elements.btnW.classList.add('active') : elements.btnW.classList.remove('active');
    if (elements.btnS) s ? elements.btnS.classList.add('active') : elements.btnS.classList.remove('active');
    if (elements.btnA) a ? elements.btnA.classList.add('active') : elements.btnA.classList.remove('active');
    if (elements.btnD) d ? elements.btnD.classList.add('active') : elements.btnD.classList.remove('active');

    let lp = 0, rp = 0;
    const pwr = (elements.keyPower && elements.keyPower.value) ? parseFloat(elements.keyPower.value) : 0.5;

    if (w) { lp += pwr; rp += pwr; }
    if (s) { lp -= pwr; rp -= pwr; }
    if (a) { lp -= pwr; rp += pwr; } // Pivot Left
    if (d) { lp += pwr; rp -= pwr; } // Pivot Right

    lp = Math.max(-1, Math.min(1, lp));
    rp = Math.max(-1, Math.min(1, rp));

    // Send only if changed
    if (lp === 0 && rp === 0 && (state.lastLeftPower !== 0 || state.lastRightPower !== 0)) {
        sendMessage({ type: "stop" });
        state.lastLeftPower = 0; state.lastRightPower = 0;
    } else if (Math.abs(lp - state.lastLeftPower) > 0.01 || Math.abs(rp - state.lastRightPower) > 0.01) {
        sendMessage({ type: "set_power", motor: "left", power: lp });
        sendMessage({ type: "set_power", motor: "right", power: rp });
        state.lastLeftPower = lp; state.lastRightPower = rp;
    }

    // Update Sliders
    if (elements.leftSlider && elements.leftFill) {
        elements.leftSlider.value = Math.round(lp * 100);
        updateVisuals(elements.leftSlider.value, elements.leftFill, elements.leftThumb);
    }
    if (elements.rightSlider && elements.rightFill) {
        elements.rightSlider.value = Math.round(rp * 100);
        updateVisuals(elements.rightSlider.value, elements.rightFill, elements.rightThumb);
    }
}
