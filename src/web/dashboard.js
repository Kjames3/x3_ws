/* ==========================================================================
   X3 dashboard layer.

   Additive on purpose: main.js keeps ownership of the WebSocket, the canvases
   and every existing control. This file only

     - runs the tab bar,
     - renders the status strip and the diagnostics panel,
     - batches all of its DOM writes into one requestAnimationFrame commit,
     - adds auto-reconnect and a staleness watchdog to the main socket.

   It reads messages by wrapping the existing handler rather than opening a
   second socket, so there is exactly one connection and one source of truth.
   ========================================================================== */
(function () {
    'use strict';

    // ---------------------------------------------------------------- state
    const S = {
        readout: null,        // newest fast-lane message
        slow: null,           // newest slow-lane message
        sweep: null,          // newest sweep_config
        build: null,          // server build stamp from `hello`
        lastReadout: 0,       // ms timestamp of the last fast-lane message
        lastSlow: 0,
        msgCount: 0,          // messages since the last rate sample
        byteCount: 0,
        msgRate: 0,
        byteRate: 0,
        rateWindowStart: performance.now(),
        dirty: false,
        rafHandle: null,
        lastCommit: 0,   // drives the non-rAF heartbeat below
        // Gyro-z bias tracker. The chassis IMU's drift is a recurring fault on
        // this robot, so the strip shows a running mean rather than an instant
        // value: instantaneous rate is dominated by noise and tells you nothing.
        gyro: { sum: 0, n: 0, last: null },
    };

    const STALE_MS = 1500;    // fast lane runs at 20 Hz; 1.5 s is ~30 missed
    const SLOW_STALE_MS = 6000;

    // ------------------------------------------------------------- utilities
    const $ = (id) => document.getElementById(id);

    function fmt(v, digits, unit) {
        if (v === null || v === undefined || Number.isNaN(v)) return '--';
        return v.toFixed(digits) + (unit || '');
    }

    // Every DOM write goes through here so a value that has not changed costs
    // nothing. At 20 Hz across ~40 fields this is the difference between a
    // steady render and continuous layout thrash.
    function setText(el, text) {
        if (!el) return;
        if (el.__last !== text) {
            el.__last = text;
            el.textContent = text;
        }
    }

    function setClass(el, cls, on) {
        if (!el) return;
        el.classList.toggle(cls, !!on);
    }

    function tileState(id, level, value, sub) {
        const t = $('tile-' + id);
        if (!t) return;
        for (const c of ['ok', 'warn', 'bad', 'idle']) {
            if (c !== level) t.classList.remove(c);
        }
        t.classList.add(level);
        setText($('tile-' + id + '-value'), value);
        setText($('tile-' + id + '-sub'), sub);
    }

    function markStale(id, stale) {
        setClass($('tile-' + id), 'stale', stale);
    }

    // --------------------------------------------------------------- rAF loop
    function schedule() {
        S.dirty = true;
        if (S.rafHandle === null) {
            S.rafHandle = requestAnimationFrame(commit);
        }
    }

    function commit() {
        S.rafHandle = null;
        if (!S.dirty) return;
        S.dirty = false;
        S.lastCommit = performance.now();
        try {
            render();
        } catch (e) {
            console.error('[dashboard] render failed', e);
        }
    }

    // The strip must keep updating even with no traffic, otherwise a dead link
    // leaves the last good values on screen looking healthy.
    //
    // This tick must NOT rely on requestAnimationFrame. rAF stops firing
    // whenever the page is not being composited -- a backgrounded or minimised
    // tab, and also any offscreen/headless renderer -- which would freeze the
    // strip *and* the staleness watchdog with it, leaving a dead link showing
    // its last good values indefinitely: exactly the failure the watchdog
    // exists to catch. So if a frame has not landed in a while, render
    // synchronously instead of waiting for one that may never come.
    const FRAME_STALL_MS = 1000;

    setInterval(function heartbeat() {
        const now = performance.now();
        if (now - S.lastCommit > FRAME_STALL_MS) {
            if (S.rafHandle !== null) {
                cancelAnimationFrame(S.rafHandle);
                S.rafHandle = null;
            }
            S.dirty = true;
            commit();
            return;
        }
        schedule();
    }, 500);

    // ----------------------------------------------------------- strip render
    function render() {
        const now = performance.now();
        const r = S.readout;
        const sl = S.slow;
        const linkDead = !window.state || !window.state.connected;
        const readoutStale = linkDead || (now - S.lastReadout) > STALE_MS;
        const slowStale = linkDead || (now - S.lastSlow) > SLOW_STALE_MS;

        for (const id of ['battery', 'lidar', 'tilt', 'oak', 'imu', 'net']) {
            markStale(id, readoutStale);
        }
        markStale('cpu', slowStale);

        renderLink(linkDead, readoutStale);
        renderBattery(r);
        renderTilt(r);
        renderOak(r, sl);
        renderImu(r);
        renderLidar(r, sl);
        renderNet();
        renderSystem(sl);
        renderSweepPanel(r);
    }

    function renderLink(dead, stale) {
        if (dead) {
            tileState('link', 'bad', 'DOWN', 'no server');
            return;
        }
        if (stale) {
            tileState('link', 'warn', 'STALLED', 'no data');
            return;
        }
        const b = S.build;
        const sub = b ? (b.host || '?') + ' ' + (b.rev || '?') + (b.dirty ? '+' : '') : 'connected';
        tileState('link', 'ok', 'UP', sub);
    }

    function renderBattery(r) {
        if (!r || !r.power) return;
        const p = r.power;
        const pct = p.battery_pct;
        const level = pct < 15 ? 'bad' : pct < 30 ? 'warn' : 'ok';

        // "measured" false means current came from the motor-duty guess, not the
        // shunt. Presenting a guess in the same style as a measurement is how
        // the old GUI showed a fabricated amp figure as fact.
        const amps = p.measured ? fmt(p.current, 2, ' A') : '~' + fmt(p.current, 2, ' A');
        let sub = fmt(p.voltage, 2, ' V') + ' / ' + amps;
        if (p.minutes_left !== null && p.minutes_left !== undefined) {
            sub += '  ' + Math.round(p.minutes_left) + ' min';
        }
        tileState('battery', level, fmt(pct, 0, '%'), sub);
    }

    function renderTilt(r) {
        const t = r && r.tilt;
        if (!t) { tileState('tilt', 'idle', '--', 'no mount'); return; }

        let level = 'idle';
        let sub;
        if (t.age_s !== null && t.age_s > 1.0) {
            // Stale samples with a plausible-looking angle are the dangerous
            // case: the number is wrong but does not look wrong.
            level = 'bad';
            sub = 'sample ' + fmt(t.age_s, 1, ' s old');
        } else if (t.scan_gated) {
            level = 'warn';
            sub = 'scan gated';
        } else if (t.scanning) {
            level = 'ok';
            sub = t.moving ? 'sweeping' : 'station';
        } else {
            sub = Math.abs(t.deg) < 1.0 ? 'level' : 'off level';
        }
        tileState('tilt', level, fmt(t.deg, 1, ' deg'), sub);

        // Sweep-panel detail
        const mark = $('tilt-mark');
        if (mark) {
            const pctPos = Math.max(0, Math.min(100, (t.deg + 45) / 90 * 100));
            if (mark.__pos !== pctPos) {
                mark.__pos = pctPos;
                mark.style.left = pctPos + '%';
            }
            setClass(mark, 'gated', t.scan_gated);
        }
        setText($('tilt-deg'), fmt(t.deg, 2, ' deg'));
        setText($('tilt-counts'), t.counts === null ? '--' : String(t.counts));
        setText($('tilt-age'), t.age_s === null ? '--' : fmt(t.age_s, 2, ' s'));
        setText($('tilt-errors'), t.reads ?
            t.errors + ' / ' + t.reads + ' (' + (100 * t.errors / t.reads).toFixed(1) + '%)' : '--');
        setText($('tilt-gate'), t.scan_gated ? 'CLOSED - scans discarded' : 'open');

        const gateEl = $('tilt-gate');
        if (gateEl) {
            setClass(gateEl, 'warn', !!t.scan_gated);
            setClass(gateEl, 'ok', !t.scan_gated);
        }
    }

    function renderOak(r, sl) {
        const fps = sl ? sl.fps_oak_depth : null;
        const hasImu = !!(r && r.oak_imu);
        if (!fps && !hasImu) { tileState('oak', 'idle', '--', 'not detected'); return; }
        const level = fps > 10 ? 'ok' : fps > 0 ? 'warn' : 'bad';
        tileState('oak', level, fmt(fps, 1, ' fps'), 'depth stream');
    }

    function renderImu(r) {
        // Only the OAK's BMI270 reaches the browser today. The chassis IMU (the
        // ICM-42688P replacing the MPU9250) has no publisher in this payload
        // yet, so the tile says which sensor it is showing rather than
        // implying the robot has one IMU.
        const imu = r && r.oak_imu;
        if (!imu || !imu.gyro) { tileState('imu', 'idle', '--', 'no IMU'); return; }

        const gz = imu.gyro.z !== undefined ? imu.gyro.z : imu.gyro[2];
        if (typeof gz === 'number') {
            S.gyro.sum += gz;
            S.gyro.n += 1;
            S.gyro.last = gz;
        }
        // Mean drift in deg/min is the number that actually diagnoses this
        // robot: the MPU9250 sat at -11.7 deg/min and walked the costmap.
        const meanDegMin = S.gyro.n ? (S.gyro.sum / S.gyro.n) * (180 / Math.PI) * 60 : 0;
        const mag = Math.abs(meanDegMin);
        const level = mag > 5 ? 'bad' : mag > 1 ? 'warn' : 'ok';
        tileState('imu', level, fmt(meanDegMin, 2, ' d/m'), 'oak bias, n=' + S.gyro.n);
    }

    function renderLidar(r) {
        // Scan rate arrives over the Foxglove bridge, not this socket, so the
        // tile reports what this socket does know: whether the sweep is
        // currently gating scans away from the costmap and the CBF.
        const t = r && r.tilt;
        if (!t) { tileState('lidar', 'idle', '--', 'unknown'); return; }
        if (t.scan_gated) {
            tileState('lidar', 'warn', 'GATED', 'CBF set is frozen');
        } else {
            tileState('lidar', 'ok', 'OPEN', t.scanning ? 'sweep running' : 'gate open');
        }
    }

    function renderNet() {
        const now = performance.now();
        const dt = (now - S.rateWindowStart) / 1000;
        if (dt >= 1.0) {
            S.msgRate = S.msgCount / dt;
            S.byteRate = S.byteCount / dt;
            S.msgCount = 0;
            S.byteCount = 0;
            S.rateWindowStart = now;
        }
        const kbps = (S.byteRate * 8) / 1000;
        // The documented failure was a 4 Mbps stream collapsing a congested
        // link, so the thresholds sit either side of the 1.5 Mbps cap.
        const level = kbps > 2500 ? 'bad' : kbps > 1500 ? 'warn' : 'ok';
        tileState('net', level,
            kbps > 1000 ? (kbps / 1000).toFixed(2) + ' Mbps' : Math.round(kbps) + ' kbps',
            S.msgRate.toFixed(0) + ' msg/s');
    }

    function renderSystem(sl) {
        const sys = sl && sl.system;
        if (!sys || !sys.cpu_per_core) { tileState('cpu', 'idle', '--', 'no metrics'); return; }

        const cores = sys.cpu_per_core;
        const peak = cores.length ? Math.max.apply(null, cores) : 0;
        const level = sys.cpu_total > 85 ? 'bad' : sys.cpu_total > 60 ? 'warn' : 'ok';
        tileState('cpu', level, fmt(sys.cpu_total, 0, '%'),
            'peak core ' + fmt(peak, 0, '%') +
            (sys.temp_c !== null && sys.temp_c !== undefined ? '  ' + fmt(sys.temp_c, 0, ' C') : ''));

        // psutil scales cpu_percent so 100% == one core fully busy. On a 6-core
        // Orin the ceiling is 600%, so 126% is 1.3 cores' worth spread across
        // them -- NOT one core at 126%, which is not a thing. Spell out both
        // numbers, because the bare percentage is what makes this look pinned.
        const nCores = cores.length || 1;
        const coresUsed = sys.proc_cpu / 100.0;
        setText($('sys-proc'),
            fmt(sys.proc_cpu, 0, '%') + '  =  ' + coresUsed.toFixed(2) +
            ' of ' + nCores + ' cores' + '   ' + fmt(sys.proc_rss_mb, 0, ' MB'));

        // A process sitting just under 1.00 core is the GIL signature worth
        // flagging; above it, Python-level work is already overlapping with
        // C extensions that release the lock.
        const procEl = $('sys-proc');
        if (procEl) {
            const nearOneCore = coresUsed > 0.85 && coresUsed < 1.05;
            setClass(procEl, 'warn', nearOneCore);
            setClass(procEl, 'ok', coresUsed >= 1.05);
            procEl.title = nearOneCore
                ? 'Pinned near one core: the GIL serialises Python bytecode, so this is the ceiling for pure-Python work.'
                : 'Above one core, so work is running outside the GIL (cv2/numpy/torch/depthai release it) or in subprocesses.';
        }
        setText($('sys-mem'), fmt(sys.mem_pct, 1, '%'));
        setText($('sys-load'), fmt(sys.loadavg, 2, ''));
        setText($('sys-temp'), sys.temp_c === null || sys.temp_c === undefined
            ? '--' : fmt(sys.temp_c, 1, ' C'));

        renderCoreBars(cores);
    }

    function renderCoreBars(cores) {
        const host = $('sys-cores');
        if (!host) return;
        if (host.children.length !== cores.length) {
            host.innerHTML = '';
            cores.forEach((_, i) => {
                const row = document.createElement('div');
                row.className = 'corebar';
                row.innerHTML = '<span style="width:2.6rem">cpu' + i + '</span>' +
                    '<span class="corebar-track"><span class="corebar-fill"></span></span>' +
                    '<span class="corebar-pct" style="width:2.8rem;text-align:right"></span>';
                host.appendChild(row);
            });
        }
        cores.forEach((v, i) => {
            const row = host.children[i];
            if (!row) return;
            const fill = row.querySelector('.corebar-fill');
            const pct = row.querySelector('.corebar-pct');
            if (fill.__v !== v) {
                fill.__v = v;
                fill.style.width = Math.max(0, Math.min(100, v)) + '%';
                fill.classList.toggle('bad', v > 85);
                fill.classList.toggle('warn', v > 60 && v <= 85);
            }
            setText(pct, v.toFixed(0) + '%');
        });
    }

    function renderSweepPanel(r) {
        const note = $('sweep-state-note');
        if (!note) return;
        const t = r && r.tilt;
        const mode = S.sweep ? S.sweep.mode : null;
        if (!t) { setText(note, 'idle'); return; }
        setText(note, (t.scanning ? 'sweeping' : 'idle') + (mode ? ' / ' + mode : ''));
    }

    // ------------------------------------------------------------ message tap
    // Wrap main.js's handleMessage so both layers see every message and there
    // is still only one socket.
    function installTap() {
        const original = window.handleMessage;
        if (typeof original !== 'function') return false;

        window.handleMessage = function (data) {
            try {
                ingest(data);
            } catch (e) {
                console.error('[dashboard] ingest failed', e);
            }
            return original.apply(this, arguments);
        };
        return true;
    }

    function ingest(data) {
        S.msgCount += 1;
        const now = performance.now();
        switch (data && data.type) {
            case 'readout':
                S.readout = data;
                S.lastReadout = now;
                break;
            case 'readout_slow':
                S.slow = data;
                S.lastSlow = now;
                break;
            case 'sweep_config':
                S.sweep = data;
                break;
            case 'hello':
                S.build = data.build || null;
                if (S.build) {
                    setText($('sys-build'),
                        'build ' + (S.build.rev || '?') + (S.build.dirty ? ' (dirty)' : '') +
                        ' on ' + (S.build.host || '?'));
                }
                // A new server means a new integral; drop the gyro average so a
                // restart does not average across two sessions.
                S.gyro = { sum: 0, n: 0, last: null };
                break;
            default:
                break;
        }
        schedule();
    }

    // Byte accounting: count the binary camera frames too, since they are the
    // overwhelming majority of the traffic the link has to carry.
    function installByteCounter() {
        if (!window.state || !window.state.ws) return false;
        const ws = window.state.ws;
        if (ws.__counted) return true;
        ws.__counted = true;
        ws.addEventListener('message', (ev) => {
            S.byteCount += (ev.data && ev.data.byteLength) ? ev.data.byteLength
                : (typeof ev.data === 'string' ? ev.data.length : 0);
        });
        return true;
    }

    // ------------------------------------------------------------- reconnect
    // main.js drops the socket on close and never retries, so a server restart
    // silently bricks an open tab -- it looks exactly like broken hardware.
    const RC = { timer: null, delay: 1000, want: false };

    function banner(show, text) {
        const el = $('reconnect-banner');
        if (!el) return;
        el.classList.toggle('visible', show);
        if (show) setText(el, text);
    }

    function scheduleReconnect() {
        if (!RC.want || RC.timer) return;
        banner(true, 'Connection lost. Reconnecting in ' + Math.round(RC.delay / 1000) + 's...');
        RC.timer = setTimeout(() => {
            RC.timer = null;
            if (!RC.want) return;
            banner(true, 'Reconnecting...');
            try {
                window.connect();
            } catch (e) {
                console.warn('[dashboard] reconnect attempt failed', e);
            }
            RC.delay = Math.min(RC.delay * 2, 15000);
            watchSocket();
        }, RC.delay);
    }

    // Re-arm the close/open hooks on whatever socket currently exists.
    function watchSocket() {
        const ws = window.state && window.state.ws;
        if (!ws || ws.__watched) return;
        ws.__watched = true;
        installByteCounter();

        ws.addEventListener('open', () => {
            RC.delay = 1000;
            RC.want = true;
            banner(false);
            schedule();
        });
        const onDown = () => { schedule(); scheduleReconnect(); };
        ws.addEventListener('close', onDown);
        ws.addEventListener('error', onDown);
    }

    // main.js reassigns state.ws on every connect(), so poll for a new socket
    // object rather than trying to hook a single instance for all time.
    setInterval(watchSocket, 500);

    // An explicit Disconnect click is a decision, not a fault: stop retrying.
    function wireDisconnect() {
        const btn = $('disconnect-btn');
        if (btn && !btn.__wired) {
            btn.__wired = true;
            btn.addEventListener('click', () => {
                RC.want = false;
                if (RC.timer) { clearTimeout(RC.timer); RC.timer = null; }
                banner(false);
            });
        }
        const cbtn = $('connect-btn');
        if (cbtn && !cbtn.__wired) {
            cbtn.__wired = true;
            cbtn.addEventListener('click', () => { RC.want = true; RC.delay = 1000; });
        }
    }

    // ------------------------------------------------------------------ tabs
    function wireTabs() {
        const bar = $('tabbar');
        if (!bar) return;
        bar.addEventListener('click', (ev) => {
            const btn = ev.target.closest('.tab');
            if (!btn) return;
            const name = btn.dataset.tab;
            bar.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t === btn));
            document.querySelectorAll('.tab-panel').forEach(p => {
                p.classList.toggle('active', p.dataset.panel === name);
            });
            // Canvas-backed panels (lidar, 3D, nav map) size themselves on
            // layout; a panel that was display:none measured zero, so give
            // them a resize once they are actually visible.
            window.dispatchEvent(new Event('resize'));
            try { localStorage.setItem('x3_tab', name); } catch (e) { /* private mode */ }
        });

        let saved = null;
        try { saved = localStorage.getItem('x3_tab'); } catch (e) { /* ignore */ }
        if (saved) {
            const btn = bar.querySelector('.tab[data-tab="' + saved + '"]');
            if (btn) btn.click();
        }
    }

    // ------------------------------------------------------------------ init
    function init() {
        wireTabs();
        wireDisconnect();
        watchSocket();
        if (!installTap()) {
            console.warn('[dashboard] handleMessage not exported by main.js; ' +
                'strip will not receive telemetry');
        }
        schedule();
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
