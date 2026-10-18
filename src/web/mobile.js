/* Touch UI for phone and iPad: stick overlay, STOP, speed slider, status
 * pills, and the phone's trimmed layout.  Relies on globals from main.js
 * (state, sendMessage, motorToggle).  Desktop is unaffected: nothing here runs
 * unless the device has a coarse pointer or the URL has ?mobile=1.
 *
 * Drive mapping copies the Bluetooth gamepad path in main.js so a touch stick
 * and a controller feel the same:
 *   right stick  forward/back (vx) + strafe (vy)
 *   left stick   rotate (omega)
 *   speed slider stands in for R2: MOVE 0.20 -> 0.45, ROT 0.5 -> 1.0
 */
(function () {
    'use strict';

    const params = new URLSearchParams(location.search);
    const forced = params.get('mobile');
    const coarse = window.matchMedia('(pointer: coarse)').matches;
    if (!coarse && !forced) return;

    const shortSide = Math.min(screen.width, screen.height);
    const isPhone = forced === 'phone' || (forced !== 'tablet' && shortSide < 600);
    document.body.classList.add('touch-ui');
    if (isPhone) document.body.classList.add('phone');

    // Same constants and curve as the gamepad branch in main.js.
    const DEADZONE = 0.1;
    const BASE_MOVE = 0.20, FAST_MOVE = 0.45;
    const BASE_ROT = 0.5, FAST_ROT = 1.0;
    const SEND_MS = 50;           // 20 Hz, the gamepad throttle
    const expo = v => Math.sign(v) * Math.pow(Math.abs(v), 1.5);

    // ---- overlay DOM ------------------------------------------------------
    const root = document.createElement('div');
    root.className = 'm-overlay';
    root.innerHTML = `
        <div class="m-top">
            <span class="m-pill" id="m-link">LINK --</span>
            <span class="m-pill" id="m-batt">BATT --</span>
            <span class="m-pill" id="m-nav">NAV --</span>
            <span class="m-pill" id="m-cbf" hidden></span>
        </div>
        <div class="m-stick left" id="m-stick-rot"><span class="m-label">ROTATE</span><div class="m-knob"></div></div>
        <div class="m-stick right" id="m-stick-move"><span class="m-label">MOVE</span><div class="m-knob"></div></div>
        <div class="m-bottom">
            <button class="m-stop" id="m-stop">STOP</button>
            <div class="m-row">
                <button class="m-chip" id="m-motors">MOVEMENT</button>
                <label class="m-speed">SPEED <input type="range" id="m-speed" min="0" max="100" value="0"></label>
                <button class="m-chip" id="m-sticks">STICKS: AUTO</button>
            </div>
        </div>`;
    document.body.appendChild(root);
    const $ = id => root.querySelector('#' + id);

    // iPad: warnings sit above STOP, within thumb reach.
    if (!isPhone) $('m-stop').parentElement.insertBefore(root.querySelector('.m-top'), $('m-stop'));

    // ---- sticks -----------------------------------------------------------
    function makeStick(el) {
        const knob = el.querySelector('.m-knob');
        const s = { x: 0, y: 0, id: null };   // x right+, y down+, like gamepad axes
        function set(ev) {
            const r = el.getBoundingClientRect();
            const R = r.width / 2;
            let dx = (ev.clientX - (r.left + R)) / R;
            let dy = (ev.clientY - (r.top + R)) / R;
            const m = Math.hypot(dx, dy);
            if (m > 1) { dx /= m; dy /= m; }
            s.x = dx; s.y = dy;
            knob.style.transform = `translate(${dx * R * 0.6}px, ${dy * R * 0.6}px)`;
        }
        function release() {
            s.x = 0; s.y = 0; s.id = null;
            knob.style.transform = '';
        }
        el.addEventListener('pointerdown', ev => {
            if (s.id !== null || el.classList.contains('disabled')) return;
            s.id = ev.pointerId;
            el.setPointerCapture(ev.pointerId);
            set(ev);
            ev.preventDefault();
        });
        el.addEventListener('pointermove', ev => { if (ev.pointerId === s.id) set(ev); });
        ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(t =>
            el.addEventListener(t, ev => { if (ev.pointerId === s.id) release(); }));
        el.addEventListener('contextmenu', e => e.preventDefault());
        s.release = release;
        return s;
    }
    const rotStick = makeStick($('m-stick-rot'));
    const moveStick = makeStick($('m-stick-move'));
    const stickEls = [$('m-stick-rot'), $('m-stick-move')];

    // ---- drive loop -------------------------------------------------------
    let stopFrames = 0;
    function releaseAll() { rotStick.release(); moveStick.release(); }

    function canDrive() {
        return state.connected && state.motorsEnabled && !state.isAutoDriving;
    }

    function tick() {
        const usable = canDrive();
        stickEls.forEach(e => e.classList.toggle('disabled', !usable));
        if (!usable) { releaseAll(); }

        const t = ($('m-speed').value || 0) / 100;
        const MOVE = BASE_MOVE + t * (FAST_MOVE - BASE_MOVE);
        const ROT = BASE_ROT + t * (FAST_ROT - BASE_ROT);
        const dz = v => (Math.abs(v) < DEADZONE ? 0 : v);
        const rx = dz(moveStick.x), ry = dz(moveStick.y), lx = dz(rotStick.x);

        const vx = Math.round(-expo(ry) * MOVE * 100) / 100;
        const vy = Math.round(-expo(rx) * MOVE * 100) / 100;
        const om = Math.round(-expo(lx) * ROT * 100) / 100;

        const active = usable && (vx !== 0 || vy !== 0 || om !== 0);
        let send = false;
        if (active) { stopFrames = 3; send = true; }
        else if (stopFrames > 0 && state.connected) { stopFrames--; send = true; }
        if (send) {
            sendMessage({ type: 'set_move', vx: vx, vy: vy, omega: om });
            state.lastVx = vx; state.lastVy = vy; state.lastOmega = om;
        }
    }
    setInterval(tick, SEND_MS);

    // Fail safe: any way the page can lose the finger sends zero immediately.
    function haltNow() {
        releaseAll();
        if (state.connected) {
            for (let i = 0; i < 3; i++) sendMessage({ type: 'set_move', vx: 0, vy: 0, omega: 0 });
        }
        stopFrames = 0;
    }
    document.addEventListener('visibilitychange', () => { if (document.hidden) haltNow(); });
    window.addEventListener('pagehide', haltNow);
    window.addEventListener('blur', haltNow);

    // ---- buttons ----------------------------------------------------------
    $('m-stop').addEventListener('click', () => {
        haltNow();
        const b = document.getElementById('stop-btn');
        if (b) b.click();     // reuse the existing STOP handler
    });
    $('m-motors').addEventListener('click', () => { haltNow(); motorToggle(); });

    // STICKS: AUTO hides them while a real gamepad is connected, so two
    // sources never send set_move at once.
    const MODES = ['AUTO', 'ON', 'OFF'];
    let stickMode = 'AUTO';
    $('m-sticks').addEventListener('click', () => {
        stickMode = MODES[(MODES.indexOf(stickMode) + 1) % MODES.length];
        $('m-sticks').textContent = 'STICKS: ' + stickMode;
        haltNow();
        refresh();
    });
    const padConnected = () =>
        Array.from(navigator.getGamepads ? navigator.getGamepads() : []).some(Boolean);

    // ---- status pills / visibility ---------------------------------------
    function pill(el, text, cls) {
        el.textContent = text;
        el.className = 'm-pill ' + (cls || '');
        el.hidden = !text;
    }
    function refresh() {
        const showSticks = stickMode === 'ON' || (stickMode === 'AUTO' && !padConnected());
        stickEls.forEach(e => { e.style.display = showSticks ? '' : 'none'; });
        if (!showSticks) haltNow();

        const m = $('m-motors');
        m.textContent = state.motorsEnabled ? 'MOVEMENT ON' : 'MOVEMENT OFF';
        m.className = 'm-chip ' + (state.motorsEnabled ? 'on' : 'off');

        pill($('m-link'), state.connected ? 'LINK OK' : 'NO LINK', state.connected ? 'ok' : 'bad');

        const bv = document.getElementById('tile-battery-value');
        pill($('m-batt'), 'BATT ' + (bv ? bv.textContent : '--'), '');

        const nb = document.getElementById('nav-status-badge');
        const navText = nb ? nb.textContent.trim() : '';
        pill($('m-nav'), 'NAV ' + (navText || '--'), /NAVIGATING|ACTIVE/i.test(navText) ? 'ok' : '');

        const cbf = state.latestData && state.latestData.readout && state.latestData.readout.cbf;
        const c = $('m-cbf');
        if (cbf && cbf.blind) pill(c, 'NO LIDAR - UNPROTECTED', 'bad');
        else if (cbf && cbf.hold) pill(c, 'HOLD - 3D SWEEP', 'warn');
        else if (cbf && cbf.limited) pill(c, 'OBSTACLE - SLOWING', 'warn');
        else pill(c, '', '');
    }
    setInterval(refresh, 250);
    window.addEventListener('gamepadconnected', refresh);
    window.addEventListener('gamepaddisconnected', refresh);

    // ---- phone layout trimming (things CSS can't address by id alone) -----
    if (isPhone) {
        const tabbar = document.getElementById('tabbar');
        if (tabbar) {
            tabbar.querySelectorAll('.tab').forEach(t => {
                if (t.dataset.tab === 'nav') t.textContent = 'Map';
                else if (t.dataset.tab !== 'drive') t.classList.add('m-extra');
            });
            const more = document.createElement('button');
            more.className = 'm-more-btn';
            more.textContent = 'More';
            more.addEventListener('click', () => {
                const open = !tabbar.classList.contains('m-more-open');
                tabbar.classList.toggle('m-more-open', open);
                document.body.classList.toggle('m-more-open', open);
                more.textContent = open ? 'Less' : 'More';
                window.dispatchEvent(new Event('resize'));
            });
            tabbar.appendChild(more);
        }

        // Map tab: keep only SLAM start/stop, map name + save, and the status line.
        const tools = document.querySelector('#nav-card .navigation-tools');
        const slamBtn = document.getElementById('start-slam-btn');
        const nameInput = document.getElementById('map-name-input');
        const status = document.getElementById('slam-status-text');
        if (tools && slamBtn && nameInput) {
            const slamRow = slamBtn.parentElement;
            const keep = new Set([
                slamRow, slamRow.previousElementSibling,
                nameInput.closest('.nav-map-row'), status,
            ]);
            Array.from(tools.children).forEach(k => { if (!keep.has(k)) k.classList.add('m-hide'); });
            ['start-slam-3d-btn', 'stop-slam-3d-btn', 'slam-scan-3d-btn'].forEach(id => {
                const e = document.getElementById(id);
                if (e) e.classList.add('m-hide');
            });
        }
    }

    // Open on the Drive tab every time: a remembered Sweep/Pedestrian tab has
    // no phone layout and would hide the sticks' context.
    if (isPhone) {
        const drive = document.querySelector('#tabbar .tab[data-tab="drive"]');
        if (drive) drive.click();
    }
})();
