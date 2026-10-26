/* ==========================================================================
   Log console: a terminal-style drawer under the tab area.

   Shows the server's log records and the output of the test scripts the GUI
   launches (point-to-point, reactive / predictive A/B). The server only sends
   lines while the drawer is open (log_subscribe / log_unsubscribe), so a
   closed console costs no bandwidth.

   Like dashboard.js it taps main.js's handleMessage instead of opening a
   second socket.
   ========================================================================== */
(function () {
    'use strict';

    const MAX_KEEP = 3000;        // entries kept client-side for re-filtering
    const OPEN_KEY = 'x3_log_open';
    const HEIGHT_KEY = 'x3_log_height';
    const LEVEL_RANK = { DEBUG: 0, INFO: 1, WARNING: 2, ERROR: 3, CRITICAL: 4 };

    const $ = (id) => document.getElementById(id);
    const L = {
        entries: [],       // {seq, t, src, lvl, msg} or {sep: text}
        lastSeq: 0,
        open: false,
        follow: true,      // auto-scroll; turned off when the user scrolls up
        unseenErrors: 0,
        pending: [],       // entries waiting for the next batched render
        timer: null,       // batches DOM writes; a timer, not rAF, so hidden tabs keep up
        sources: new Set(),
    };

    function send(msg) {
        if (typeof window.sendMessage === 'function') window.sendMessage(msg);
    }

    function fmtTime(t) {
        const d = new Date(t * 1000);
        const p = (n, w) => String(n).padStart(w || 2, '0');
        return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}.${p(d.getMilliseconds(), 3)}`;
    }

    // ------------------------------------------------------------- filtering
    function passes(e) {
        if (e.sep) return true;
        const src = $('log-src-filter').value;
        if (src !== 'all' && !(src === 'tests' ? e.src !== 'server' : e.src === src)) return false;
        const min = LEVEL_RANK[$('log-level-filter').value] || 0;
        if ((LEVEL_RANK[e.lvl] ?? 1) < min) return false;
        const q = $('log-search').value.trim().toLowerCase();
        if (q && !(e.msg.toLowerCase().includes(q) || e.src.toLowerCase().includes(q))) return false;
        return true;
    }

    function lineEl(e) {
        const div = document.createElement('div');
        if (e.sep) {
            div.className = 'log-line log-sep';
            div.textContent = e.sep;
            return div;
        }
        div.className = 'log-line lvl-' + (e.lvl || 'INFO').toLowerCase();
        const ts = document.createElement('span');
        ts.className = 'log-ts';
        ts.textContent = fmtTime(e.t);
        const src = document.createElement('span');
        src.className = 'log-src' + (e.src === 'server' ? '' : ' log-src-test');
        src.textContent = e.src;
        const msg = document.createElement('span');
        msg.className = 'log-msg';
        msg.textContent = e.msg;
        div.append(ts, src, msg);
        return div;
    }

    function rerender() {
        const body = $('log-console-body');
        body.replaceChildren(...L.entries.filter(passes).map(lineEl));
        if (L.follow) body.scrollTop = body.scrollHeight;
    }

    function flush() {
        L.timer = null;
        const body = $('log-console-body');
        if (!L.open) { L.pending = []; return; }
        const frag = document.createDocumentFragment();
        for (const e of L.pending) if (passes(e)) frag.appendChild(lineEl(e));
        L.pending = [];
        body.appendChild(frag);
        while (body.childElementCount > MAX_KEEP) body.firstElementChild.remove();
        if (L.follow) body.scrollTop = body.scrollHeight;
    }

    function push(e) {
        L.entries.push(e);
        if (L.entries.length > MAX_KEEP) L.entries.splice(0, L.entries.length - MAX_KEEP);
        L.pending.push(e);
        if (!e.sep && !L.sources.has(e.src)) addSource(e.src);
        if (!L.timer) L.timer = setTimeout(flush, 50);
    }

    function addSource(src) {
        L.sources.add(src);
        const sel = $('log-src-filter');
        if ([...sel.options].some(o => o.value === src)) return;
        const opt = document.createElement('option');
        opt.value = src;
        opt.textContent = src;
        sel.appendChild(opt);
    }

    // -------------------------------------------------------------- messages
    function ingest(data) {
        if (data.type === 'hello') {
            if (L.open) send({ type: 'log_subscribe' });
            return;
        }
        if (data.type !== 'log_lines') return;
        const lines = data.lines || [];
        if (data.backlog && lines.length && lines[lines.length - 1].seq < L.lastSeq) {
            // Sequence went backwards: a new server process, not a reconnect.
            push({ sep: '--- server restarted ---' });
            L.lastSeq = 0;
        }
        if (data.dropped) push({ sep: `--- ${data.dropped} lines skipped ---` });
        for (const e of lines) {
            if (e.seq <= L.lastSeq) continue;     // already shown before a reconnect
            L.lastSeq = e.seq;
            push(e);
            if (!data.backlog && (LEVEL_RANK[e.lvl] ?? 1) >= LEVEL_RANK.ERROR && !L.open) {
                L.unseenErrors += 1;
            }
        }
        updateBadge();
    }

    function updateBadge() {
        const b = $('log-error-badge');
        b.textContent = L.unseenErrors > 99 ? '99+' : String(L.unseenErrors);
        b.hidden = L.unseenErrors === 0;
    }

    // --------------------------------------------------------------- drawer
    function setOpen(open) {
        L.open = open;
        const panel = $('log-console');
        panel.classList.toggle('open', open);
        $('log-toggle-btn').textContent = open ? '▾ Hide' : '▸ Show';
        try { localStorage.setItem(OPEN_KEY, open ? '1' : '0'); } catch (e) { /* ignore */ }
        if (open) {
            L.unseenErrors = 0;
            updateBadge();
            send({ type: 'log_subscribe' });
            rerender();
        } else {
            send({ type: 'log_unsubscribe' });
        }
    }

    function wireResize() {
        const panel = $('log-console');
        const handle = $('log-resize-handle');
        let startY = 0, startH = 0;
        const move = (ev) => {
            const h = Math.max(120, Math.min(window.innerHeight * 0.8, startH + (startY - ev.clientY)));
            panel.style.setProperty('--log-h', h + 'px');
        };
        const up = () => {
            window.removeEventListener('pointermove', move);
            window.removeEventListener('pointerup', up);
            try { localStorage.setItem(HEIGHT_KEY, panel.style.getPropertyValue('--log-h')); } catch (e) { /* ignore */ }
        };
        handle.addEventListener('pointerdown', (ev) => {
            if (!L.open) return;
            startY = ev.clientY;
            startH = $('log-console-body').getBoundingClientRect().height;
            window.addEventListener('pointermove', move);
            window.addEventListener('pointerup', up);
            ev.preventDefault();
        });
        try {
            const h = localStorage.getItem(HEIGHT_KEY);
            if (h) panel.style.setProperty('--log-h', h);
        } catch (e) { /* ignore */ }
    }

    function wire() {
        $('log-toggle-btn').addEventListener('click', () => setOpen(!L.open));
        $('log-console-title').addEventListener('click', () => setOpen(!L.open));
        for (const id of ['log-src-filter', 'log-level-filter']) {
            $(id).addEventListener('change', rerender);
        }
        $('log-search').addEventListener('input', rerender);

        const body = $('log-console-body');
        body.addEventListener('scroll', () => {
            const atBottom = body.scrollHeight - body.scrollTop - body.clientHeight < 24;
            L.follow = atBottom;
            $('log-follow-btn').hidden = atBottom;
        });
        $('log-follow-btn').addEventListener('click', () => {
            L.follow = true;
            body.scrollTop = body.scrollHeight;
            $('log-follow-btn').hidden = true;
        });
        $('log-clear-btn').addEventListener('click', () => {
            L.entries = [];
            body.replaceChildren();
        });
        $('log-copy-btn').addEventListener('click', () => {
            const text = L.entries.filter(passes).map(e => e.sep ? e.sep
                : `${fmtTime(e.t)} [${e.src}] ${e.lvl} ${e.msg}`).join('\n');
            const btn = $('log-copy-btn');
            const done = (ok) => {
                btn.textContent = ok ? 'Copied' : 'Copy failed';
                setTimeout(() => { btn.textContent = 'Copy'; }, 1200);
            };
            // The GUI is served over plain http from the robot, where the
            // async clipboard API is unavailable; fall back to execCommand.
            if (navigator.clipboard && window.isSecureContext) {
                navigator.clipboard.writeText(text).then(() => done(true), () => done(false));
            } else {
                const ta = document.createElement('textarea');
                ta.value = text;
                document.body.appendChild(ta);
                ta.select();
                let ok = false;
                try { ok = document.execCommand('copy'); } catch (e) { /* ignore */ }
                ta.remove();
                done(ok);
            }
        });
        wireResize();
    }

    function installTap() {
        const original = window.handleMessage;
        if (typeof original !== 'function') return false;
        window.handleMessage = function (data) {
            try {
                if (data && (data.type === 'log_lines' || data.type === 'hello')) ingest(data);
            } catch (e) {
                console.error('[log-console] ingest failed', e);
            }
            return original.apply(this, arguments);
        };
        return true;
    }

    function init() {
        if (!$('log-console')) return;
        wire();
        if (!installTap()) {
            console.warn('[log-console] handleMessage not exported by main.js');
            return;
        }
        let open = false;
        try { open = localStorage.getItem(OPEN_KEY) === '1'; } catch (e) { /* ignore */ }
        // If the socket is already up, subscribe now; otherwise `hello` will.
        setOpen(open);
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
