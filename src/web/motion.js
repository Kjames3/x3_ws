// Motion layer for the GUI (styles in motion.css). Purely additive: it only
// watches the DOM other scripts already drive, so no existing handler changes.
//
// Doherty threshold: a system that answers within ~400 ms keeps the user in
// flow. Press feedback here starts on pointerdown, and every transition below
// finishes inside 400 ms, so the GUI itself never becomes the slow part.
(function () {
    'use strict';
    const reduce = window.matchMedia('(prefers-reduced-motion: reduce)');
    const PRESS_SEL = '.btn, .btn-nav, .btn-nav-sm, .tab, .nav-mode-btn, .toggle-btn, ' +
        '.nav-view-controls button, .shutdown-dialog-actions button, #shutdown-btn, button[data-tab]';

    function restart(el, cls) {
        el.classList.remove(cls);
        void el.offsetWidth;            // restart the animation if it is mid-run
        el.classList.add(cls);
    }
    function once(el, cls) {
        el.addEventListener('animationend', function done(ev) {
            if (ev.target !== el) return;
            el.classList.remove(cls);
            el.removeEventListener('animationend', done);
        });
    }

    // 1. Pulse to confirm --------------------------------------------------
    let lastPressed = null;
    document.addEventListener('pointerdown', ev => {
        const b = ev.target.closest(PRESS_SEL);
        if (!b) return;
        lastPressed = b;
        if (b.disabled || reduce.matches) return;
        restart(b, 'm-pulse'); once(b, 'm-pulse');
    }, true);
    document.addEventListener('click', ev => {       // keyboard activation
        if (ev.detail !== 0) return;
        const b = ev.target.closest(PRESS_SEL);
        if (!b || reduce.matches) return;
        lastPressed = b;
        restart(b, 'm-pulse'); once(b, 'm-pulse');
    }, true);

    // 2. One sliding indicator per tab group --------------------------------
    function inkGroup(bar, itemSel) {
        if (!bar) return;
        bar.classList.add('m-has-ink');
        const ink = document.createElement('span');
        ink.className = 'm-ink';
        ink.setAttribute('aria-hidden', 'true');
        bar.appendChild(ink);
        let first = true;
        function place() {
            const a = bar.querySelector(itemSel + '.active');
            if (!a || !a.offsetWidth) { ink.style.opacity = '0'; return; }
            if (first) ink.style.transition = 'none';
            ink.style.opacity = '1';
            ink.style.width = a.offsetWidth + 'px';
            ink.style.translate = a.offsetLeft + 'px 0';
            if (first) { void ink.offsetWidth; ink.style.transition = ''; first = false; }
        }
        new MutationObserver(place).observe(bar, { subtree: true, attributes: true, attributeFilter: ['class'] });
        window.addEventListener('resize', place);
        new ResizeObserver(place).observe(bar);
        place();
    }

    // 2b. Dialogs grow out of the control that opened them ------------------
    function wireDialogs() {
        document.querySelectorAll('dialog').forEach(dlg => {
            new MutationObserver(() => {
                if (!dlg.open || reduce.matches) return;
                const src = lastPressed && lastPressed.getBoundingClientRect();
                const r = dlg.getBoundingClientRect();
                if (src && src.width) {
                    dlg.style.setProperty('--m-from-x', (src.left + src.width / 2 - (r.left + r.width / 2)) + 'px');
                    dlg.style.setProperty('--m-from-y', (src.top + src.height / 2 - (r.top + r.height / 2)) + 'px');
                }
                restart(dlg, 'm-grow'); once(dlg, 'm-grow');
            }).observe(dlg, { attributes: true, attributeFilter: ['open'] });
        });
    }

    // 3. Blur-crossfade labels that change -----------------------------------
    // Buttons and status badges only: stat readouts update many times a
    // second and would just shimmer.
    const SWAP_SEL = 'button, .nav-badge, #nav-loc-badge';
    const lastText = new WeakMap();
    const lastSwap = new WeakMap();
    function wireSwap() {
        document.querySelectorAll(SWAP_SEL).forEach(el => lastText.set(el, el.textContent.trim()));
        new MutationObserver(muts => {
            if (reduce.matches) return;
            const seen = new Set();
            for (const m of muts) {
                const node = m.target.nodeType === 1 ? m.target : m.target.parentElement;
                const el = node && node.closest(SWAP_SEL);
                if (!el || seen.has(el)) continue;
                seen.add(el);
                const t = el.textContent.trim();
                if (t === lastText.get(el)) continue;
                const had = lastText.has(el);
                lastText.set(el, t);
                const now = performance.now();
                if (!had || now - (lastSwap.get(el) || 0) < 600) continue;   // skip first sight + chatter
                lastSwap.set(el, now);
                restart(el, 'm-swap'); once(el, 'm-swap');
            }
        }).observe(document.body, { subtree: true, childList: true, characterData: true });
    }

    // 4. Rubber band past the scroll edge -----------------------------------
    const MAX = 56;
    const damp = d => Math.sign(d) * MAX * (1 - Math.exp(-Math.abs(d) / 160));
    function rubberBand(el) {
        if (!el || el.dataset.mRb) return;
        el.dataset.mRb = '1';
        let acc = 0, idle = 0, touchY = null;
        const atTop = () => el.scrollTop <= 0;
        const atBottom = () => el.scrollTop + el.clientHeight >= el.scrollHeight - 1;
        function set(px) {
            el.classList.remove('m-rb-release');
            el.classList.add('m-rb-on');
            el.style.setProperty('--m-rb', px.toFixed(1) + 'px');
        }
        function release() {
            acc = 0;
            el.classList.add('m-rb-release');
            el.style.setProperty('--m-rb', '0px');
            setTimeout(() => {
                if (acc === 0) el.classList.remove('m-rb-on', 'm-rb-release');
            }, 400);
        }
        const skip = ev => reduce.matches || ev.defaultPrevented || ev._mRb ||
            (ev.target.closest && ev.target.closest('canvas, input, textarea, select, video'));
        el.addEventListener('wheel', ev => {
            if (skip(ev) || ev.ctrlKey) return;
            const up = ev.deltaY < 0;
            if (!((up && atTop()) || (!up && atBottom()))) { if (acc) release(); return; }
            ev._mRb = true;                   // the innermost scroller wins
            acc += -ev.deltaY * 0.6;
            set(damp(acc));
            clearTimeout(idle);
            idle = setTimeout(release, 120);
        }, { passive: true });
        el.addEventListener('touchstart', ev => {
            touchY = skip(ev) ? null : ev.touches[0].clientY;
        }, { passive: true });
        el.addEventListener('touchmove', ev => {
            if (touchY === null || ev._mRb) return;
            const dy = ev.touches[0].clientY - touchY;
            if ((dy > 0 && atTop()) || (dy < 0 && atBottom())) {
                ev._mRb = true; acc = dy; set(damp(dy));
            } else {
                touchY = ev.touches[0].clientY;
                if (acc) release();
            }
        }, { passive: true });
        const end = () => { touchY = null; if (acc) release(); };
        el.addEventListener('touchend', end, { passive: true });
        el.addEventListener('touchcancel', end, { passive: true });
    }

    // 5 + 6. Masked, staged reveal of a newly shown tab panel --------------
    function wirePanels() {
        document.querySelectorAll('.tab-panel').forEach(p => {
            let wasActive = p.classList.contains('active');
            new MutationObserver(() => {
                const on = p.classList.contains('active');
                if (on && !wasActive && !reduce.matches) {
                    [...p.children].forEach((c, i) => c.style.setProperty('--m-i', Math.min(i, 6)));
                    p.classList.remove('m-enter'); void p.offsetWidth; p.classList.add('m-enter');
                    // Drop the class once done: the clip-path end state would
                    // otherwise clip popovers and focus rings.
                    clearTimeout(p._mT);
                    p._mT = setTimeout(() => p.classList.remove('m-enter'), 260 + 6 * 35 + 40);
                }
                wasActive = on;
            }).observe(p, { attributes: true, attributeFilter: ['class'] });
        });
    }

    function init() {
        inkGroup(document.getElementById('tabbar'), '.tab');
        document.querySelectorAll('.nav-mode-toggle').forEach(g => inkGroup(g, '.nav-mode-btn'));
        wireDialogs();
        wireSwap();
        wirePanels();
        rubberBand(document.querySelector('.tab-area'));
        document.querySelectorAll('.detection-list, .log-console, [data-rubber-band]').forEach(rubberBand);
    }
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
})();
