# Furniture look V2 (Reality view), 2026-11-27 overnight

Branch `overnight/furniture-look-2026-11-27`. This is display only and feeds nothing to control.
The new look sits behind a flag and the default is unchanged. With the flag off, the frames are
pixel-identical to HEAD `4857e5e` (diffed against an archive of HEAD's `src/web`,
overview and lineup views; the stats line is excluded).

## Turning it on

`?furnitureLookV2=1` in the GUI URL. The value is remembered in `localStorage['furnitureLookV2']`,
and `?furnitureLookV2=0` turns it off again. It is read once, when `RealityMode.create` runs, so a
page reload applies a change. The pattern copies `navKeepYaw` in `main.js`.

## What changed

- **New `src/web/reality-furniture.js`** (`window.RealityFurniture`; node can `require` it).
  - `layout(kind, box, state, legacyParts)` is pure. It returns parts in **metres** in the local
    frame. The legacy code stretched a unit box by L x H x D, so legs, tops and cushions changed
    thickness with box size. In V2 a 2.2 m table still has a 3 cm top and 4 cm legs.
  - **Rectangular table:** thin top (2.4 to 3.6 cm) with a small overhang and four square legs.
    Dining and desk heights get a 7 cm apron. Low or coffee tables (`low` set, or H < 0.55) get
    chunkier legs and a lower shelf.
  - **Round table:** disc top with diameter = mean(L, D), a collar, a column, and a foot plate
    with a hub. reality-structure only marks a top round when D > 0.85 L, so the disc overhangs
    D by at most (L-D)/4.
  - **Chair:** seat slab whose top sits at the measured `seat`, using the legacy rule
    (seat/H clamped to [0.30, 0.72], default 0.46 H). Four legs, back posts continuing the rear
    legs up to H, and a backrest panel. Arms are added when L >= 0.61 m and dropped when
    L < 0.56 m. That is a guess, because no box field says "arms".
  - **Couch:** four 5 cm feet (<= 7 cm tall), a plinth between two armrests (arm top at
    seat + 20 cm), and a back frame. Seat cushions are split along the length: 1 below an inner
    width of 1.0 m, 2 below 1.65 m, 3 above, with 6 cm hysteresis so a drifting L does not
    flicker. Each seat cushion has a matching back cushion leaning 0.12 rad. Without a `seat`
    field the seat is clamp(0.52 H, 0.36, 0.48).
  - **Other kinds** (tv, bed, plant, cabinet, box) reuse the legacy `PARTS` table, scaled to
    metres, so they also get the new shading.
  - **Shading:** one `ShaderMaterial` source with fog support. Light grey (#dbe0e6) and
    untextured, with a fixed key light plus hemisphere and fill, and a height-based AO term
    (darker toward the floor). A fresnel rim and a **metric edge highlight** stand in for a
    bevel: the shader gets each part's size from `modelMatrix` and draws a ~6 mm bright line
    plus a 3 cm glow along every box edge or cylinder rim. Bevels therefore need no extra
    geometry and do not distort under scale.
  - **Contact shadow:** one plane per object with a rounded-rect or disc SDF, giving a dark core
    under the footprint plus a faint light halo just outside it. A dark blob alone was invisible
    on the #111820 floor; the halo is what makes the footprint read. It replaces the cyan
    footprint plate in V2. It sits at local y = 0.002 - bottom with `depthWrite: false`; the
    other floor layers sit at 0.004 to 0.016.
  - **Appear and disappear:** 650 ms fade-in with a 4 cm rise, and a 400 ms fade-out on removal
    or kind change. The fade runs from `RealityMode.update()` every frame. Turning the structure
    toggle or the Reality view off removes objects instantly. While translucent, an object reads
    as a ghost that materialises (inner parts show through), then goes solid: `transparent` is
    switched off at full opacity, so it sorts as opaque.
  - **Cost:** three unit geometries (box, 24-segment cylinder, 12-segment post) are shared by
    every part. Each object has three material clones, which share the compiled programs and
    exist so each object can carry its own opacity. Refitting only moves and scales the meshes it
    already has; nothing is rebuilt as the smoothed box drifts. Spare meshes are hidden.
    Material clones are disposed on removal, and the existing splat geometry is disposed through
    `ob.onDispose`.
- **`src/web/reality-mode.js`** (small hooks only, in the furniture section):
  - `look = RealityFurniture.createRenderer(THREE)` when the flag is on.
  - `removeObject(ob, instant)` hands V2 objects to `look.retire`.
  - In `updateStructure`, V2 keys objects on `o.kind`, not `table_round`, so a flickering
    `round` re-lays the parts without recreating and re-fading the object. It calls
    `look.fit(ob, o, PARTS)` in place of `inner.scale`, and the legacy chair index code is
    skipped for V2.
  - `update()` calls `look.tick`.
- **`src/web/GUI.html`:** added `<script src="reality-furniture.js?v=1">` before
  `reality-mode.js`, and bumped `reality-mode.js?v=14` to `v=15`. **Merge note:** the other
  agents also edit `reality-mode.js` and may bump the same `?v=`. Take the highest number and
  keep the new script tag.

Not touched: yaw, size, `seat` and `round` estimation (reality-structure.js), the lookalike splat
model, and the intent code. The V2 object keeps the `{group, inner, ring}` shape that
`updateModel` expects. When the OAK lookalike is ready, the body is hidden and the contact
shadow stays.

## Facing convention (unchanged, now asserted in a test)

The local stand-in frame is: x along L, y up, z along D, **back at local -Z**.

`reality-mode.js` sets `group.rotation.y = yaw + pi/2`. With scene x = -base y and
z = -base x, local -Z maps to the base direction **yaw + 90 deg** (the box's left side).

`reality-structure.js` fits a chair heading as `atan2(backrest offset) - pi/2`, so the backrest
lies at yaw + 90 deg. These agree. `reality_furniture_look_check.cjs` checks the transform
algebra for several yaws. It also runs the real `furnitureSurfaces` fitter on a seat with
raised returns along its -Y edge and asserts that the convention puts the back at -Y.

A couch has no measured back side: its yaw is pi-symmetric from the camera box. Its back can
therefore land on either long side, as it already could with the legacy meshes.

## Screenshots

These are in `artifacts/surroundings-2026-11-27/furniture-look/`:

- `before_*.png` / `after_*.png`: views `overview`, `close`, `top` and `lineup`.
- `compare_*.png`: before stacked over after.
- `after_fade_250ms.png`: the fade-in caught at 250 ms.
- `stats.txt`: `renderer.info` for each shot.

To regenerate: `tests/web/render_furniture_look.sh`. It runs headless Chrome with SwiftShader.
The harness `tests/web/furniture_look_harness.html` runs the real `RealityMode` code path, with
`RealityStructure` stubbed to return fixed boxes.

Render counts from `renderer.info` (headless fps means nothing, so none is quoted):

| scene | look | triangles | draw calls |
|---|---|---|---|
| room (9 objects) | current | 1844 | 58 |
| room (9 objects) | V2 | 2336 | 93 |
| lineup (7 objects) | current | 1720 | 46 |
| lineup (7 objects) | V2 | 2188 | 79 |

Each part is one draw call: about 10 per object, plus one for the shadow. A busy room of 15
pieces is about 170 calls, which a laptop handles easily at 30+ fps. If it ever matters, the
parts of one object can be merged into one geometry per refit, or instanced per kind.

## Tests

- `tests/web/reality_furniture_look_check.cjs` (and the pytest wrapper
  `tests/test_reality_furniture_look.py`, which skips without node) checks:
  - every layout stays inside the measured box across the SIZE ranges, stands on the floor,
    reaches H, and stays at or under 400 triangles;
  - the table top is thin and full-size, with 4 corner legs;
  - the round table is a centred pedestal;
  - the chair seat top equals `seat`, with everything above it at -Z, and the arms' hysteresis;
  - couch cushion counts (1/2/3) with hysteresis, the back cushions at -Z, and 4 short feet;
  - the legacy fallback scaling, the facing convention, and the flag (URL wins, remembered,
    default off);
  - the renderer lifecycle with the real three r128 in node: shared geometries, mesh reuse on
    refit, the fade-in/solid/fade-out sequence, material disposal, and instant removal.
- The existing `tests/web/reality_*.cjs` checks all still pass.
- `pytest tests/` from an unsourced shell gives 4 failed and 8 errors, the same as the baseline.
  The unsourced shell's `/usr/bin/node` is v12, which cannot parse the optional chaining in
  `reality-structure.js`, so the wrapper skips there (it needs node >= 16). With the nvm node
  (v22) it passes. `reality-furniture.js` itself loads on node 12.

## Open / next

- Not seen in the live GUI on the robot. The robot was off-limits tonight. Try
  `http://orin:8080/GUI.html?furnitureLookV2=1` with Reality on in a furnished room.
- Chair arms are inferred from width only. If the fitter ever reports arm returns, use those.
- The TV, bed and plant still use the legacy part lists, with the new shading only.
