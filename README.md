# Handoff: LiDAR Bridge — 2D LiDAR Tracking → TouchDesigner

## Overview
**LiDAR Bridge** is a standalone desktop application that connects to a 2D planar LiDAR
sensor (Slamtec RPLIDAR family / Hokuyo), tracks people and objects as they move through a
space, lets the operator draw trigger zones and visually calibrate a coordinate mapping
(warp / corner-pin), and streams the resulting data into **TouchDesigner** over OSC / TUIO —
plus a live visual output (window / extended display / NDI) for projector calibration.

It is the **application tier** of a two-tier system:

```
┌─────────────┐   raw points   ┌────────────────────────────────────────┐   OSC/TUIO    ┌──────────────┐
│  LiDAR HW   │ ─────────────▶ │  LiDAR Bridge  (THIS APP)                │ ───────────▶  │ TouchDesigner│
│ RPLIDAR/etc │  angle+dist    │  acquire → filter → track → map → output │   + NDI video │              │
└─────────────┘                └────────────────────────────────────────┘               └──────────────┘
```

The user previously built a TouchDesigner C++ CHOP plugin (`reference_plugin/`) that does the
**acquisition tier** (read sensor → raw point array inside TD). This app instead does
acquisition **outside** TD and adds everything the raw plugin lacks: background subtraction,
blob tracking, trigger zones, coordinate warping, and network output. The plugin source is
included as an authoritative reference for the acquisition + filtering layer.

---

## About the Design Files
The file `LiDAR Bridge.dc.html` in this bundle is a **design reference created in HTML** — an
interactive prototype showing the intended look, layout, and behavior. **It is not production
code to ship.** All sensor data in it is simulated (a 2D raycaster fakes the scan; tracked
"people" are scripted movers). OSC/TUIO/NDI output is represented visually, not actually sent.

The task is to **recreate this design in a real desktop application** that talks to real
hardware and really emits OSC/TUIO/NDI. Use the prototype for exact layout, interactions,
colors, typography, and the behavior of every control; use this README for the system
architecture and algorithms behind the UI.

> `.dc.html` is a self-contained component format. To view it, open it in a browser. The
> markup is plain inline-styled HTML; the logic is a single JS class (`class Component`) near
> the bottom of the file — read that class to see exactly how every interaction behaves
> (background subtraction, homography warp, marquee-select transform, undo/redo, etc.).

## Fidelity
**High-fidelity.** Colors, typography, spacing, and interactions are final. Recreate the UI
faithfully using the target stack's component library, then wire it to the real pipeline.

---

## Recommended Architecture & Stack

This app needs three things a browser can't do alone: talk to serial/network hardware, run a
real-time tracking loop, and emit UDP (OSC/TUIO) + NDI video. Recommended approach:

### Option A — Electron + Node (recommended; reuses the HTML UI directly)
- **Renderer (UI):** Port `LiDAR Bridge.dc.html` to React (or keep as a web view). All the
  canvas rendering (point cloud, sweep, blobs, zones, warp overlay) maps 1:1 to a `<canvas>`
  in the renderer.
- **Main process (pipeline):** Node.js handles:
  - **Acquisition:** `serialport` for USB RPLIDAR, or a native N-API addon wrapping Slamtec's
    `sl_lidar` SDK (the reference plugin already links it — the device code is reusable).
  - **OSC:** `osc` / `node-osc` (UDP). **TUIO:** build TUIO 1.1 packets over OSC.
  - **NDI:** `grandiose` or `node-ndi` bindings to the NDI SDK; feed it the rendered mapping
    canvas frames.
  - **IPC:** stream point frames + track state from main → renderer over `ipcRenderer`.

### Option B — Python backend + web UI
- **Backend:** `rplidar` (PyPI) or Slamtec SDK via `pybind11`; `python-osc` for OSC/TUIO;
  `ndi-python` / `cyndilib` for NDI. Run the tracking loop in Python.
- **UI:** serve the HTML and talk over a local WebSocket, or wrap with `pywebview`.

### Option C — openFrameworks / C++ (lowest latency, most work)
- Reuse the device code from the reference plugin directly; `ofxOsc`, `ofxNDI`, `ofxGui`.
- Choose only if sub-frame latency matters more than development speed.

**For most installation work, Option A is the sweet spot** — the UI is already HTML, and Node
covers serial + OSC + NDI with mature packages.

---

## System Pipeline (the real work behind the UI)

The app runs a loop, ideally on a worker thread, at the sensor's scan rate (~10–40 Hz):

```
1. ACQUIRE   read one full scan → array of (angle°, distance_m, quality, flag)
2. FILTER    drop quality<150 or flag<2 (if Quality filter ON); clip to [distMin, distMax]
3. TRANSFORM apply sensor placement (pos X/Y, rotation) → world-space (x,y) in meters
4. SUBTRACT  if background captured: keep point only if distance < background[angle] − tol
5. CLUSTER   group surviving points into blobs (DBSCAN, ε≈0.30 m, minPts≈3)
6. TRACK     match blobs to existing track IDs (nearest-neighbor + max-jump gate); assign/age IDs
7. ZONES     for each zone polygon, test point-in-polygon for any track → occupied true/false
8. MAP       if warp ON: apply homography H to each track (x,y) → normalized (u,v) in [0,1]
9. OUTPUT    send OSC/TUIO at the configured send-rate; render mapping canvas for window/NDI
```

### Key algorithms (all implemented in the prototype's JS — read it for exact code)

**Background subtraction** (`captureBackground`, point `.fg` flag): capture a baseline array
`background[angle] = distance` of the empty room. A point at a given angle is *foreground*
(a person/object) only if its distance is meaningfully closer than the baseline:
`isForeground = liveDistance < background[angle] − tolerance`. Tolerance is user-set (5–60 cm).
Only foreground points proceed to clustering — this is what removes walls/static geometry so
visuals don't stick to them. Consider an optional **adaptive** mode (slowly blend the current
scan into the baseline) to fight sensor drift.

**Clustering → blobs:** DBSCAN over foreground points in world space, ε ≈ 0.30 m. Each cluster's
centroid is a blob; estimate radius from extent.

**Tracking (ID assignment):** keep a list of active tracks. Each frame, match new blobs to the
nearest existing track within a max-jump gate (e.g. 0.6 m); unmatched blobs spawn new IDs;
tracks unmatched for N frames are dropped. Velocity = Δposition / Δt (used for the VEL column
and the direction arrow).

**Coordinate warp (homography / corner-pin):** the operator places a quadrilateral (4 source
corners in meters) that maps to the unit square `(0,0)(1,0)(1,1)(0,1)`. Compute the 3×3
perspective transform **H** by solving the 8-equation linear system (see `computeH` / `solve8`
in the prototype — Gaussian elimination). Then for any world point:
`(u,v,w) = H·(x,y,1)`, `u/=w; v/=w`. Points with u,v outside [0,1] are outside the mapped area
("OUT") and are typically clipped by the TD patch. **Transform** ops (move/scale/rotate/flip)
operate on the 4 source corners as a group (marquee-select a subset, drag to move with Shift =
axis-lock, drag a bounding-box handle to scale about the opposite corner). Full undo/redo of
all warp edits (Ctrl+Z / Ctrl+Shift+Z), history depth ~80.

---

## OSC / TUIO Output Spec

Configurable target **host** + **port** (defaults: OSC `127.0.0.1:7000`, TUIO `127.0.0.1:3333`)
and **send rate** (10 / 30 / 60 Hz). When **warp "Apply to output"** is ON, coordinates are the
normalized `u,v ∈ [0,1]`; when OFF, raw meters `x,y`.

### OSC mode (default — recommended for TouchDesigner)
Per tracked object, each send tick:
```
/lidar/trk/<id>   <x|u>  <y|v>  <vel>      e.g.  /lidar/trk/01  0.185  0.753  0.41
```
Per zone, on state change (and/or every tick):
```
/lidar/zone/<slug>   <0|1>                  e.g.  /lidar/zone/stage_front  1
```
`<slug>` = zone name lowercased, spaces → underscores. TD reads via an **OSC In DAT/CHOP**.

### TUIO mode (for TD's built-in blob/touch logic)
TUIO 1.1 2D object profile over OSC:
```
/tuio/2Dobj set <sessionId> <classId> <x|u> <y|v> <angle> ...
```
Send a `/tuio/2Dobj alive [...ids]` + `/tuio/2Dobj fseq <n>` bundle per frame per the TUIO spec.

---

## Visual Output (window / extended / NDI)

The "OUTPUT MONITOR" renders the mapping in normalized 0–1 space: registration corner targets
`(0,0)…(1,1)`, a 0–1 grid, center crosshair, zones and tracked points (all run through the
homography). Three output modes:

- **WINDOW** — open a separate borderless window the operator drags to the projector; **F** =
  fullscreen, **H** = hide UI chrome, **G/L/T** = grid/labels/test-pattern, brightness slider.
- **EXTENDED** — open that window positioned on the secondary display automatically.
- **NDI** — broadcast the mapping canvas as an NDI video source. Config: **source name**, a
  **custom W×H resolution** (free numeric entry + 720p/1080p/1440p/4K presets, live aspect-ratio
  readout), **FPS** (30/60), and **scale mode FIT / FILL / STRETCH** (letterbox / crop / distort
  relative to the canvas's native aspect). START/STOP toggles the stream; TD receives via **NDI In**.

In the real app, WINDOW/EXTENDED are real OS windows on the chosen display; NDI uses the NDI SDK
to publish frames captured from the mapping canvas at the chosen resolution/FPS.

---

## Screens / Views

Single-window desktop dashboard, dark technical theme. Three-column layout under a top bar,
with a bottom transport bar, plus a fullscreen projector-output overlay.

### Top Bar (height 50px, bg `#0e1115`, 1px bottom border `rgba(255,255,255,0.07)`)
- **Logo lockup** "LiDAR**Bridge**" (Bridge in `#00e5ff`) + mono subtitle "TRACKING → TOUCHDESIGNER".
- **Stream toggle** — STREAMING / PAUSED (cyan outline button, play/pause glyph).
- **Live stats** (mono): RENDER fps · POINTS count · LATENCY ms.
- **Protocol pill** (right) — shows OSC/TUIO + host:port, green pulsing dot; click → OUTPUT tab.
- **PROJECT** button — opens output per current Output Mode.
- **RECORD** button — red dot, starts/stops recording.

### Left Column (width 248px, bg `#0c0f13`) — scrollable
- **DEVICES** list — sensor cards (RPLIDAR A3 / S2E / A2): status dot (green online / red offline),
  name, connection (SERIAL COMx / NET ip), range, Hz. Selected card has cyan left-bar + tint.
  "+ ADD DEVICE" dashed button.
- **CONNECTION · <sensor>** — SERIAL/NETWORK segmented toggle. Serial: COM Port input + Baudrate
  select (115200 / 256000 / 1000000). Network: IP + Port inputs + TCP/UDP toggle. Scan Mode:
  STANDARD / EXPRESS.
- **ACQUISITION** — Precision 1–4 segmented (shows `360 × precision samples/scan`); Dist Min/Max
  (m) inputs; Coordinate POLAR/CARTESIAN; Quality FILTER ON/OFF.
- **BACKGROUND MASK** — status label; CAPTURE BACKGROUND button; once captured: "Subtract
  background" toggle (green), TOLERANCE slider (5–60 cm), CLEAR BACKGROUND.
- **PLACEMENT · room map** — Pos X / Y / Rot inputs; AUTO-LEVEL FLOOR PLANE button.

### Center — Live Scan Canvas (flex-fill, radial-gradient bg)
- Full-bleed `<canvas>`: cartesian grid, cyan range rings, rotating radar sweep line, raw point
  cloud (bright = fresh), tracked blobs (colored ring + crosshair + ID/coord label + velocity
  arrow + motion trail), trigger-zone polygons (filled cyan, brighter when occupied), sensor
  marker at origin. Background mask: foreground points render **green**, walls hidden + faint
  orange dashed baseline outline.
- **Tool palette** (top-left, floating): Select/Pan, Draw Zone, Warp/Corner-pin, Calibrate, Measure.
- **Scene legend** (top-right). **Cursor readout** (bottom-left, world x/y). **Zoom −/FIT/+** (bottom-right).
- Pan = drag (Select tool); zoom = wheel (cursor-anchored).

### Right Column (width 312px, bg `#0c0f13`) — tabbed: TRACK / ZONES / WARP / OUTPUT
- **TRACK** — table of tracked objects: ID (colored dot), X/Y (m), VEL, ZONE; footer TRACKED count + cluster ε.
- **ZONES** — zone cards: visibility toggle, name, TRIGGERED/IDLE badge, vertex count + OSC slug;
  "+ DRAW NEW ZONE".
- **WARP** — "Apply to output" toggle (sends 0–1); **TRANSFORM** (Select All / Clear / Undo / Redo;
  ROTATE numeric-degree input + ↺/↻; Flip H/V); **CORNER-PIN** numeric corner inputs (TL/TR/BR/BL,
  each → its 0/1 target); **OUTPUT MONITOR** (live normalized canvas, NDI badge when streaming);
  **OUTPUT MODE** WINDOW/EXTENDED/NDI + NDI config; action button (open window / start NDI).
- **OUTPUT** — OSC/TUIO segmented; Target Host + Port; Send Rate (10/30/60 Hz); address-pattern
  preview; live message monitor (scrolling sent messages).

### Bottom Transport (height 62px, bg `#0e1115`)
- RECORD button + timecode; scrubbable timeline with playhead; recorded "takes" chips; play/skip transport.

### Projector Output Overlay (fullscreen, z-index 80) — for WINDOW/fullscreen fallback
- Black bg, normalized mapping render; top + bottom chrome (title, warp-state pill, EXIT,
  FULLSCREEN, GRID/LABELS/TEST toggles, BRIGHT slider, HIDE UI). Keyboard: F/H/G/L/T/Esc.

---

## Interactions & Behavior
- **Stream toggle** pauses/resumes the acquisition loop (canvas + stats freeze).
- **Device select** loads that sensor's connection/acquisition config into the left panel.
- **Draw Zone** — click points on the canvas to build a polygon; FINISH (≥3 pts) creates a zone;
  CANCEL discards. New zones get an auto OSC slug.
- **Warp**: marquee-drag selects corners; drag selected corners to move (Shift = axis-lock); drag
  a bounding-box handle (offset just outside the selection) to scale about the opposite corner;
  ROTATE by typed degrees or ↺/↻; Flip H/V; Ctrl+Z / Ctrl+Shift+Z undo/redo (depth ~80).
- **Background capture** snapshots the static scan; subtraction + tolerance filter live.
- **Output mode** switches the PROJECT/action button behavior (open window / extended / start NDI).
- **Record** accumulates a timecode and produces a "take" chip on stop (for record/playback of sessions).

## State Management
Top-level app state (see the prototype's `class Component` `state` object for the exact shape):
streaming/paused, selected sensor + per-sensor config map (connType, comPort, baudrate, netProto,
ipAddr, ipPort, scanMode, precision, quality, distMin/Max, coordSys), background (captured,
subtract, tolerance), placement (x/y/rot), active right-tab + tool, zones[], tracked objects[],
warp (corners[4], enabled, selection, undo/redo stacks, rotation step), output (protocol, host,
port, sendRate, log[]), output-mode (window/extended/ndi + NDI name/W/H/fps/fit/on), projector
(open, grid/labels/test/chrome/brightness), recording (on, time, takes[]).

The **pipeline state** (live point frame, clusters, tracks, background baseline, homography H)
lives in the worker/main process and streams to the UI each frame.

---

## Design Tokens
**Colors**
- Background base `#0a0b0e`; panels `#0c0f13`; bars `#0e1115`; cards `#10141a`; inputs `#0e1216` / `#0b0e11`.
- Borders `rgba(255,255,255,0.07)`; hairlines `rgba(255,255,255,0.06)`.
- Primary accent (cyan) `#00e5ff`; cyan tints `rgba(0,229,255,0.06–0.25)`.
- Foreground/positive green `#39ff7a`; warning amber `#ffb000`; record/negative red `#ff4d5e` / `#ff4d5e`.
- Text: primary `#e8ecf1`; secondary `#9aa3ad` / `#cdd4dc`; muted `#717a84`; faint `#5b636d`; disabled `#454c54`.
- Track-blob palette: `#00e5ff`, `#39ff7a`, `#ffb000`, `#ff5d8f`.

**Typography**
- UI sans: **IBM Plex Sans** (400/500/600/700). Body 13px.
- Mono (labels, numbers, code): **IBM Plex Mono** (400/500/600). Label sizes 8.5–11px, letter-spacing 0.04–0.18em, often uppercase.

**Spacing / radius / misc**
- Panel padding 14px; control gaps 6–9px. Inputs/buttons height 28–30px; primary actions 38px.
- Radius: inputs/buttons 5–8px; pills 20px; cards 7–8px.
- Sweep period ≈ 0.85 s/rev; point fade ≈ 900 ms; toggle knob transition 0.15s.

## Assets
No external image assets — all iconography is inline SVG (logo radar mark, tool icons,
projector/transport glyphs). Fonts loaded from Google Fonts (IBM Plex Sans + Mono); in a real
app, self-host them. No Anthropic brand assets are used.

---

## Reference: TouchDesigner C++ Plugin (`reference_plugin/`)
The user's existing Slamtec CHOP plugin — authoritative for the **acquisition + filtering** layer.
Reuse its device logic (it links Slamtec's `sl_lidar` SDK).
- `RPLidarDevice.cpp/.h` — connect (serial baud auto-try / TCP / UDP), `startScan`/`startScanExpress`,
  grab one scan, the quality/flag filtering, the polar→cartesian conversion.
- `SlamtecCHOP.cpp/.h` — channel output model: `angle / distance / quality / flag` (polar) or
  `x / y / quality / flag` (cartesian), `360 × precision` samples, info channels.
- `Parameters.cpp/.h` — the parameter set the left panel mirrors (connection, scan mode, precision,
  distance, coordinate system, quality).

**Two known issues to fix when reusing the device code:**
1. **Array overflow at 360°:** `halfAngle = floor(tempAngle × precision)` can equal `360×precision`
   when `tempAngle == 360`, writing one past the data array. Guard the index (`% (360*precision)`
   or clamp).
2. **Cartesian units:** distance is in **millimeters** — convert to **meters** for the app's world
   space (or expose a unit parameter). The app/world math assumes meters throughout.

---

## Files
- `LiDAR Bridge.dc.html` — the full high-fidelity design + behavior reference (open in a browser;
  read the `class Component` block for exact interaction logic and the math: `computeH`, `solve8`,
  `captureBackground`, homography `normPt`, marquee transform, undo/redo).
- `reference_plugin/` — the user's Slamtec TouchDesigner CHOP plugin source (acquisition reference).

## Suggested Build Order
1. Acquisition: connect to one RPLIDAR over serial, read scans, render the raw point cloud in the UI.
2. Placement transform + meters world space; FIT/zoom/pan canvas.
3. Background capture + subtraction; verify walls disappear.
4. DBSCAN clustering + nearest-neighbor tracking with stable IDs.
5. Trigger zones + point-in-polygon occupancy.
6. OSC output (objects + zones); confirm reception in TouchDesigner.
7. Warp homography + "Apply to output" (normalized 0–1); TUIO mode.
8. Visual output: window → extended display → NDI.
9. Record/playback of sessions; multi-sensor; calibration polish.
