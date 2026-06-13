# LiDAR Bridge — App (Electron)

Real desktop implementation of the `LiDAR Bridge.dc.html` design handoff: 2D LiDAR
tracking → TouchDesigner. **Build-order steps 1–9 are implemented.**

## Stack (Option A from the root README)
Electron. The **main process** runs acquisition + the full tracking pipeline; the
**renderer** is the UI + canvases, fed per-frame state over IPC.

```
main/main.js        window, IPC, source selection, OSC/TUIO send timer,
                    projector window, record/playback
main/rplidar.js     RPLIDAR serial driver (Slamtec wire protocol, legacy SCAN)
main/simulator.js   hardware-free source (raycasts the prototype's room) — port "SIM"
main/pipeline.js    FILTER → TRANSFORM(placement, mm→m) → SUBTRACT → CLUSTER(DBSCAN)
                    → TRACK → ZONES(point-in-polygon) → MAP(homography)
main/homography.js  corner-pin homography (computeH / solve8 / invert3)
main/osc.js         dependency-free OSC 1.0 + TUIO 1.1 over UDP
preload.js          window.lidar bridge   projector-preload.js  window.projector bridge
renderer/           index.html + renderer.js (UI + live scan canvas + output monitor)
renderer/projector* the borderless projector output window (normalized 0–1 mapping)
```

## Run
```bash
npm install
npm start
```
**Without hardware:** COM PORT defaults to `SIM` → **CONNECT** streams the simulated
scene so you can exercise the whole pipeline (point cloud, background, tracking, zones,
OSC, warp, output, record/playback) with no sensor.

**With a real RPLIDAR:** pick the device, keep SERIAL, set the detected COM PORT +
baudrate, **CONNECT**.

## What each build step does (and how to test)
1. **Acquisition** — serial RPLIDAR + simulator; raw point cloud. → CONNECT (SIM).
2. **Placement + world space** — pos X/Y/rot in metres; FIT/zoom/pan. → edit PLACEMENT, wheel-zoom, drag-pan.
3. **Background subtraction** — CAPTURE BACKGROUND, then "Subtract background" + tolerance → walls vanish, movers go green.
4. **Clustering + tracking** — DBSCAN (ε 0.30 m) + nearest-neighbor IDs → blobs + TRACK table.
5. **Zones** — Draw Zone tool → click vertices → FINISH; ZONES tab badges; point-in-polygon occupancy.
6. **OSC output** — OUTPUT tab host/port/rate; `/lidar/trk/<id> x y vel`, `/lidar/zone/<slug> 0|1`. TD: OSC In.
7. **Warp + apply-to-output + TUIO** — WARP tab: corner-pin, drag corners/marquee/scale, rotate/flip, undo-redo; "Apply to output" → 0–1; TUIO 1.1 2Dobj.
8. **Visual output** — WINDOW/EXTENDED real OS windows (F/H/G/L/T/Esc); NDI config UI.
9. **Record/playback** — transport bar: RECORD a session, replay takes through the pipeline; multi-device config; warp scale-handle polish.

## Known issues from the reference plugin (handled)
- **360° array overflow** — angle binning is guarded with `% NBINS` (`pipeline.js`), so `angle == 360°` can never write past the buffer.
- **mm → m** — distances are converted once in the FILTER stage; all world math is in metres.

## Limitations / external dependencies
- **NDI broadcast** needs the **NDI SDK + the native `grandiose` module**, which aren't
  bundled here. The NDI config UI + START/STOP are wired; `lidar:ndi-start` lazy-loads
  `grandiose` and reports cleanly if absent (WINDOW/EXTENDED work without it). To enable:
  install the NDI runtime + `npm i grandiose`, then frames can be pushed from the mapping canvas.
- **Multi-sensor** — the device list manages multiple sensors with independent configs and
  lets you switch the active one. Simultaneous multi-sensor capture + fusion is future work.
- **EXPRESS scan** — step 1 uses the universally-supported legacy SCAN command for both
  STANDARD and EXPRESS; capsuled express decoding is a later optimization.

## Verifying headlessly (no screen-capture permission)
`LIDAR_AUTOSHOT=/tmp/shot.png npm start` connects SIM, renders, saves a PNG, and quits.
Optional `LIDAR_SHOT_JS` (renderer JS to run first), `LIDAR_SHOT_WAIT` (ms), `LIDAR_SHOT_PROJ=1` (capture the projector window).

## Notes
- `serialport` ships ABI-stable N-API prebuilds (loads under Electron without rebuild) and is lazy-loaded so SIM mode works even if the binding is missing.
- The app requests **no microphone / camera** permission — it uses none.
