# LiDAR Bridge — App (Electron)

Real desktop implementation of the `LiDAR Bridge.dc.html` design handoff.
**Build Order step 1** is implemented: connect to an RPLIDAR over serial, read
scans, and render the raw point cloud in the UI (steps 2–9 to follow).

## Stack
Electron (Option A from the root README). The **main process** does acquisition;
the **renderer** does the UI + canvas, fed point frames over IPC.

```
main/rplidar.js    RPLIDAR serial driver — speaks the Slamtec wire protocol directly
                   (the TD plugin linked Slamtec's SDK; Node has no binding, so we
                   implement the protocol: STOP/RESET/GET_INFO/GET_HEALTH/SCAN +
                   5-byte measurement-node parsing).
main/simulator.js  Hardware-free source: raycasts the prototype's room, emits the
                   same `scan` frames. Selected with COM port = "SIM".
main/main.js       Window, IPC, source selection, FILTER stage (quality / dist clip,
                   mm→m), streams interleaved [angleDeg, distM] frames to the renderer.
preload.js         contextBridge `window.lidar` API.
renderer/          UI (faithful port of the design) + live scan canvas.
```

## Run
```bash
npm install
npm start
```

### Without hardware (default)
The COM PORT field defaults to **`SIM`** — click **CONNECT** to stream the built-in
simulated scan and verify the full render path.

### With a real RPLIDAR
1. Plug in the sensor (USB → serial).
2. Pick the device card, keep **SERIAL**, set **COM PORT** to the detected port
   (the dropdown lists serial ports; on macOS e.g. `/dev/tty.usbserial-*`, on
   Windows `COM4`), and the matching **BAUDRATE** (A1 115200 · A2 115200 · A3/S 256000
   or 1000000).
3. Click **CONNECT**. The motor spins (DTR), the driver runs the legacy `SCAN`
   command, and points stream live.

> Step 1 uses the **legacy SCAN** command, supported by every RPLIDAR. EXPRESS-scan
> decoding (the capsule format) is a later optimization; the STANDARD/EXPRESS toggle
> is wired into state but both currently use legacy SCAN.

## Notes
- Verify rendering headlessly (no screen-capture permission needed):
  `./node_modules/.bin/electron tools/shot.js /tmp/shot.png`
- `serialport` ships ABI-stable N-API prebuilds, so it loads under Electron without a
  rebuild. It's lazy-loaded, so SIM mode and the UI work even if the native binding is
  unavailable.
- NETWORK (TCP/UDP) acquisition, background subtraction, clustering/tracking, zones,
  OSC/TUIO output, warp, and NDI are the subsequent build-order steps.
