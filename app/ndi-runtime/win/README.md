# Windows NDI runtime

Drop **`Processing.NDI.Lib.x64.dll`** here, then `npm run dist:win` will bundle it
into the app (the running app finds it via its resources folder).

Where to get the DLL (any one):
- Install **NDI Tools** for Windows → copy from its install folder
  (e.g. `C:\Program Files\NDI\NDI 6 Runtime\v6\Processing.NDI.Lib.x64.dll`).
- Or the **NDI SDK** → `Bin\x64\Processing.NDI.Lib.x64.dll`.
- Or copy the one TouchDesigner already uses.

(Redistributing the NDI runtime is allowed under NDI's license with attribution.)
