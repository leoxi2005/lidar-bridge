# macOS NDI runtime

Drop **`libndi.dylib`** here, then `npm run dist:mac` will bundle it into the .app
(the running app finds it via its Resources folder).

Where to get it:
- Install **NDI Tools for Mac** → the runtime is at `/usr/local/lib/libndi.dylib`
  (or a versioned `libndi.4.dylib`). Copy it here and rename to `libndi.dylib`.
- Or the **NDI SDK for Apple** → `lib/macOS/libndi.dylib`.

(Redistributing the NDI runtime is allowed under NDI's license with attribution.)
