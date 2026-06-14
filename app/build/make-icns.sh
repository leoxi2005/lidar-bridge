#!/bin/bash
# Build icon.icns from icon.png (macOS only — uses sips + iconutil).
# Run from app/:  node build/make-icon.js && bash build/make-icns.sh
set -e
cd "$(dirname "$0")"
rm -rf icon.iconset && mkdir icon.iconset
for s in 16 32 64 128 256 512; do
  sips -z $s $s icon.png --out icon.iconset/icon_${s}x${s}.png >/dev/null
  d=$((s*2)); sips -z $d $d icon.png --out icon.iconset/icon_${s}x${s}@2x.png >/dev/null
done
iconutil -c icns icon.iconset -o icon.icns
rm -rf icon.iconset
echo "wrote $(pwd)/icon.icns"
