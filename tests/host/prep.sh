#!/usr/bin/env bash
# Copies the sketch into <out>/sketch for a host build: from a directory, or from a ref of the repo at $PIXELBUDS_REPO.
# The real network.h is swapped for the stub, webapi.h is dropped and secrets.h is replaced by placeholders.
# Usage: prep.sh <dir or ref> <out dir>
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
src="$1"; out="$2"
rm -rf "$out/sketch"; mkdir -p "$out/sketch"
if [ -d "$src" ]; then
  cp "$src"/PixelBuds.ino "$src"/*.h "$out/sketch/"
else
  repo="${PIXELBUDS_REPO:-$here/../..}"
  for f in $(/usr/bin/env git -C "$repo" ls-tree --name-only "$src" | grep -E '\.(h|ino)$'); do
    /usr/bin/env git -C "$repo" show "$src:$f" > "$out/sketch/$f"
  done
fi
rm -f "$out/sketch/secrets.h" "$out/sketch/webapi.h"
cp "$here/stubs/network_stub.h" "$out/sketch/network.h"
# Placeholder secrets: the real file is gitignored and the host build needs none of its values.
cat > "$out/sketch/secrets.h" <<'EOF'
#pragma once
#define WIFI_SSID "host-test"
#define WIFI_PASSWORD "host-test"
#define OTA_PASSWORD "host-test"
#define TZ_INFO "UTC0"
#define LATITUDE  0.0
#define LONGITUDE 0.0
EOF

# Shim for a sketch from before the frame clocks (one step per frame): count the frames that run a move or a
# mix, as moveTicks and mixTicks do in the newer sketch.
if ! grep -q 'moveTicks' "$out/sketch/PixelBuds.ino"; then
  sed -i \
    -e 's|^int frameCount = 1;|int frameCount = 1;\nuint32_t moveTicks = 0, mixTicks = 0, moveSeen = 0, mixSeen = 0;|' \
    -e 's|^    if (frameCount % moveSpeed == 0) {|&\n      if (moveSeen != (uint32_t)frameCount) { moveSeen = frameCount; moveTicks++; }|' \
    -e 's|^    if (frameCount % colorSpeed == 0) {|&\n      if (mixSeen != (uint32_t)frameCount) { mixSeen = frameCount; mixTicks++; }|' \
    "$out/sketch/PixelBuds.ino"
  if ! grep -q 'moveSeen = frameCount' "$out/sketch/PixelBuds.ino" || ! grep -q 'mixSeen = frameCount' "$out/sketch/PixelBuds.ino"; then
    echo "prep.sh: shim did not apply" >&2; exit 1
  fi
fi
