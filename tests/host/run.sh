#!/usr/bin/env bash
# Builds and runs the host tests against the working tree, or against a ref when SKETCH_REF is set.
# Usage: run.sh [test ...]   (ladder alloc pacing fade clocks ripple status brightness fadein; default all)   Exit status is non-zero on any failure.
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
work="$(mktemp -d)"; trap 'rm -rf "$work"' EXIT
"$here/prep.sh" "${SKETCH_REF:-$here/../..}" "$work" || exit 1
tests=("$@"); [ ${#tests[@]} -eq 0 ] && tests=(ladder alloc pacing fade clocks ripple status brightness fadein)
status=0
for t in "${tests[@]}"; do
  echo "== $t${SKETCH_REF:+ (sketch at $SKETCH_REF)}"
  g++ -O2 -std=gnu++17 -DNEWCODE -I "$here/stubs" -I "$work/sketch" -I "$here" -o "$work/$t" "$here/$t.cpp" || { status=1; continue; }
  "$work/$t" || status=1
done
[ $status -eq 0 ] && echo "ALL PASSED" || echo "FAILED"
exit $status
