#!/usr/bin/env bash
# Checks that the sketch at HEAD simulates the same as the sketch at <base-ref> at 100 % speed: for every look 0-3,
# seed 1-3 and effect config (none, all) the lit, sat, val, islands, islandsize and created figures of the S line and
# the island-end log (E lines) must be exactly equal after 5000 steps. hash and power legitimately differ. Not part
# of run.sh.
# Usage: parity.sh <base-ref>
set -euo pipefail
[ $# -eq 1 ] || { echo "usage: $0 <base-ref>" >&2; exit 2; }
here="$(cd "$(dirname "$0")" && pwd)"
work="$(mktemp -d)"; trap 'rm -rf "$work"' EXIT
steps=5000

for build in base head; do
  ref="$1"; [ $build = head ] && ref=HEAD
  mkdir -p "$work/$build"
  "$here/prep.sh" "$ref" "$work/$build"
  g++ -O2 -std=gnu++17 -DNEWCODE -I "$here/stubs" -I "$work/$build/sketch" -o "$work/$build/sim" "$here/sim.cpp"
  echo "built $build from $ref"
done

jobs=()
for build in base head; do
  for look in 0 1 2 3; do
    for seed in 1 2 3; do
      jobs+=("$build $look $seed none 0 0 0")
      jobs+=("$build $look $seed all 1 7 3")
    done
  done
done
printf '%s\n' "${jobs[@]}" | xargs -P "$(nproc)" -L 1 bash -c \
  '"$0/$1/sim" $5 $6 $7 1 2 $2 '"$steps"' $3 0 "$0/$1/out-$2-$3-$4.txt"' "$work" >/dev/null
# (sim args: ageing death island strength boost look steps seed preview out)

field() { grep '^S ' "$1" | tr ' ' '\n' | grep "^$2=" | cut -d= -f2; }
bad=0
declare -A maxrel
printf '%-4s %-4s %-5s %s\n' look seed fx result
for look in 0 1 2 3; do
  for seed in 1 2 3; do
    for fx in none all; do
      b="$work/base/out-$look-$seed-$fx.txt"; h="$work/head/out-$look-$seed-$fx.txt"
      diffs=""
      for k in lit sat val islands islandsize created; do
        bv="$(field "$b" $k)"; hv="$(field "$h" $k)"
        [ "$bv" = "$hv" ] || diffs="$diffs $k($bv->$hv)"
        rel="$(awk -v a="$bv" -v b="$hv" 'BEGIN { d = a - b; if (d < 0) d = -d; print (a > 0) ? d / a : 0 }')"
        maxrel[$k]="$(awk -v a="${maxrel[$k]:-0}" -v b="$rel" 'BEGIN { print (b > a) ? b : a }')"
      done
      cmp -s <(grep '^E ' "$b") <(grep '^E ' "$h") || diffs="$diffs ends($(grep -c '^E ' "$b")->$(grep -c '^E ' "$h") lines)"
      if [ -z "$diffs" ]; then res="equal (lit $(field "$h" lit) sat $(field "$h" sat) val $(field "$h" val) islands $(field "$h" islands) islandsize $(field "$h" islandsize) created $(field "$h" created) ends $(grep -c '^E ' "$h"))"
      else res="DIFFERS:$diffs"; bad=1; fi
      printf '%-4s %-4s %-5s %s\n' "$look" "$seed" "$fx" "$res"
    done
  done
done
for k in lit sat val islands islandsize created; do echo "largest relative difference, $k: ${maxrel[$k]}"; done
[ $bad -eq 0 ] && echo "PARITY OK" || echo "PARITY FAILED"
exit $bad
