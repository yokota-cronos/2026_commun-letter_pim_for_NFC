#!/usr/bin/env bash
# Flash the ring-buffer build over SWD. Run build.sh first.
# openocd は xPack(kashiwa) と Homebrew(mac) のどちらでも拾えるようにする。
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ELF="$HERE/build/pim_ring424k.ino.elf"
[ -f "$ELF" ] || { echo "missing $ELF — run ./build.sh first"; exit 1; }

OOCD="$(ls ~/.local/xpack/xpack-openocd-*/bin/openocd 2>/dev/null | head -1 || true)"
if [ -n "$OOCD" ]; then
  SCR="$(ls -d ~/.local/xpack/xpack-openocd-*/openocd/scripts | head -1)"
else
  OOCD="$(command -v openocd || true)"
  [ -n "$OOCD" ] || { echo "openocd not found (xPack にも PATH にも無い)"; exit 1; }
  SCR="$(cd "$(dirname "$OOCD")/../share/openocd/scripts" && pwd)"
fi
echo ">> openocd = $OOCD"
echo ">> scripts = $SCR"

"$OOCD" -s "$SCR" -f board/st_nucleo_l4.cfg \
  -c "program \"$ELF\" verify reset exit"
