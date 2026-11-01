#!/usr/bin/env bash
# Build the 424 kHz RING-BUFFER PIM reader with arduino-cli (Nucleo-L476RG).
# firmware-multitag/build.sh と同じ手順。スケッチだけ pim_ring424k/ を指す。
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

# ★opt=o2std（-O2）: 既定の -Os だと 1 セグメントの復号が平均 15 ms かかり、送信周期 50 ms で
#   追いつかない（2026-09-28 実測。docs/PLAN_TX_PERIOD.md）。
# ★pim_ring424k/build_opt.h（gcc へそのまま渡るフラグ）で UART 送信バッファを 2048 byte に
#   している。既定 64 byte だと [rx] 1 行の表示で ~4 ms ブロックし、その間走査が止まる。
#   build_opt.h を変えたらコアも作り直す必要があるので build/ を消してからビルドすること。
FQBN="STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_L476RG,opt=o2std"

arduino-cli compile -b "$FQBN" \
  --build-path "$HERE/build" \
  --export-binaries \
  "$HERE/pim_ring424k"

echo "---"
echo "ELF: $HERE/build/pim_ring424k.ino.elf"
