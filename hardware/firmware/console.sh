#!/usr/bin/env bash
# ============================================================================
#  リーダのコンソール (USART2 / ST-Link VCP, 115200 8N1) を kermit で開く。
#
#  ST-Link の VCP は CTS も DCD も上げない。以下の 2 つが必須:
#     set carrier-watch off  … DCD が low なので、on だと connect が弾かれる
#     set flow none          … CTS が low なので、既定の RTS/CTS だと送信がブロックされる
#
#  起動直後から連続捕捉しているので、タグが送れば [rx] 行が勝手に流れてくる。
#  > stat
#  > cycle on      （検波トリガ式のタグを動かすとき）
#  抜けるには Ctrl-\ を押してから c、そのあと exit。
# ============================================================================
set -euo pipefail
PORT="${1:-}"
if [ -z "$PORT" ]; then
  PORT="$(ls /dev/cu.usbmodem* 2>/dev/null | head -1 || true)"
fi
[ -n "$PORT" ] || { echo "No /dev/cu.usbmodem* found. Nucleo は繋がっている?"; exit 1; }

HOLDER="$(lsof -t "$PORT" 2>/dev/null || true)"
if [ -n "$HOLDER" ]; then
  echo "!! $PORT は PID $HOLDER が掴んでいます。先に終了させてください:"
  ps -o pid,tty,command -p $HOLDER
  echo "   kermit なら Ctrl-\\ then c then exit / 強制なら: kill $HOLDER"
  exit 1
fi

echo ">> $PORT @115200 (抜ける: Ctrl-\\ then c then exit)"
exec kermit -C "set line $PORT, set speed 115200, set carrier-watch off, set flow none, set handshake none, connect"
