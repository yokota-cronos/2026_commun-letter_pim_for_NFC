#!/usr/bin/env python3
"""タグの負荷変調の深さをリアルタイム表示する調整ツール。

なぜこれが要るか（2026-08-18 の実測より）:
  タグが復調できるかどうかは、突き詰めると「MOSFET を ON にしたときリーダが感じる
  磁界がどれだけ変化するか」の 1 点に尽きる。検波出力の電圧や距離を手探りするより、
  この量を直接見た方が速い。実測では
      遠い : 深度 0    -> リーダの生波形はオール 0（何も見えない）
      中間 : 深度あり  -> tag_id/data の復号に成功
      近い : 深度不明  -> 受信機が飽和して生波形オール 1
  という非常に狭い窓しかなかった。同調(C_tune)が取れていれば窓はもっと広いはずで、
  この値を最大化する作業がそのまま同調合わせになる。

原理:
  DIO を静的に L / H に固定して MOSFET を OFF / ON し、その各々でリーダの
  ST25R3911B に MEASURE_AMPLITUDE を実行させて REG_AD_RESULT を読む。その差が深度。

使い方:
  python3 host/pim_tune.py                    # DIO0+DIO2（既定のゲート 2 本）
  python3 host/pim_tune.py --pins 0+2
  python3 host/pim_tune.py --scan             # どの DIO にゲートが繋がっているか総当たり
  コイルを動かす / C_tune のトリマを回す → バーが最大になる点を探す。
  目標: 差 >= 10 LSB。0 のままなら共振が合っていないか結合が無い。
"""
from __future__ import annotations
import argparse, ctypes, glob, os, select, sys, termios, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dwf_ad import AD, dwf


class Reader:
    """ST-Link VCP 経由のリーダコンソール。CTS/DCD が出ないので raw で開く。"""

    def __init__(self, port: str | None = None):
        # macOS: /dev/cu.usbmodem*、Linux: /dev/ttyACM*（ST-Link VCP は CDC-ACM）
        self.port = port or (sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
                             or [None])[0]
        if not self.port:
            raise SystemExit("/dev/cu.usbmodem* も /dev/ttyACM* も見つからない。Nucleo は繋がっている?")
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        a[6] = list(a[6]); a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, a)

    def cmd(self, line: str, wait: float = 1.2) -> str:
        os.write(self.fd, b"\r"); time.sleep(0.12)          # 行バッファを空にする
        termios.tcflush(self.fd, termios.TCIFLUSH)
        os.write(self.fd, line.encode() + b"\r")
        out = b""; t0 = time.time()
        while time.time() - t0 < wait:
            if select.select([self.fd], [], [], 0.08)[0]:
                try: out += os.read(self.fd, 8192)
                except OSError: pass
        return out.decode("utf-8", "replace")

    def rf_amplitude(self):
        """'meas' の RF amplitude を返す（0..255）。取れなければ None。"""
        s = self.cmd("meas")
        if "[meas]" not in s:
            return None
        for tok in s.replace("=", " ").split():
            if tok.startswith("0x") and len(tok) == 4:
                return int(tok, 16)
        return None

    def close(self):
        os.close(self.fd)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pins", default="0+2",
                   help="ゲートを繋いだ DIO を + で（既定 0+2）")
    p.add_argument("--port", help="/dev/cu.usbmodem...")
    p.add_argument("--settle", type=float, default=0.25, help="切替後の待ち [s]")
    p.add_argument("--scan", action="store_true",
                   help="DIO0..15 を総当たりして、どのピン(組)で変調が起きるか探す")
    p.add_argument("--scan-max", type=int, default=8, dest="scan_max",
                   help="--scan で調べる DIO の上限番号+1 (既定 8)")
    a = p.parse_args()
    pins = [int(x, 0) for x in a.pins.split("+")]
    mask = 0
    for x in pins:
        mask |= 1 << x

    rd = Reader(a.port)
    ad = AD(); h = ad.h

    def drive_mask(m: int, level: int):
        dwf.FDwfDigitalOutReset(h)                       # DigitalOut を解放してから
        dwf.FDwfDigitalIOOutputEnableSet(h, ctypes.c_uint(m))
        dwf.FDwfDigitalIOOutputSet(h, ctypes.c_uint(m if level else 0))
        dwf.FDwfDigitalIOConfigure(h)
        time.sleep(a.settle)

    def drive(level: int):
        drive_mask(mask, level)

    def depth_of(m: int):
        """そのピン集合を OFF/ON したときの RF amplitude 差。None = 測れず。"""
        drive_mask(m, 0); off = rd.rf_amplitude()
        drive_mask(m, 1); on = rd.rf_amplitude()
        if off is None or on is None:
            return None
        return off - on

    def scan():
        n = a.scan_max
        print("リーダの磁界 ON。DIO0..%d を総当たりします。" % (n - 1))
        print("この基板は anti-series の 2 石なので、**2 本同時**でないと導通しません。")
        print("※ コイルがリーダの磁界に十分入っていないと全部 0 になります。\n")
        print("--- 単独 ---")
        singles = {}
        for i in range(n):
            d = depth_of(1 << i)
            singles[i] = d
            print("   DIO%-2d          -> %s" % (i, "meas失敗" if d is None else "%+d LSB" % d))
        print("\n--- 2 本の組み合わせ ---")
        hits = []
        for i in range(n):
            for j in range(i + 1, n):
                d = depth_of((1 << i) | (1 << j))
                if d is None:
                    continue
                if abs(d) >= 3:
                    hits.append(((i, j), d))
                    print("   DIO%-2d + DIO%-2d  -> %+d LSB   <== ここにゲートが繋がっている" % (i, j, d))
        if not hits:
            print("   どの組でも変調なし。")
            print("   -> コイルが磁界から遠い / C_tune 未実装 / GND 未接続 のいずれかを疑う。")
        else:
            print("\n結論: --pins %s" % "+".join(str(x) for x in hits[0][0]))

    try:
        print("リーダの磁界を ON にします...")
        rd.cmd("auto off"); time.sleep(0.3)
        rd.cmd("field on"); time.sleep(0.5)
        if a.scan:
            scan()
            return
        print("DIO %s を OFF/ON しながらリーダの RF amplitude を読みます。" % pins)
        print("コイルの距離 / C_tune のトリマを動かして、差が最大になる点を探してください。")
        print("目標: 差 >= 10 LSB。  Ctrl-C で終了。\n")
        print("    OFF   ON   差    ")
        best = 0
        while True:
            drive(0); off = rd.rf_amplitude()
            drive(1); on = rd.rf_amplitude()
            if off is None or on is None:
                print("  (meas 応答なし。リーダのコンソールを他で開いていないか確認)")
                time.sleep(0.5); continue
            d = off - on
            best = max(best, abs(d))
            bar = "#" * min(60, abs(d) * 2)
            note = ""
            if abs(d) == 0:      note = "  <- 変調なし。共振か結合を疑う"
            elif abs(d) < 5:     note = "  <- 弱い"
            elif abs(d) >= 10:   note = "  <- OK"
            print("   0x%02X 0x%02X %+4d  %-60s%s (best %d)" % (off, on, d, bar, note, best))
    except KeyboardInterrupt:
        print("\n終了。best = %d LSB" % best)
    finally:
        drive(0)
        dwf.FDwfDigitalIOOutputEnableSet(h, ctypes.c_uint(0))
        dwf.FDwfDigitalIOConfigure(h)
        ad.close(); rd.close()


if __name__ == "__main__":
    main()
