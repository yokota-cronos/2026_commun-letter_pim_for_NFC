#!/usr/bin/env python3
"""多タグ PIM ドライバ — AD3 の DIO で N 個の疑似タグを同時に駆動する。

  検波出力(DC)   --> Scope CH1 (1+/1-)   リーダの磁界 ON を検出するトリガ源
                                        ★--free-run では «使わない»（下記）
  アンテナ生波形 --> Scope CH2 (2+/2-)   搬送波 13.56 MHz の周波数・振幅の実測
  DIO0..15       --> 各タグの CTRL       検出と同時に全タグが PIM 送信を開始
  1- / 2- と AD3 の GND はタグ／NUCLEO と共通にする（浮くと MOSFET が誤動作する）。

トリガに使うのは «CH1 だけ»（2026-09-22 の配線変更後も同じ）。CH2 は生の 13.56 MHz で、
検波用の --det-fs 1 MHz では折り返してノイズに見えるだけなので、トリガ経路では明示的に
無効化する（arm_detector）。CH2 は `carrier` モードで「リーダが今どれだけ出しているか」を
測るのに使う。

物理層の定義は host/pim.py が唯一の真実。このファイルは pim.py が作った波形を
DIO のビットパターンに詰め替えて配るだけで、PIM の中身には一切手を触れない。

なぜ AWG(W1/W2) でなく DIO か:
  * W1/W2 は 2 本しかない = 最大 2 タグ。DIO は 16 本 = 最大 16 タグ。
  * DIO は FDwfDigitalOutIdleSet(ch, Low) で「停止時 Low」が確定する。AWG の
    idle=offset だと 1.65V が出続け、MOSFET が ON のままアンテナを短絡する
    （RESULT.md 注8）。DIO に移すとこの危険が構造的に消える。

DigitalOut の制約（RESULT.md P-3b）:
  * Wait/Run/Repeat/TriggerSource は instrument-global（ch ごとに持てない）。
    タグごとのバックオフは「各chのバッファ先頭に入れるアイドル長」で表現する。
  * 分周器は整数なので sps_tu を誤ると副搬送波が最大 1.7% ずれる。
    -> pick_sps_tu() が実機の fDIO から誤差最小の sps_tu を自動選択する。

usage:
    python3 host/pim_multitag.py info                 # 実機の能力を表示するだけ
    python3 host/pim_multitag.py carrier --port /dev/cu.usbmodem11103
                                                      # CH2 でリーダの出力電圧を実測
    python3 host/pim_multitag.py carrier --port /dev/cu.usbmodem11103 --rege-sweep
                                                      # VDD_RF を 16 段振って実測
    python3 host/pim_multitag.py selftest             # 検波不要。PC トリガで数回撃つ
    python3 host/pim_multitag.py verify               # 検波不要。フリーランで復調を確認
    python3 host/pim_multitag.py run                  # 検波トリガで延々と送信（撃つだけ）
    python3 host/pim_multitag.py link                 # ★本番: 検波トリガ + リーダ復調の照合
    python3 host/pim_multitag.py link --free-run --reader-period 100
                                                     # ★検波なし: タグが 100ms ごとに自走送信
    python3 host/pim_multitag.py run --tags "0:42:100:120"\
    --level 0.877 --hyst 0.218 --vrange 5.0 --holdoff 0.05
    python3 host/pim_multitag.py link \
    --tags "0:160:100, 1:102:100, 2:4:100, 3:76:100, 4:224:100" \
    --port /dev/cu.usbmodem11103 --reader-sps 94800 \
    --random-data --random-backoff --seed 12345 \
    --reader-period 200 --reader-gap 150 --seconds 600 --tsv
    
    # 424k 10MSPS に対応する
    python3 host/pim_multitag.py link \
    --random-data --random-backoff --seed 42 \
    --reader-period 200 --reader-gap 150 --seconds 60 --tsv \
    --reader-sps 23700

    # 推奨コマンド
    python3 host/pim_multitag.py link --free-run --random-data --random-backoff --seed 42 \
  --reader-period 200 --reader-gap 150 --dio-config 4 --reader-rate 3 \
  --tags "0:40:250, 2:42:250, 3:43:250, 4:44:250" --seconds 1800 --tsv

    # 論文と同じ 100 ms 周期（--tx-engine fast が既定。2026-09-28 実測 成功率 96 %）
    python3 host/pim_multitag.py link --free-run --random-data --random-backoff --seed 42 \
  --reader-period 100 --dio-config 4 --reader-rate 3 \
  --tags "0:40:250, 2:42:250, 3:43:250, 4:44:250" --seconds 1800 --tsv

"""
from __future__ import annotations
import argparse, ctypes, math, os, re, select, sys, time
from dataclasses import dataclass

import math
import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import pim
import pim_tsvlog
from dwf_ad import AD, dwf, last_error, DwfStateDone, acqmodeSingle

# ---------------------------------------------------------------------------
#  ★ここを編集すればタグの構成を変えられる（--tags を付けなければこれが使われる）
#     dio      : このタグの CTRL をつなぐ DIO 番号 (0..15)
#     tag_id   : Tag ID   1..255
#     data     : 送るセンサ値 1..255（これが 2 パルスの間隔になる）
#     backoff  : 磁界 ON からこのタグが送信を始めるまでの待ち [TU]
# ---------------------------------------------------------------------------
#   ★実測: AD3 の DigitalOut バッファは 2048 bit/ch（ch 数を増やしても減らない）。
#     sps_tu=2 なら 1024 TU、sps_tu=4 なら 512 TU が波形長の上限。
#     つまり  max(backoff) + 21 + data + 25 + TAIL_TU <= 1024  に収めること。
#   配線(2026-08-18 ユーザ確認): タグ基板は 3 枚、DIO0 / DIO1 / DIO2 に 1 本ずつ。
#   基板上で 2 石のゲートは 1 本の CTRL ネットに繋がっているので、タグ 1 個 = DIO 1 本。
#
#   ★変調深度の測り方: DIO を «静止» L/H にして、検波 DC(CH1) と搬送波 rms(CH2) の差を
#     AD3 のスコープで測る。pim_tune.py が使う MEASURE_AMPLITUDE はこのファームでは
#     更新されないので、スコープで測るのが正解。
#   ★2026-09-22 実測（数値は結合で変わるので «絶対値» でなく «横並びの差» を見ること）:
#         配線し直す前 : DIO0 +0.59 mV / DIO1 +34.5 mV / DIO2 +63.9 mV
#           -> DIO0 だけノイズと同じ = リーダの磁界を一切揺らしていなかった。
#              ピン読み返しは正常（0->3.3V）、基板を 0<->1 で入れ替えたら «両方とも» 生き返った
#              = 故障ではなく «コネクタの接触»（半挿し/挿し間違い）。挿し直しで解消。
#         入れ替え後   : DIO0 +36.5 mV / DIO1 +22.9 mV / DIO2 +44.2 mV（3 本とも変調あり）
#              link 実測: DIO0 単独で frame=137 / 一致=137 / 誤り=0。
#     -> 届かないときは «まず深さを測る»。0.5 mV 級なら電気的に繋がっていない（挿し直す）。
#   ★リーダ側の RX_CONF1 も要注意: bit7=0（AM ch）だと «捕捉列がタグの 2 倍の周波数» に
#     なり、sof は立つのに ID が永久に読めない。必ず bit7=1（PM ch）側を使う。
#     ファーム既定は 0xA0 = PM ch + 2MHz LP。★DS11793 p.9 Table 3 の "Recommended for
#     424/484 kHz subcarrier"（0x8C = 600kHz LP + h200）は «framing 経路» 用の推奨で、
#     Transparent の PIM では使えない: 2026-09-28 の実測（1 タグ/100ms/150 送信を 3 回交互）
#       0xA0 -> 復号 140/146/145、sof/frame 2.00      0x8C -> 4/0/0、sof/frame 3.2-3.8
#     PIM は 1 チップ = 1 TU なので占有帯域が 424kHz±424kHz ≒ 850kHz まで広がり、600kHz
#     では SOF が分裂する。コイルや同調を触ったら --reader-rxconf A0 / 8C / 20 で見比べること。
#     経緯は docs/MEANDER_COIL_DEBUG_LOG.md の 2026-09-22 / 2026-09-28 追記。
DEFAULT_TAGS = [
    dict(pins=[0], tag_id=41, data=250, backoff=500),
    dict(pins=[1], tag_id=42, data=200, backoff=300),
    dict(pins=[2], tag_id=43, data=100, backoff=128),
    dict(pins=[3], tag_id=44, data=100, backoff=128),
    dict(pins=[4], tag_id=45, data=100, backoff=128),
]

# dwfconstants（dwf.h より。dwf_ad.py が持っていない分だけここで定義）
trigsrcNone             = 0
trigsrcPC               = 1
trigsrcDetectorAnalogIn = 2
DwfDigitalOutTypeCustom = 1
DwfDigitalOutIdleLow    = 1
DwfTriggerSlopeRise     = 0
trigtypeEdge            = 0
trigtypePulse           = 1
trigcondRisingPositive  = 0
triglenMore             = 2
filterDecimate          = 0
filterAverage           = 1

# ===========================================================================
#  ★タグが «何 ms ごとに» センサ値を送るか
#
#  PIM のタグは «磁界 ON の立ち上がり» を合図に送信を始める（検波トリガ）。つまり
#  リーダが磁界を OFF->ON する周期 = タグの送信間隔。ここを変えれば間隔が変わる。
#  実行時に上書きしたいときは --reader-period / --reader-gap。
#
#    READER_PERIOD_MS : 送信間隔 [ms]（OFF+ON で 1 周期）。論文の評価条件は 100 ms
#    READER_GAP_MS    : そのうち磁界 OFF の時間 [ms]。ON の長さは period-gap になる
#
#  gap の決め方: 検波出力がトリガ閾値より下まで放電する時間が要る。2026-09-22 実測で
#  磁界 OFF から 0.03 ms で 0.010 V まで落ちるので 20 ms は充分。長くしすぎると ON の
#  時間が減り、backoff の大きい抽選が窓に入らなくなる。
#  ★毎ラウンド乱数を引くとき（--random-data / --random-backoff）は AD3 への波形載せ替えに
#    約 84 ms かかるので、READER_PERIOD_MS は 200 ms 以上にすること（下の警告も参照）。
#  ファーム側の既定は firmware-NFC05A1-424k-ringbuffer の PIM_CYCLE_PERIOD_MS /
#  PIM_CYCLE_GAP_MS。ここで指定した値は起動後にコンソール経由で上書きする。
# ===========================================================================
#  ★--free-run（検波回路なし）の場合
#
#  リーダのファーム（firmware-NFC05A1-424k-ringbuffer）は «磁界 ON 固定 + リングバッファで
#  連続捕捉» ができる。連続捕捉ならタグの送信がどの位相で来ても拾えるので、そもそも
#  «磁界 ON の合図» をタグに伝える必要が無い ＝ 検波回路が要らない。
#  --free-run を付けると:
#      * リーダへ 'cycle off' を送る（磁界は常時 ON。OFF の谷が消えるので GAP も無関係）
#      * タグは AD3 の DigitalOut を READER_PERIOD_MS ごとにホストのタイマで撃つ
#      * Scope CH1（検波）は一切触らない → 1+/1- の配線を外してよい
#  READER_PERIOD_MS の意味は検波版と同じ «タグの送信間隔» のまま変わらない。
# ===========================================================================
READER_PERIOD_MS = 100   # ★送信間隔 [ms]
READER_GAP_MS    = 20    # そのうち磁界 OFF の時間 [ms]

# ---------------------------------------------------------------------------
#  検波トリガのしきい値（2026-09-22 メアンダコイル服での実測）
#    field OFF +0.003 V -> field ON +0.19..0.26 V。段差の半分を level にする。
#    hyst は段差より十分小さければよい（detector モードの推奨は段差/8 = 0.024）。
#    レンジは AD3 が持つ 5 / 50 の 2 段だけ。0.2 V の段差なら 5 の方が分解能が効く。
#  ★コイル・検波回路・配置を変えたら必ず測り直すこと:
#      python3 host/pim_multitag.py detector --port /dev/cu.usbmodem...
#    が推奨値をそのまま出すので、この 3 つを書き換える（--level 等でも上書き可）。
# ---------------------------------------------------------------------------
DETECT_LEVEL_V  = 0.100  # 検波トリガ電圧 [V]
DETECT_HYST_V   = 0.050  # ヒステリシス [V]
DETECT_VRANGE_V = 5.0    # Scope CH1 のレンジ [V, peak-to-peak]

TAIL_TU = 16          # 全タグの送信が終わったあとに足す無音 [TU]
PPM_WARN = 500        # 副搬送波の許容誤差 [ppm]。これを超えたら警告

# 2026-09-22 の配線変更。dwf のチャネル番号は 0 起点なので CH1=0 / CH2=1。
DETECT_CH  = 0        # CH1 = ダイオード検波 + CR 平滑の DC（トリガ源）
CARRIER_CH = 1        # CH2 = アンテナ部の生 13.56 MHz
CARRIER_HZ = 13.56e6  # NFC の搬送波（公称）。実測とのずれを ppm で出すのに使う
# 実測(2026-09-22): 磁界 OFF のとき CH2 は 8 mVpp（±2.5 V レンジのノイズフロア）。
# これより上なら «搬送波あり» とする。下回ったら周波数は読まない（ノイズの最大ビンが
# 出てくるだけで、0.1 MHz などと表示して人を混乱させる）。
CARRIER_MIN_VPP = 0.012


@dataclass
class TagSpec:
    pins: tuple           # このタグの CTRL を出す DIO（複数可。同じ波形を全ピンに出す）
    tag_id: int
    data: int
    backoff_tu: int

    @property
    def label(self) -> str:
        return "+".join(str(p) for p in self.pins)

    def cycle(self, mult: int) -> pim.PimCycle:
        return pim.PimCycle(tag_id=self.tag_id, data=self.data,
                            backoff_tu=self.backoff_tu, data_tu_mult=mult)


def parse_tags(text: str | None) -> list[TagSpec]:
    """--tags "pins:id:data[:backoff]" をカンマ区切りで。pins は + で複数指定できる。
    例: "0+2:42:100:8"  = DIO0 と DIO2 に同じ波形（= 2 石のゲート両方を叩く 1 タグ）
        "0+2:42:100, 4+6:43:200"  = 基板 2 枚 = 2 タグ
    省略時は DEFAULT_TAGS。"""
    if not text:
        return [TagSpec(tuple(t["pins"]), t["tag_id"], t["data"], t["backoff"])
                for t in DEFAULT_TAGS]
    out = []
    for item in text.split(","):
        f = [x.strip() for x in item.strip().split(":") if x.strip() != ""]
        if len(f) not in (3, 4):
            raise SystemExit("bad --tags item %r (want pins:id:data[:backoff])" % item)
        pins = tuple(int(x, 0) for x in f[0].split("+"))
        out.append(TagSpec(pins, int(f[1], 0), int(f[2], 0),
                           int(f[3], 0) if len(f) == 4 else 0))
    return out


def validate(tags: list[TagSpec], mult: int, random_backoff: bool = False) -> None:
    seen = {}
    for t in tags:
        if not t.pins:
            raise SystemExit("tag %d: DIO ピンが指定されていない" % t.tag_id)
        for p in t.pins:
            if not 0 <= p <= 15:
                raise SystemExit("tag id=%d: DIO%d は 0..15 の範囲外" % (t.tag_id, p))
            if p in seen:
                raise SystemExit("DIO%d が tag %d と tag %d で重複" % (p, seen[p], t.tag_id))
            seen[p] = t.tag_id
        if not 0 <= t.tag_id <= 255:
            raise SystemExit("tag_id=%d は 0..255 の範囲外" % t.tag_id)
        if not 1 <= t.data <= 255:
            raise SystemExit("tag %d: data=%d は 1..255 の範囲外" % (t.tag_id, t.data))
        if t.backoff_tu < 0:
            raise SystemExit("tag %d: backoff が負" % t.tag_id)
    # ★backoff が同じタグ同士は «毎回必ず» 衝突する。--tags の 4 つ目（backoff）を
    #   書き忘れると全タグ 0 TU になり、1 パケットも復号できない状態で悩むことになる。
    #   --random-backoff を使うなら毎ラウンド引き直すのでここは無視してよい。
    by_backoff = {}
    for t in tags:
        by_backoff.setdefault(t.backoff_tu, []).append(t.tag_id)
    dup = [v for v in by_backoff.values() if len(v) > 1]
    if dup and len(tags) > 1 and not random_backoff:
        print("  ! backoff が同じタグがあります %s -> 送信が重なって毎回衝突します。"
              % dup)
        print("    --tags \"dio:id:data:backoff\" の 4 つ目を指定するか、"
              "--random-backoff を使ってください。")

    ids = [t.tag_id for t in tags]
    if len(set(ids)) != len(ids):
        raise SystemExit("tag_id が重複している: %r（復号側が preamble/postamble を"
                         "取り違えるので必ず別 ID にすること）" % ids)


# ---------------------------------------------------------------------------
#  波形生成: pim.py の ±1 バッファ -> DIO の 0/1 ビット列
# ---------------------------------------------------------------------------
def pick_sps_tu(fdio_hz: float, want: int | None = None):
    """DIO の分周器は整数なので、sps_tu によって副搬送波の誤差が大きく変わる。
    fDIO=100MHz なら fDIO/FSC=235.99 なので sps_tu=2 or 4 だけが実用（-50ppm）、
    8 や 16 は 1.7% もずれて data が数 TU 狂う。誤差最小の候補を返す。"""
    cands = []
    for sps in (2, 4, 6, 8, 10, 12, 16):
        div = max(1, int(round(fdio_hz / (pim.FSC * sps))))
        fsc = fdio_hz / div / sps
        cands.append((abs((fsc - pim.FSC) / pim.FSC * 1e6), sps, div, fsc))
    cands.sort()
    if want is not None:
        for ppm, sps, div, fsc in cands:
            if sps == want:
                return sps, div, fsc, ppm
        raise SystemExit("sps_tu=%d は候補外（2,4,6,8,10,12,16 のいずれか）" % want)
    ppm, sps, div, fsc = cands[0]
    return sps, div, fsc, ppm


def build_bits(tags: list[TagSpec], sps_tu: int, mult: int, cap_tu: int | None = None):
    """全タグ分のビット列を「同じ長さ」で作る。DigitalOut は全ch同時スタートなので、
    タグごとのバックオフはバッファ先頭のアイドル長として表現される。

    cap_tu を渡すとバッファ長をそこで打ち切る。ランダム backoff（論文の 2^14 TU）だと
    最大 backoff + パケットがバッファ上限を超えうるため、**入りきらないタグは
    «送信させない»**（途中で切ると偽の SOF を作ってしまうので、まるごと落とす）。
    どのみちリーダの捕捉窓より遥か後ろなので観測結果は変わらない。
    戻り値の dropped に落としたタグを入れて呼び出し側へ知らせる。"""
    need_tu = max(int(math.ceil(t.cycle(mult).postamble_end_tu)) for t in tags) + TAIL_TU
    total_tu = need_tu if cap_tu is None else min(need_tu, int(cap_tu))
    n = total_tu * sps_tu
    out, dropped = {}, []
    for t in tags:
        wave = pim.build_switch_waveform(t.cycle(mult), sps_tu, idle_tu_after=0)
        bits = np.zeros(n, dtype=np.uint8)
        if len(wave) <= n:
            bits[:len(wave)] = (wave > 0).astype(np.uint8)
        else:
            dropped.append(t.tag_id)        # バッファに収まらない = 窓外なので送らない
        for p in t.pins:                    # 同じ波形を全ピンへ（2 石のゲート両方）
            out[p] = bits
    return out, total_tu, n, dropped


# ---------------------------------------------------------------------------
#  AD3 セットアップ
# ---------------------------------------------------------------------------
def dio_internal_clock(h) -> float:
    hz = ctypes.c_double()
    dwf.FDwfDigitalOutInternalClockInfo(h, ctypes.byref(hz))
    return hz.value


def dio_buffer_bits(h, ch: int) -> int:
    n = ctypes.c_uint()
    dwf.FDwfDigitalOutDataInfo(h, ctypes.c_int(ch), ctypes.byref(n))
    return n.value


def program_digitalout(h, bitmap, div: int, rate: float, trigsrc: int, sps_tu: int = 2,
                       repeat: int = 0, wait_s: float = 0.0):
    """各 DIO に custom パターンを載せ、1 トリガ = 1 回再生 で武装する。

    ★repeat は「wait-run を何周するか」であって「1 周で何回撃つか」ではない。
      - repeat=1: 1 周で Done に落ち、以降のトリガを無視する。**ホストが毎回
        FDwfDigitalOutConfigure(h,1) で再武装すれば** 1 トリガ = 1 送信になる。
        検波トリガ用はこれ（run / link）。
      - repeat=0: 無限に周回する。RepeatTrigger が期待どおり 1 周 1 トリガに
        ゲートしてくれないので、1 回のトリガで数 ms の連射になることがある
        （2026-08-20 実測: 1 トリガで 0.34ms の 1 サイクル → 1.6-5.3ms 後に
         2.4-3.8ms の連射。捕捉窓に 2 回入って sof=4 / tags=2 になっていた）。
        トリガを使わないフリーラン（verify）専用。
    trigsrc=trigsrcNone なら周回にトリガが挟まらないので、そのまま連続再生になる
    （= フリーラン。verify モードがこれを使う）。"""
    dwf.FDwfDigitalOutReset(h)
    nbits = len(next(iter(bitmap.values())))
    for ch, bits in sorted(bitmap.items()):
        cap = dio_buffer_bits(h, ch)
        if cap and nbits > cap:
            raise SystemExit(
                "DIO%d: 波形 %d bit がバッファ上限 %d bit を超過。\n"
                "  上限 = %d TU（sps_tu で割った値）。max(backoff)+21+data+25+%d を\n"
                "  これ以下にすること。対策: backoff を詰める / --sps-tu 2 にする /\n"
                "  FDwfDigitalOutPlayDataSet によるストリーミングへ移行する。"
                % (ch, nbits, cap, cap // sps_tu, TAIL_TU))
        packed = np.packbits(bits, bitorder="little")     # dwf.h: bits order is lsb first
        buf = (ctypes.c_ubyte * len(packed))(*packed.tolist())
        c = ctypes.c_int(ch)
        dwf.FDwfDigitalOutEnableSet (h, c, ctypes.c_int(1))
        dwf.FDwfDigitalOutTypeSet   (h, c, ctypes.c_int(DwfDigitalOutTypeCustom))
        dwf.FDwfDigitalOutIdleSet   (h, c, ctypes.c_int(DwfDigitalOutIdleLow))  # 停止時 Low
        dwf.FDwfDigitalOutDividerSet(h, c, ctypes.c_uint(div))
        dwf.FDwfDigitalOutDataSet   (h, c, buf, ctypes.c_uint(nbits))
    run_s = nbits / rate
    dwf.FDwfDigitalOutRunSet          (h, ctypes.c_double(run_s))   # 1 トリガ = 1 サイクル
    # ★wait_s: トリガしてから «実際に撃つまで» の待ち時間。検波は磁界 OFF の過渡でも
    #   1 回トリガしてしまい、そこで撃つと gap の間ずっと無磁界に送って丸損になる
    #   （2026-09-22 実測: backoff < gap のタグは半分しか届かない）。wait に gap を
    #   入れておくと、OFF で撃っても «磁界が戻る瞬間» に着地するので無駄撃ちが消える。
    #   ON でトリガした分は全タグ一様に gap だけ後ろへずれるだけ（相対関係は不変 =
    #   衝突の幾何も不変）。ON の長さは gap + 最大 backoff + 復号時間 より長く取ること。
    dwf.FDwfDigitalOutWaitSet         (h, ctypes.c_double(wait_s))
    dwf.FDwfDigitalOutRepeatSet       (h, ctypes.c_uint(repeat))    # 0 = 無限に周回
    dwf.FDwfDigitalOutRepeatTriggerSet(h, ctypes.c_int(0 if trigsrc == trigsrcNone else 1))
    dwf.FDwfDigitalOutTriggerSourceSet(h, ctypes.c_ubyte(trigsrc))
    dwf.FDwfDigitalOutTriggerSlopeSet (h, ctypes.c_int(DwfTriggerSlopeRise))
    dwf.FDwfDigitalOutConfigure       (h, ctypes.c_int(1))
    return run_s


class FastTx:
    """free-run 用の高速送信エンジン（docs/PLAN_TX_PERIOD.md の A+B+C 案、2026-09-28 実機検証済）。

    program_digitalout() との違い:
      A. backoff を «バッファ先頭の 0» で表さず、チャネルごとのカウンタ初期値で表す。
         Custom 型は CounterSet(0,0) でループ長 = バッファ全長になり、そのとき
         CounterInit(0, v) = «先頭 v サンプルの遅延» になる（実測 ±1 サンプル）。
         ※DividerInit は Custom 型では効かない（実測）。
         → 毎ラウンド載せるのはパケットだけ（数百 bit）。
      B. AutoConfigure=0。既定の 1 だと Set のたびに USB 転送が走り、これが載せ替え
         84 ms の主犯だった。0 にすると Configure(1) の 1 回にまとまる。
      C. 載せたら PC トリガ待ちで武装しておき、周期の区切りで fire() する。
         載せ替え時間のばらつきが発射時刻に乗らない。
      実測（cfg 4, 4 タグ）: 載せ替え 平均 4.8 ms / 最大 5.6 ms（旧方式 89 ms）。

    ★パケットは毎回 «最大パケット長» まで 0 で埋めて上書きする。ループ長 = バッファ全長
      なので DataSet した長さの外も再生される。短いパケットをそのまま載せると、前の
      ラウンドの長いパケット（data 大）の残骸が出る（実測 1 ラウンド ~16 bit 化けた）。
      同じ理由で open 時に全バッファを 0 で埋めておく（旧方式の波形が残っていても消える）。
    ★有効範囲は 遅延 + パケット <= バッファ全長（超えると先頭へ回り込む）。
      入らないタグはこの回 «無送信» にする。
    ★ラウンドの起点を «そのラウンドで最も早いタグの backoff» まで詰めて再生する
      （2026-10-17）。遅延 = backoff - shift_tu。リーダは固定の受信窓を持たず、衝突の
      幾何はタグ間の相対時刻だけで決まるので、結果は変わらない。必要なバッファ長が
      «backoff_max + パケット» から «最早タグ〜最遅タグの終端» に縮み、無送信は
      抽選の約 1.15 % → 0.03 % 以下（N<=5）になる。TSV の時刻は詰める前（シミュレータ
      と同じ起点）のまま書き、FireLog は shift_tu を引いて実際の発射時刻を求める。
    ★AutoConfigure=0 はデバイス全体に効くので、close() で必ず 1 に戻す。"""

    def __init__(self, h, pins, div: int, rate: float, sps_tu: int, mult: int):
        self.h, self.div, self.rate, self.sps, self.mult = h, div, rate, sps_tu, mult
        self.pins = sorted(set(pins))
        self.cap = dio_buffer_bits(h, self.pins[0])      # bit = サンプル
        worst = pim.PimCycle(tag_id=0, data=255, backoff_tu=0, data_tu_mult=mult)
        self.pkt_bits = (int(math.ceil(worst.postamble_end_tu)) + TAIL_TU) * sps_tu
        self.run_s = 0.0
        self.shift_tu = 0          # 直近の load() で詰めた起点 [TU]
        self.fired_at = 0.0
        self.armed = False

        dwf.FDwfDigitalOutReset(h)
        dwf.FDwfDeviceAutoConfigureSet(h, ctypes.c_int(0))
        zeros = self._buf(np.zeros(self.cap, dtype=np.uint8))
        for p in self.pins:
            c = ctypes.c_int(p)
            dwf.FDwfDigitalOutEnableSet (h, c, ctypes.c_int(1))
            dwf.FDwfDigitalOutTypeSet   (h, c, ctypes.c_int(DwfDigitalOutTypeCustom))
            dwf.FDwfDigitalOutIdleSet   (h, c, ctypes.c_int(DwfDigitalOutIdleLow))
            dwf.FDwfDigitalOutDividerSet(h, c, ctypes.c_uint(div))
            dwf.FDwfDigitalOutDataSet   (h, c, zeros, ctypes.c_uint(self.cap))
        dwf.FDwfDigitalOutWaitSet         (h, ctypes.c_double(0.0))
        dwf.FDwfDigitalOutRepeatSet       (h, ctypes.c_uint(1))
        dwf.FDwfDigitalOutRepeatTriggerSet(h, ctypes.c_int(1))
        dwf.FDwfDigitalOutTriggerSourceSet(h, ctypes.c_ubyte(trigsrcPC))
        dwf.FDwfDigitalOutTriggerSlopeSet (h, ctypes.c_int(DwfTriggerSlopeRise))

    @staticmethod
    def _buf(bits):
        packed = np.packbits(bits, bitorder="little")   # dwf.h: bits order is lsb first
        return (ctypes.c_ubyte * len(packed))(*packed.tolist())

    def load(self, tags: list[TagSpec]) -> list[int]:
        """1 ラウンド分を載せて PC トリガ待ちで武装する。戻り値 = 無送信にした tag_id。"""
        h, dropped, end = self.h, [], 0
        self.shift_tu = min((t.backoff_tu for t in tags), default=0)
        for t in tags:
            pkt = np.zeros(self.pkt_bits, dtype=np.uint8)
            cyc = TagSpec(t.pins, t.tag_id, t.data, 0).cycle(self.mult)   # backoff 抜き
            wave = (pim.build_switch_waveform(cyc, self.sps, idle_tu_after=0) > 0)
            delay = (t.backoff_tu - self.shift_tu) * self.sps
            if delay + len(wave) + TAIL_TU * self.sps > self.cap:
                dropped.append(t.tag_id)        # 入らない = 窓外。途中で切らず丸ごと落とす
                delay = 0                       # pkt は全 0 のまま = 何も出さない
            else:
                pkt[:len(wave)] = wave
                end = max(end, delay + self.pkt_bits)
            buf = self._buf(pkt)
            for p in t.pins:
                c = ctypes.c_int(p)
                dwf.FDwfDigitalOutDataSet(h, c, buf, ctypes.c_uint(self.pkt_bits))
                # DataSet がループ長をデータ長に戻すので、毎回その後で全長モードにする
                dwf.FDwfDigitalOutCounterSet(h, c, ctypes.c_uint(0), ctypes.c_uint(0))
                dwf.FDwfDigitalOutCounterInitSet(h, c, ctypes.c_int(0), ctypes.c_uint(delay))
        self.run_s = min(max(end, self.pkt_bits), self.cap) / self.rate
        dwf.FDwfDigitalOutRunSet(h, ctypes.c_double(self.run_s))
        dwf.FDwfDigitalOutConfigure(h, ctypes.c_int(1))   # ここで全設定を 1 回で送る
        self.armed = True
        return dropped

    def rearm(self):
        """同じ内容をもう一度撃てるように武装し直す（data/backoff 固定のとき）。"""
        dwf.FDwfDigitalOutConfigure(self.h, ctypes.c_int(1))
        self.armed = True

    def fire(self):
        dwf.FDwfDeviceTriggerPC(self.h)
        self.fired_at = time.monotonic()
        self.armed = False

    def busy(self) -> bool:
        """直前の発射がまだ再生中か。再生中に DigitalOut を触ると波形が切れる。"""
        if not self.fired_at:
            return False
        if time.monotonic() < self.fired_at + self.run_s:
            return True
        st = ctypes.c_ubyte()
        dwf.FDwfDigitalOutStatus(self.h, ctypes.byref(st))
        return st.value != DwfStateDone

    def close(self):
        dwf.FDwfDigitalOutReset(self.h)
        dwf.FDwfDigitalOutConfigure(self.h, ctypes.c_int(0))
        dwf.FDwfDeviceAutoConfigureSet(self.h, ctypes.c_int(1))


def arm_detector(h, level: float, vrange: float, hyst: float, fs: float, nbuf: int,
                 min_width: float = 0.0, holdoff: float = 0.0):
    """Scope ch0（検波出力）の立ち上がりを DigitalOut のトリガ源にする。

    AutoTimeout=0 が肝: これを 0 にしないとスコープが自分で勝手にトリガして、
    リーダが磁界を出していなくてもタグが送信してしまう。

    ★min_width > 0 のとき「エッジ」ではなく「パルス幅 >= min_width」で判定する。
      磁界 ON/OFF の過渡スパイク（幅 1 us 未満・振幅 0.1-0.27 V）が、狙っている
      包絡線の段差（23 mV）の 10 倍あって素のエッジでは区別できないため。
      本物の field ON は最低でも数百 us 続くので、幅で捨てるのが唯一の分離軸。

    ★filterAverage が第 2 の防波堤: 既定の filterDecimate は 125 サンプルに 1 個を
      残して残りを捨てるので、細いスパイクがそのまま通ってしまう。average なら
      間引き窓（1/fs 秒）で平均され、幅 100 ns のスパイクは 1/10 以下に潰れる。
      保存バッファ側にも掛けるので、表示される mean/max も実態を映すようになる。

    ★holdoff > 0 で、トリガ後その時間だけ次のトリガを禁止する。実測(2026-08-18)では
      磁界 ON の 1 回につきトリガが 30 ms 間隔で 2 発（時々 3 発）出る。2 発目は
      リーダの捕捉窓 13.1 ms の外なので無駄撃ちになる。auto の実周期 1.11 s より
      十分短く、ペア間隔 30 ms より十分長い値（50-500 ms）を渡して 1 発に潰す。
      ただし「1 発目と 2 発目のどちらが本物の磁界 ON か」はリーダの sof で確認すること。"""
    dwf.FDwfAnalogInReset(h)
    dwf.FDwfAnalogInChannelEnableSet(h, ctypes.c_int(DETECT_CH), ctypes.c_int(1))
    # ★CH2 には生の 13.56 MHz が来ている（2026-09-22 の配線変更）。トリガ源は CH1 だけ
    #   なので明示的に切る。--det-fs 1 MHz では 13.56 MHz は折り返して «直流に見える
    #   偽の段差» になりかねないし、有効なままだと取得バッファを無駄に食う。
    dwf.FDwfAnalogInChannelEnableSet(h, ctypes.c_int(CARRIER_CH), ctypes.c_int(0))
    dwf.FDwfAnalogInChannelRangeSet (h, ctypes.c_int(0), ctypes.c_double(vrange))
    dwf.FDwfAnalogInChannelOffsetSet(h, ctypes.c_int(0), ctypes.c_double(0.0))
    dwf.FDwfAnalogInChannelFilterSet(h, ctypes.c_int(0), ctypes.c_int(filterAverage))
    dwf.FDwfAnalogInFrequencySet    (h, ctypes.c_double(fs))
    dwf.FDwfAnalogInBufferSizeSet   (h, ctypes.c_int(nbuf))
    dwf.FDwfAnalogInAcquisitionModeSet(h, acqmodeSingle)
    dwf.FDwfAnalogInTriggerAutoTimeoutSet(h, ctypes.c_double(0.0))   # ★自動トリガ禁止
    dwf.FDwfAnalogInTriggerSourceSet   (h, ctypes.c_ubyte(trigsrcDetectorAnalogIn))
    dwf.FDwfAnalogInTriggerChannelSet  (h, ctypes.c_int(0))
    dwf.FDwfAnalogInTriggerConditionSet(h, ctypes.c_int(trigcondRisingPositive))
    dwf.FDwfAnalogInTriggerLevelSet    (h, ctypes.c_double(level))
    dwf.FDwfAnalogInTriggerHysteresisSet(h, ctypes.c_double(hyst))
    dwf.FDwfAnalogInTriggerHoldOffSet  (h, ctypes.c_double(holdoff))
    dwf.FDwfAnalogInTriggerFilterSet   (h, ctypes.c_int(filterAverage))
    if min_width > 0.0:
        dwf.FDwfAnalogInTriggerTypeSet           (h, ctypes.c_int(trigtypePulse))
        dwf.FDwfAnalogInTriggerLengthConditionSet(h, ctypes.c_int(triglenMore))
        dwf.FDwfAnalogInTriggerLengthSet         (h, ctypes.c_double(min_width))
    else:
        dwf.FDwfAnalogInTriggerTypeSet(h, ctypes.c_int(trigtypeEdge))
    dwf.FDwfAnalogInTriggerPositionSet (h, ctypes.c_double(0.0))
    dwf.FDwfAnalogInConfigure(h, ctypes.c_int(1), ctypes.c_int(1))  # reconfigure=1 = 初回だけ
    # ★レンジを変えた直後はアナログ段がまだ前の設定のまま（上の _AI_SETTLE_S の注参照）。
    #   トリガ閾値はその利得で判定されるので、落ち着く前に武装すると «電圧が 11 倍
    #   ずれた状態で» トリガしてしまう。ここは実行あたり 1 回なので素直に待つ。
    time.sleep(_AI_SETTLE_S)
    global _ai_last_cfg
    _ai_last_cfg = None              # 次の single_shot に «設定が変わった» と伝える
    return _wait_armed(h)


def _wait_armed(h) -> bool:
    """Configure 直後は「前回の Done」が残っていることがある。Armed に移るまで待たないと
    再アームするたびに Done を読んで空トリガを数え、Python のループ速度(~200/s)で
    カウントが増え続ける。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.5:
        if not analogin_done(h):
            return True
        time.sleep(0.0005)
    return False


def rearm_detector(h) -> bool:
    """2 回目以降の再アーム。★入力段に触らず、取り込みだけを再スタートする。

    トリガのたびに arm_detector() を呼ぶと FDwfAnalogInReset + レンジ再設定で
    アナログ入力段が毎回作り直され、AD3 が「カチ」と鳴る。
    2026-08-20 実機切り分け:
      - DIO のリード線を全部外しても、搬送波を受けるたびに鳴る -> 出力側は無関係
      - WaveForms でレンジを変えるだけでも鳴る                -> 音源は入力段の再設定
    reconfigure=0 なら設定はデバイス内に残ったままなので入力段は一切動かず無音。
    USB 往復も減るので再アームが速くなる。★1 サイクルに 2 トリガ出る（磁界 OFF の
    過渡スパイクと ON の立ち上がりで 1 発ずつ、19-21 ms 間隔 = 谷の長さ）ので、
    再アームが遅いと本命の ON 側を取り逃す。PLAN.md 実測: nbuf=16384(16.4 ms) では
    再武装が間に合わず 1 サイクル 1 トリガしか拾えない。
    ★設定値（level/vrange/holdoff 等）を変えたいときだけ arm_detector() を呼び直すこと。"""
    dwf.FDwfAnalogInConfigure(h, ctypes.c_int(0), ctypes.c_int(1))   # reconfig=0, start=1
    return _wait_armed(h)


def pick_range(h, need_peak: float) -> float:
    """実機の AnalogIn が持つレンジ候補から、±need_peak が収まる最小のものを返す。
    dwf のレンジは peak-to-peak 表記なので、収容できる片側振幅は range/2。
    AD3 は 5 V と 50 V の 2 段しかない（それ以外を渡しても近い方に丸められる）。"""
    lo = ctypes.c_double(); hi = ctypes.c_double(); steps = ctypes.c_double()
    dwf.FDwfAnalogInChannelRangeInfo(h, ctypes.byref(lo), ctypes.byref(hi),
                                     ctypes.byref(steps))
    n = max(int(steps.value), 1)
    if n == 1:
        cands = [hi.value]
    else:   # 対数等間隔で n 段
        cands = [lo.value * (hi.value / lo.value) ** (i / (n - 1)) for i in range(n)]
    for c in cands:
        if c / 2.0 >= need_peak:
            return round(c, 3)
    return round(cands[-1], 3)


def analogin_done(h) -> bool:
    sts = ctypes.c_byte()
    dwf.FDwfAnalogInStatus(h, ctypes.c_int(1), ctypes.byref(sts))
    return sts.value == DwfStateDone


def read_ch(h, ch: int, nbuf: int):
    buf = (ctypes.c_double * nbuf)()
    dwf.FDwfAnalogInStatusData(h, ctypes.c_int(ch), buf, ctypes.c_int(nbuf))
    return np.array(buf, dtype=float)


def read_ch0(h, nbuf: int):
    """検波出力（CH1 = dwf の ch0）。トリガ経路はこのチャネルしか見ない。"""
    return read_ch(h, DETECT_CH, nbuf)


# ---------------------------------------------------------------------------
#  CH2（アンテナの生 13.56 MHz）の実測
#    2026-09-22 の配線変更で CH2 に搬送波そのものが来た。これで「リーダが今どれだけ
#    出しているか」を、検波器の特性を介さずに直接測れる。
# ---------------------------------------------------------------------------
# ★AD3 の入力レンジを切り替えた直後は «前のレンジのままのアナログ段» を «新しい
#   レンジの目盛り» で読んでしまう。2026-09-22 実測: 同じ DC を range 5 -> 0.1916 V、
#   直後に range 50 -> 2.1951 V（比 11.47 = 実レンジ 60.9/5.3 そのもの）。0.3 s 待つと
#   両方 0.19-0.22 V で一致した。つまり «待たないと 11 倍間違える»。
#   dwf_ad.py の scope_capture() が Configure 前に 0.1 s 寝ているのはこのため。
_AI_SETTLE_S = 0.25          # レンジ/チャネルを変えたあとアナログ段が落ち着くまで
_ai_last_cfg = None          # 直前に «実際に» 出した設定（同じなら待たない）


def single_shot(h, ch: int, fs: float, nbuf: int, vrange: float,
                atten: float = 1.0, filt: int = filterDecimate):
    """1 チャネルをトリガ無しで単発取得して (サンプル列, 実 fs) を返す。

    filt の既定を decimate にしているのは搬送波を測るため。filterAverage は間引き窓
    (1/fs) で平均する = ローパスなので、13.56 MHz を測ると振幅が小さく出る。逆に
    検波出力の DC を測るときは average の方がノイズに強い（measure_dc がそう呼ぶ）。
    vrange は «入力換算» の peak-to-peak（atten を掛けた値をそのまま渡す）。

    ★設定が前回と変わったときは _AI_SETTLE_S 待ってから 1 発捨てる（上のコメント）。
      同じ設定で連続して呼ぶ分にはこの待ちは入らない。"""
    global _ai_last_cfg
    cfg = (ch, fs, nbuf, vrange, atten, filt)
    c = ctypes.c_int(ch)
    dwf.FDwfAnalogInReset(h)
    dwf.FDwfAnalogInChannelEnableSet     (h, c, ctypes.c_int(1))
    dwf.FDwfAnalogInChannelAttenuationSet(h, c, ctypes.c_double(atten))
    dwf.FDwfAnalogInChannelRangeSet      (h, c, ctypes.c_double(vrange))
    dwf.FDwfAnalogInChannelOffsetSet     (h, c, ctypes.c_double(0.0))
    dwf.FDwfAnalogInChannelFilterSet     (h, c, ctypes.c_int(filt))
    dwf.FDwfAnalogInFrequencySet         (h, ctypes.c_double(fs))
    dwf.FDwfAnalogInBufferSizeSet        (h, ctypes.c_int(nbuf))
    dwf.FDwfAnalogInAcquisitionModeSet   (h, acqmodeSingle)
    dwf.FDwfAnalogInTriggerSourceSet     (h, ctypes.c_ubyte(trigsrcNone))
    got = ctypes.c_double(); dwf.FDwfAnalogInFrequencyGet(h, ctypes.byref(got))

    def take(reconfigure: int):
        dwf.FDwfAnalogInConfigure(h, ctypes.c_int(reconfigure), ctypes.c_int(1))
        t0 = time.monotonic()
        while not analogin_done(h):
            if time.monotonic() - t0 > 2.0:
                raise RuntimeError("AnalogIn が Done にならない: %s" % last_error())
            time.sleep(0.0005)
        return read_ch(h, ch, nbuf)

    if cfg != _ai_last_cfg:
        take(1)                      # アナログ段を新しい設定にしてから
        time.sleep(_AI_SETTLE_S)     # 落ち着くのを待ち
        _ai_last_cfg = cfg
        return take(0), got.value    # 改めて測る（1 発目は捨てる）
    return take(1), got.value


def dominant_freq(x, fs: float) -> float:
    """一番強い成分の周波数 [Hz]。FFT のビン幅より細かく読むため放物線補間する
    （nbuf=8192 / fs=50 MHz ならビン幅 6.1 kHz = 13.56 MHz に対して 450 ppm。
    補間しないと «搬送波が公称からどれだけずれているか» を論じられない）。"""
    n = len(x)
    if n < 16:
        return float("nan")
    sp = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(n)))
    sp[0] = 0.0
    k = int(np.argmax(sp))
    if 0 < k < len(sp) - 1:
        a, b, c = float(sp[k - 1]), float(sp[k]), float(sp[k + 1])
        d = a - 2.0 * b + c
        if d != 0.0:
            k = k + 0.5 * (a - c) / d
    return k * fs / n


def measure_carrier(h, ch: int = CARRIER_CH, fs: float = 50e6, nbuf: int = 8192,
                    atten: float = 1.0, vrange: float | None = None) -> dict:
    """CH2 の生 13.56 MHz を測って振幅・実効値・周波数を返す。

    ★レンジは自動。AD3 が持つのは 5 V / 50 V の 2 段だけ（peak-to-peak 表記なので
      収容できる片振幅はその半分）。アンテナ端の電圧は整合回路の Q で電源電圧より
      大きくなりうるので、まず広い方で測り、収まる最小のレンジで測り直す。
    ★AD3 のアナログ入力は ±25 V が限界。それを超えるなら分圧か ×10 プローブを入れ、
      その比を atten に渡すこと（渡さないと表示が実電圧にならない）。"""
    def acquire(vr_dev):
        v, fs_act = single_shot(h, ch, fs, nbuf, vr_dev * atten, atten, filterDecimate)
        return v, fs_act, vr_dev
    v, fs_act, vr_dev = acquire(50.0 if vrange is None else vrange / atten)
    if vrange is None:
        peak = max(abs(float(v.min())), abs(float(v.max()))) / atten
        vr2 = pick_range(h, peak * 1.3)
        if vr2 < vr_dev:                       # 過大レンジは分解能を捨てているだけ
            v, fs_act, vr_dev = acquire(vr2)
    dc   = float(v.mean())
    ac   = v - dc
    vpp  = float(v.max() - v.min())
    peak = max(abs(float(v.min())), abs(float(v.max())))
    full    = vr_dev * atten / 2.0             # このレンジで入る片振幅
    present = vpp > CARRIER_MIN_VPP * atten
    freq    = dominant_freq(ac, fs_act) if present else float("nan")
    return dict(v=v, fs=fs_act, vrange=vr_dev * atten, dc=dc, vpp=vpp,
                amp=vpp / 2.0, vrms=float(np.sqrt(np.mean(ac * ac))), peak=peak,
                clipped=(peak >= 0.98 * full), full=full, freq=freq, present=present)


def measure_carrier_best(h, a, timeout: float = 1.5) -> dict:
    """搬送波が «出ている瞬間» を捕まえるまで測り直して、一番大きかったものを返す。

    リーダが磁界サイクル（cycle on）を回しているときは OFF の谷が周期の大半を占める
    ことがある（実測: period 200 ms / ON は 25 %）。1 発だけ測ると谷に当たって
    「搬送波なし」と誤読するので、見つかるまで（最大 timeout 秒）粘る。"""
    best = None
    t0 = time.monotonic()
    while True:
        car = measure_carrier(h, a.carrier_ch, a.carrier_fs, a.carrier_nbuf,
                              a.carrier_atten)
        if best is None or car["vpp"] > best["vpp"]:
            best = car
        if best["present"] or time.monotonic() - t0 > timeout:
            return best


def measure_dc(h, ch: int = DETECT_CH, vrange: float = 50.0, fs: float = 1e6,
               nbuf: int = 4096) -> dict:
    """CH1（検波出力）の DC レベルを 1 回だけ測る。carrier モードの表示用。
    段差の統計を取りたいときは detector モード（こちらは «今いくらか» だけ）。"""
    v, fs_act = single_shot(h, ch, fs, nbuf, vrange, 1.0, filterAverage)
    return dict(v=v, fs=fs_act, mean=float(v.mean()),
                min=float(v.min()), max=float(v.max()))


# ---------------------------------------------------------------------------
#  リーダ側（ST25R3911B）の出力電圧
#    RFO が振る電圧 = VDD_RF。bring-up の ADJUST_REGULATORS(0xD6) が自動で選び、
#    結果は REGULATOR_RESULT(0x2B) の reg_d[7:4] に出る。手で決めるなら
#    REGULATOR_CONTROL(0x2A) の reg_s(bit7)=1 + rege[6:3]。
# ---------------------------------------------------------------------------
REG_IO_CONF2          = 0x01   # bit7 = sup3V（電源モード）
REG_REGULATOR_CONTROL = 0x2A   # bit7 = reg_s, [6:3] = rege
REG_REGULATOR_RESULT  = 0x2B   # [7:4] = reg_d（ADJUST_REGULATORS の結果）
REG_IC_IDENTITY       = 0x3F


def vddrf_mv(reg4: int, sup3v: bool) -> int:
    """reg_d / rege の 4 bit 値 -> VDD_RF [mV]。RFAL(st25r39xx) と同じ換算式。
    5 V 電源: 3600 + 100*reg / 3.3 V 電源: 2400 + 100*(reg-5)（0..4 は未使用）。"""
    if sup3v:
        return 2400 + 100 * (reg4 - 5 if reg4 > 4 else 0)
    return 3600 + 100 * reg4


def reader_regulator(rd) -> dict | None:
    """リーダのコンソール越しにレジスタを読み、今の VDD_RF を返す。読めなければ None。

    ★番地は ST25R3911B(NFC05A1)のもの。3916B(NFC08A1)版ファームでは
      REGULATOR_RESULT が Space-B にあり、コンソールの 'r'（Space-A のみ）では
      読めないので、IC_IDENTITY を見て 3911B 以外なら諦める。
    ★'r' はファームが transparent を抜けて SPI ピンを戻す = 捕捉が止まる。
      carrier モードは先に 'listen off' しているので影響しない。"""
    def rd_reg(reg):
        m = re.search(r"R 0x%02X=0x([0-9A-Fa-f]{2})" % reg, rd.cmd("r %02x" % reg, 0.7))
        return int(m.group(1), 16) if m else None
    ident = rd_reg(REG_IC_IDENTITY)
    if ident is None:
        return None
    if (ident & 0xF8) != 0x08:
        return dict(unsupported=True, ident=ident)
    io2 = rd_reg(REG_IO_CONF2)
    res = rd_reg(REG_REGULATOR_RESULT)
    ctl = rd_reg(REG_REGULATOR_CONTROL)
    if io2 is None or res is None or ctl is None:
        return None
    sup3v = bool(io2 & 0x80)
    reg_d = (res >> 4) & 0x0F
    rege  = (ctl >> 3) & 0x0F
    manual = bool(ctl & 0x80)
    active = rege if manual else reg_d
    return dict(unsupported=False, ident=ident, sup3v=sup3v, reg_d=reg_d, rege=rege,
                manual=manual, vddrf_mv=vddrf_mv(active, sup3v))


# ---------------------------------------------------------------------------
#  レポート
# ---------------------------------------------------------------------------
def report_plan(tags, sps_tu, div, fsc_act, ppm, total_tu, nbits, rate, mult, cap=0,
                reader_fs=40e6):
    tu_us = 1e6 / fsc_act
    print("---- DIO の設定 ----")
    print("  sps_tu       : %d sample/TU" % sps_tu)
    print("  divider      : %d" % div)
    print("  DIO rate     : %.3f MHz" % (rate / 1e6))
    print("  subcarrier   : %.1f Hz (nominal %.1f, err %+0.0f ppm)%s"
          % (fsc_act, pim.FSC, ppm, "  <-- 要注意" if abs(ppm) > PPM_WARN else ""))
    print("  TU           : %.4f us" % tu_us)
    print("  pattern      : %d TU = %d bit/ch = %.3f ms" % (total_tu, nbits, nbits / rate * 1e3))
    if cap:
        print("  buffer usage : %d / %d bit  (%.0f%%)   上限 %d TU"
              % (nbits, cap, 100.0 * nbits / cap, cap // sps_tu))
    if abs(ppm) > PPM_WARN:
        print("  !! 副搬送波の誤差が大きい。--sps-tu 2 か 4 を試すこと。")
    # ★リーダの捕捉レートは rate 0..3 = 40/20/10/5 Msps（公称）。ここを 40 MHz 固定で
    #   書いていたため、rate 3 で運用しているのに «sps 94400» と案内してしまい、
    #   8 で割った 11850 を使う→ID 復号率が 1/5 になる、という事故の元になっていた
    #   （2026-09-25）。レートに合わせて出す。
    print("  -> リーダのコンソールで  sps %d  と打つと、この誤差を完全に補正できる"
          % int(round(reader_fs / fsc_act * 1000)))
    print("     （既定の 'sps auto' は公称 %.1f Hz を前提にした値 %d を使う。捕捉 %.0f Msps）"
          % (pim.FSC, int(round(reader_fs / pim.FSC * 1000)), reader_fs / 1e6))
    print()
    print("---- タグ構成 ----")
    print("   DIO      tag_id   data   backoff     preamble@     postamble@       終端")
    for t in sorted(tags, key=lambda x: x.pins):
        c = t.cycle(mult)
        print("   %-7s  %6d   %4d   %7d TU   %7.0f TU   %8.0f TU   %8.0f TU"
              % (t.label, t.tag_id, t.data, t.backoff_tu,
                 c.preamble_start_tu, c.postamble_start_tu, c.postamble_end_tu))
    if any(len(t.pins) > 1 for t in tags):
        print("   ※ 1 タグに複数ピンを割り当てた場合、そのピンには同じ波形が出る")
        print("      （基板上でゲートが分かれている構成向け）。")
    print()


# ---------------------------------------------------------------------------
#  モード
# ---------------------------------------------------------------------------
def mode_info(a):
    ad = AD()
    h = ad.h
    try:
        fdio = dio_internal_clock(h)
        print("DigitalOut internal clock : %.3f MHz" % (fdio / 1e6))
        n = ctypes.c_int(); dwf.FDwfDigitalOutCount(h, ctypes.byref(n))
        print("DigitalOut channels       : %d" % n.value)
        print("DigitalOut buffer         : %d bit/ch (DIO0)" % dio_buffer_bits(h, 0))
        print()
        print("sps_tu ごとの副搬送波誤差（分周器が整数なので選択が効く）:")
        for sps in (2, 4, 6, 8, 12, 16):
            _, div, fsc, ppm = pick_sps_tu(fdio, sps)
            print("   sps_tu=%2d  div=%4d  FSC=%9.1f Hz  err=%+8.0f ppm  %s"
                  % (sps, div, fsc, ppm, "OK" if abs(ppm) <= PPM_WARN else "<-- 使うな"))
        best = pick_sps_tu(fdio)
        print("\n推奨: --sps-tu %d" % best[0])
    finally:
        ad.close()


def mode_detector(a):
    """検波出力を眺めて field OFF / ON のレベルと推奨トリガ閾値を出す。
    リーダ側で 'auto on' を回しながら実行すること（磁界が ON/OFF するので二山になる）。

    ★--port を渡せばここで 'auto on' を送る。磁界が ON/OFF していない状態で測ると
      段差ゼロ＝「検波器が壊れている」と誤読するので（2026-09-09 に実際に踏んだ）、
      できるだけ --port を付けて実行すること。"""
    rd = None
    if a.port:
        rd = open_reader(a.port)
        if rd is not None:
            rd.cmd("auto on", 0.4)
            print("  reader      : %s に 'auto on' を送信（磁界を OFF/ON させる）" % rd.port)
            rd.close()
    else:
        print("  ! --port 未指定。リーダ側で 'auto on' が回っていないと段差が出ません")
    ad = AD(); h = ad.h
    try:
        nbuf = 4096
        dwf.FDwfAnalogInReset(h)
        dwf.FDwfAnalogInChannelEnableSet(h, ctypes.c_int(0), ctypes.c_int(1))
        dwf.FDwfAnalogInChannelRangeSet (h, ctypes.c_int(0), ctypes.c_double(a.vrange))
        dwf.FDwfAnalogInChannelFilterSet(h, ctypes.c_int(0), ctypes.c_int(filterAverage))
        dwf.FDwfAnalogInFrequencySet    (h, ctypes.c_double(a.det_fs))
        dwf.FDwfAnalogInBufferSizeSet   (h, ctypes.c_int(nbuf))
        dwf.FDwfAnalogInAcquisitionModeSet(h, acqmodeSingle)
        dwf.FDwfAnalogInTriggerSourceSet(h, ctypes.c_ubyte(trigsrcNone))
        print("検波出力を %.0f 秒観測します（リーダで 'auto on' を回しておくこと）..."
              % a.seconds)
        vals = []
        t0 = time.monotonic()
        first = True
        while time.monotonic() - t0 < a.seconds:
            # ★初回だけ reconfigure=1。毎回 1 にすると入力段が作り直されて AD3 が鳴る。
            dwf.FDwfAnalogInConfigure(h, ctypes.c_int(1 if first else 0), ctypes.c_int(1))
            first = False
            while not analogin_done(h):
                time.sleep(0.001)
            vals.append(read_ch0(h, nbuf))
        v = np.concatenate(vals)
        # ★過渡スパイク（磁界 ON/OFF の瞬間、実測 ±0.2 V）を統計から外す。
        #   段差が 20 mV 台しかないのに min/max が ±0.2 V まで開くので、そのまま
        #   ヒストグラムを引くとビン幅 21 mV = 実信号が 2 ビンに潰れて二山が見えない。
        #   2 平均法は単峰の塊でも必ず分割を返すので、これを放置すると「段差がある」
        #   と誤読する。0.5-99.5 パーセンタイルの外を外れ値として分離する。
        clip_lo, clip_hi = np.percentile(v, [0.5, 99.5])
        span = max(clip_hi - clip_lo, 1e-4)
        clip_lo -= 0.2 * span
        clip_hi += 0.2 * span
        outlier = (v < clip_lo) | (v > clip_hi)
        v_raw, v = v, v[~outlier]
        # ★分位点で OFF/ON を分けてはいけない: リーダの auto は磁界 ON が周期の
        #   9 割を占めるので、5% 分位ですら ON 側の分布に埋もれる（実測: 5%=1.167V,
        #   95%=1.221V となり「段差 0.05V」と誤判定して、閾値が ON の真ん中に落ちた）。
        #   2 平均法(Lloyd)で二山に分離する。
        t = (float(v.min()) + float(v.max())) / 2.0
        for _ in range(40):
            lo_m = v[v < t]
            hi_m = v[v >= t]
            if len(lo_m) == 0 or len(hi_m) == 0:
                break
            nt = (lo_m.mean() + hi_m.mean()) / 2.0
            if abs(nt - t) < 1e-6:
                t = nt; break
            t = nt
        lo = float(v[v < t].mean()) if (v < t).any() else float(v.min())
        hi = float(v[v >= t].mean()) if (v >= t).any() else float(v.max())
        duty = 100.0 * float((v >= t).mean())
        mid = (lo + hi) / 2.0
        print()
        print("  サンプル数    : %d (%.1f ms 相当)" % (len(v_raw), len(v_raw) / a.det_fs * 1e3))
        print("  min / max     : %+.4f / %+.4f V  (全サンプル)" % (v_raw.min(), v_raw.max()))
        print("  外れ値        : %d 個 (%.3f %%)  = %+.4f V の外 / %+.4f V の外"
              % (outlier.sum(), 100.0 * outlier.mean(), clip_lo, clip_hi))
        print("  以下は外れ値を除いた本体の統計:")
        print("  field OFF     : %+.4f V" % lo)
        print("  field ON      : %+.4f V   (時間の %.1f %%)" % (hi, duty))
        print("  段差          : %.4f V" % (hi - lo))
        print()
        if hi - lo < 0.01:
            print("  ★段差がほぼ無い。磁界が ON/OFF していないか、検波器が拾えていない。")
            print("    リーダで 'auto on' が回っているか確認すること。")
        elif duty > 99.0 or duty < 1.0:
            print("  ★片方の状態しか観測できていない。'auto on' を確認し、--seconds を伸ばすこと。")
        else:
            # ★dwf の range は peak-to-peak（--vrange 5.0 = ±2.5 V）。しかも AD3 が
            #   実際に持つのは 5 V と 50 V の 2 段だけで、それ以外は丸められる。
            #   2026-08-20 実測: 検波出力は +6.90 V あり、--vrange 5.0 では 2.75 倍の
            #   過大入力。フロントエンドが飽和して (a) ON レベルが 2.66 V に頭打ち
            #   （＝段差を 1.7-2.4 V と誤って読む）(b) クリップ復帰の暴れが偽トリガに
            #   なる。「1 磁界 ON で 2 回送信」の一因がこれだった。
            #   → 実機が持つレンジ候補の中から、観測ピークに余裕を持って収まる
            #     最小のものを選ぶ。
            peak = max(abs(float(v_raw.min())), abs(float(v_raw.max())))
            vr = pick_range(h, peak * 1.3)
            print("  --> 推奨:  --level %.3f  --hyst %.3f  --vrange %.1f"
                  % (mid, max(0.002, (hi - lo) / 8), vr))
            if vr > a.vrange:
                print("  ★今回の測定は --vrange %.1f (=±%.2f V) で行ったが、ピークが %.3f V"
                      % (a.vrange, a.vrange / 2, peak))
                print("    ある。クリップしているので上の数値は信用できない。")
                print("    --vrange %.1f で測り直すこと。" % vr)
        # ★CH2（アンテナの生波形）で「そもそも搬送波が出ているか」も確かめる。
        #   検波出力の段差が出ない理由は «磁界が無い» と «検波器が拾えていない» の
        #   2 つあり、CH1 だけでは区別できない（2026-09-09 に半日溶かした）。
        car = measure_carrier_best(h, a)
        print()
        print("  CH2 (アンテナ生波形): Vpp=%.3f V  片振幅=%.3f V  %s"
              % (car["vpp"], car["amp"],
                 ("%.4f MHz" % (car["freq"] / 1e6)) if car["present"] else "(搬送波なし)"))
        if not car["present"]:
            print("  ★CH2 に何も来ていない。磁界が出ていない or CH2 が繋がっていない。")
            print("    carrier モード（--port 付き）で field OFF/ON を比べると切り分けられる。")
        elif car["clipped"]:
            print("  ★CH2 がクリップしている（レンジ ±%.1f V）。--carrier-atten に分圧比を"
                  % (car["full"],))
            print("    渡すか、×10 プローブを入れること。")

        # ヒストグラム（二山になっていれば OK。外れ値を除いた本体だけを見る）
        counts, edges = np.histogram(v, bins=40)
        for c, e in zip(counts, edges):
            print("   %+7.4f V | %s" % (e, "#" * int(60 * c / counts.max())))
    finally:
        ad.close()


def mode_carrier(a):
    """★リーダが «今どれだけ出しているか» を測る（2026-09-22 の配線変更で可能になった）。

    CH2 = アンテナの生 13.56 MHz、CH1 = 検波出力の DC。--port を渡すと磁界を OFF/ON
    して両方測るので、「搬送波が出ているか」「どれだけ出ているか」「検波器がそれを
    どんな DC に変えているか」が 1 回で並ぶ。detector モードの --level を決める前に
    ここを見ると、段差ゼロの原因切り分けが速い。

    リーダの出力を変えるノブは 2 つある（どちらもこのモードから書いて実測できる）:
      1. VDD_RF = RFO が振る電圧。bring-up の ADJUST_REGULATORS(0xD6) が電源電圧に
         合わせて自動で選び、結果は REGULATOR_RESULT(0x2B) の reg_d[7:4] に出る
         （5 V 電源なら 3600 + 100*reg_d [mV]）。手で決めるなら
         REGULATOR_CONTROL(0x2A) の reg_s=1 + rege[6:3] = --reader-rege / --rege-sweep。
         ただし «上げる» 余地は無い（自動選択が既に電源電圧の上限）。
      2. RFO レベル 0x27（ファームの g_pim.txpwr が書くレジスタ）
         = --reader-rfo / --rfo-sweep。0x00 が最大で 0xFF は実質 OFF。
         下げる方向はこちらの方が広く効く（実測は下の sweep のコメント参照）。
      ★どちらも «磁界サイクルのたびに SET_DEFAULT が走る»（transparent を抜けるとき）
        ので 1 サイクルで消える。link/verify で恒久的に変えたいときはファーム側
        （pim_bringup / pim_rx_config）に書くこと。g_pim.txpwr は SWD 経由で
        pim_test.py から渡せる（host/pim_test.py の PIM_IO 参照）。
    """
    rd = open_reader(a.port) if a.port else None
    if rd is None:
        print("  ! --port が無いのでリーダには触らない。今の磁界の状態をそのまま測る。")
    ad = AD(); h = ad.h
    try:
        reg = None
        if rd is not None:
            rd.cmd("listen off", 0.8)   # 捕捉も磁界サイクルも止めて、静止した磁界で測る
            print("  reader      : %s に 'listen off'（捕捉と磁界サイクルを停止）" % rd.port)
            reg = reader_regulator(rd)
            if reg is None:
                print("  ! レジスタが読めなかった（コンソールの応答なし）")
            elif reg.get("unsupported"):
                print("  ! IC_IDENTITY=0x%02X は ST25R3911B ではない。VDD_RF の読み出しは"
                      " 3911B(NFC05A1) 版だけ対応" % reg["ident"])
            else:
                print("  chip        : %s 電源モード / VDD_RF = %d mV "
                      "(%s reg_d=%d rege=%d)"
                      % ("3.3V" if reg["sup3v"] else "5V", reg["vddrf_mv"],
                         "reg_s=1 手動" if reg["manual"] else "reg_s=0 自動",
                         reg["reg_d"], reg["rege"]))
                print("                = RFO が振る電圧。アンテナ端はここに整合回路の Q が乗る")
                if not reg["manual"] and reg["reg_d"] == 0x0F:
                    # ★ファームは磁界サイクルのたびに SET_DEFAULT する（transparent 脱出）。
                    #   ADJUST_REGULATORS はそのあと «走らない» ので、実験中の VDD_RF は
                    #   bring-up が選んだ値ではなく SET_DEFAULT の既定（reg_d=15）になる。
                    print("                ※reg_d=15 は SET_DEFAULT 直後の既定値。bring-up の")
                    print("                  ADJUST_REGULATORS の結果を見たいなら 'bringup' 後に測ること")

        def snap():
            car = measure_carrier(h, a.carrier_ch, a.carrier_fs, a.carrier_nbuf,
                                  a.carrier_atten)
            det = measure_dc(h, DETECT_CH, a.vrange, a.det_fs)
            return car, det

        rows = []
        if rd is not None:
            rd.cmd("field off", 0.5); time.sleep(0.05)
            rows.append(("field OFF",) + snap())
            rd.cmd("field on", 0.5);  time.sleep(0.05)
            rows.append(("field ON",) + snap())
        else:
            rows.append(("現状",) + snap())

        print()
        print("---- 実測 (CH2 = アンテナ生波形 / CH1 = 検波出力の DC) ----")
        print("   状態        CH2 Vpp    CH2 片振幅   CH2 Vrms     CH2 周波数      CH1 DC")
        for label, car, det in rows:
            print("   %-10s %8.3f V %9.3f V %9.3f V  %13s  %+8.4f V"
                  % (label, car["vpp"], car["amp"], car["vrms"],
                     ("%.5f MHz" % (car["freq"] / 1e6)) if car["present"] else "-",
                     det["mean"]))
        on = next((r for r in rows if r[0] != "field OFF"), None)
        off = next((r for r in rows if r[0] == "field OFF"), None)
        if on is not None and off is not None:
            print("   %-10s %8.3f V %9s %9s  %13s  %+8.4f V"
                  % ("段差", on[1]["vpp"] - off[1]["vpp"], "-", "-", "-",
                     on[2]["mean"] - off[2]["mean"]))
        print()
        car = on[1] if on is not None else rows[0][1]
        print("  CH2 レンジ    : ±%.2f V (atten=%.1f, %.1f MSps, %d sample)"
              % (car["full"], a.carrier_atten, car["fs"] / 1e6, a.carrier_nbuf))
        if car["clipped"]:
            print("  ★クリップしている。表示値は信用できない。分圧か ×10 プローブを入れ、")
            print("    その比を --carrier-atten に渡すこと（AD3 の入力は ±25 V が限界）。")
        if car["present"]:
            ppm = (car["freq"] - CARRIER_HZ) / CARRIER_HZ * 1e6
            print("  搬送波        : %.5f MHz (公称 13.56 MHz から %+.0f ppm)"
                  % (car["freq"] / 1e6, ppm))
            if reg and not reg.get("unsupported"):
                print("  VDD_RF vs 実測: %d mV (チップ内部) -> アンテナ端 %.3f Vpp"
                      % (reg["vddrf_mv"], car["vpp"]))
        else:
            print("  ★CH2 に搬送波が見えない。磁界 OFF のままか、CH2 の結線を確認すること。")

        # ---- 出力を手で振って測る（ノブは 2 つ）-------------------------
        #  ★実測 2026-09-22（メアンダコイル服の結合 = CH2 60 mVpp のとき）:
        #      VDD_RF  3600 -> 5100 mV : CH2 0.051 -> 0.060 Vpp
        #        rege<=6 では変わらず、7 から効き始め、12 以上は電源 5.13 V に
        #        頭打ち（だから ADJUST_REGULATORS は reg_d=11 を選ぶ）= 効き幅 20%
        #      0x27    0x00 -> 0xFF    : CH2 0.061 -> 0.008 Vpp（0xFF で実質 OFF）
        #    つまり «下げる» なら 0x27 が効く。«上げる» のは既定が既に上限なので無理。
        want = [k for k in ("rege", "rfo") if getattr(a, k + "_sweep")
                or getattr(a, "reader_" + k) is not None]
        if want:
            if rd is None:
                print("\n  ! 出力の変更は --port が必要（コンソールからレジスタを書く）")
            elif reg is None or reg.get("unsupported"):
                print("\n  ! 出力の手動設定は ST25R3911B(NFC05A1) 版だけ対応")
            else:
                def sweep(title, rows, restore, note):
                    """rows = [(表示, コンソールに送る 1 行), ...] を順に書いて測る。"""
                    print()
                    print(title)
                    print("   設定                CH2 Vpp    CH2 片振幅     CH1 DC")
                    rd.cmd("field on", 0.4)
                    for label, line in rows:
                        rd.cmd(line, 0.4)
                        time.sleep(0.05)
                        car_, det_ = snap()
                        print("   %-18s %7.3f V  %9.3f V  %+8.4f V"
                              % (label, car_["vpp"], car_["amp"], det_["mean"]))
                    if restore:
                        rd.cmd(restore, 0.4)
                    print("   -> %s" % note)

                if "rege" in want:
                    sup3v = reg["sup3v"]
                    vals = range(16) if a.rege_sweep else [a.reader_rege]
                    sweep("---- VDD_RF = RFO が振る電圧 "
                          "(REGULATOR_CONTROL 0x2A: reg_s=1, rege[6:3]) ----",
                          [("rege=%-2d (%d mV)" % (r, vddrf_mv(r, sup3v)),
                            "w 2a %02X" % (0x80 | (r << 3))) for r in vals],
                          "w 2a 00" if a.rege_sweep else None,
                          "reg_s=0 に戻した（ADJUST_REGULATORS の結果 = 上限）"
                          if a.rege_sweep else
                          "rege=%d のまま残した。次の磁界サイクルの SET_DEFAULT で消える"
                          % a.reader_rege)

                if "rfo" in want:
                    vals = (list(range(0, 0x100, 0x20)) + [0xFF] if a.rfo_sweep
                            else [a.reader_rfo])
                    sweep("---- RFO レベル (0x27 = ファームの g_pim.txpwr) ----",
                          [("0x27=0x%02X" % v, "w 27 %02X" % v) for v in vals],
                          "w 27 00" if a.rfo_sweep else None,
                          "0x27=0x00 に戻した（既定 = 最大出力）" if a.rfo_sweep else
                          "0x27=0x%02X のまま残した。次の磁界サイクルの SET_DEFAULT で消える"
                          % a.reader_rfo)
    finally:
        if rd is not None:
            if not a.keep_field:
                rd.cmd("field off", 0.4)
                print("\n  reader      : 'field off' を送って磁界を切った"
                      "（--keep-field で ON のまま残せる）")
            print("  再開するには リーダのコンソールで 'listen on' / 'cycle on'")
            rd.close()
        ad.close()


# ---------------------------------------------------------------------------
#  ラウンドごとのランダム化（論文の評価と同じく data / backoff を毎回引き直す）
# ---------------------------------------------------------------------------
#  論文の最適値は BACKOFF_MAX = 2^14。AD3 既定構成の DigitalOut バッファ（2048 bit/ch
#  = 1024 TU）には入らないので、cfg 4（32768 bit/ch = 16384 TU）で開く必要がある。
#  さらにリーダの捕捉窓は 13.1 ms 固定（424k で約 5554 TU）なので、抽選が窓より後ろに
#  落ちたぶんは «観測できない» ものとして TSV に out_of_window で残し、統計から外す。
BACKOFF_MAX_DEFAULT = 1 << 14        # 論文と同じ 16384 TU
DIO_CONFIG_BIGBUF   = 4              # DigitalOut 32768 bit/ch になるデバイス構成


SIM_MAX_TAGS_DEFAULT = 100          # シミュレータ config の max_tags と同じ値にする
DATA_MEAN_DEFAULT = 128.0           # --data-dist gauss の平均（シミュレータ config の data_mean）
DATA_STD_DEFAULT = 30.0             # --data-dist gauss の標準偏差（同 data_std）


class SimTagRng:
    """シミュレータと «同じ手順・同じ順番» で data / backoff を引く numpy 乱数。

    simulation/src/common/simulator.py の 1 ラウンドは
      1. max_tags 個の PimTag を作る（1 個ごとに uid, data, backoff の順で integers()）
      2. rng.choice(pool, N, replace=False) で N 個選ぶ
    なので、ここでも同じ回数・同じ順で乱数を消費し、選ばれた N 個の (data, backoff) を
    base の順にタグへ割り当てる。seed / max_tags / id_bits / backoff_max / N が同じで、
    シミュレータ側がその N だけを seed から回した場合（config の reseed_per_nodes: true）、
    ラウンド k の抽選はシミュレータのラウンド k と一致する。"""

    def __init__(self, seed: int, max_tags: int = SIM_MAX_TAGS_DEFAULT,
                 id_bits: int = pim.ID_BITS, data_dist: str = "uniform",
                 data_mean: float = DATA_MEAN_DEFAULT, data_std: float = DATA_STD_DEFAULT):
        self.rng = np.random.default_rng(seed)
        self.max_tags, self.id_bits = max_tags, id_bits
        self.data_dist, self.data_mean, self.data_std = data_dist, data_mean, data_std

    def _data(self) -> int:
        """simulation/src/pim/tag.py の draw_data と同じ式（config の data_dist）。"""
        if self.data_dist == "gauss":
            v = self.rng.normal(loc=self.data_mean, scale=self.data_std)
            return int(np.clip(np.rint(v), 1, 255))   # 四捨五入して 1..255 に丸める
        return int(self.rng.integers(1, 256))          # 1..255

    def draw(self, n: int, backoff_max: int) -> list[tuple[int, int]]:
        if n > self.max_tags:
            raise ValueError("タグ数 %d > --sim-max-tags %d" % (n, self.max_tags))
        r = self.rng
        pool = []
        for _ in range(self.max_tags):                 # PimTag.__init__ と同じ順
            r.integers(2 ** self.id_bits)              # uid（実機では使わないが消費する）
            data = self._data()                        # uniform / gauss
            backoff = int(r.integers(backoff_max))     # 0..backoff_max-1
            pool.append((data, backoff))
        return [pool[i] for i in r.choice(self.max_tags, n, replace=False)]


def draw_tags(base: list[TagSpec], rng: SimTagRng, random_data: bool, random_backoff: bool,
              backoff_max: int) -> list[TagSpec]:
    """base のピン割り当て・ID はそのままに、data / backoff だけ引き直した TagSpec を返す。

    片方だけランダムにする場合も、系列をシミュレータと揃えるため両方ぶん乱数を消費する。"""
    out = []
    for t, (data, backoff) in zip(base, rng.draw(len(base), backoff_max)):
        out.append(TagSpec(t.pins, t.tag_id,
                           data if random_data else t.data,
                           backoff if random_backoff else t.backoff_tu))
    return out


def split_sent(tags: list[TagSpec], dropped_ids, mult: int):
    """抽選を «実際に送信した cycles» と «送信しなかった cycles» に分ける。

    ★バッファに収まらず無送信になったタグ（build_bits / FastTx.load の dropped）を
      送信扱いにすると、TSV に出ていないパルスが tx_* として残り、rx_miss / collision
      に数えられる（mult=1 で抽選の約 1.2 %）。シミュレータには «送らない抽選» が
      無いので、比べるときは分母から外す必要がある。"""
    drop = set(dropped_ids or ())
    sent = [t.cycle(mult) for t in tags if t.tag_id not in drop]
    skipped = [t.cycle(mult) for t in tags if t.tag_id in drop]
    return sent, skipped


def _prepare(a):
    tags = parse_tags(a.tags)
    validate(tags, a.mult, getattr(a, "random_backoff", False))
    ad = AD(config=a.dio_config)
    h = ad.h
    fdio = dio_internal_clock(h)
    sps_tu, div, fsc_act, ppm = pick_sps_tu(fdio, a.sps_tu)
    rate = fdio / div
    cap_bits = dio_buffer_bits(h, tags[0].pins[0])
    bitmap, total_tu, nbits, _ = build_bits(tags, sps_tu, a.mult, cap_bits // sps_tu)
    report_plan(tags, sps_tu, div, fsc_act, ppm, total_tu, nbits, rate, a.mult, cap_bits,
                reader_fs=reader_fs_hz(a))
    a.sps_tu_used = sps_tu
    return ad, h, tags, bitmap, div, rate, nbits


def mode_selftest(a):
    """検波回路もリーダも不要。PC からソフトトリガして DIO を撃つ。
    スコープや WaveForms の Logic で波形が出ているかを目視するための足場。"""
    ad, h, tags, bitmap, div, rate, nbits = _prepare(a)
    try:
        run_s = program_digitalout(h, bitmap, div, rate, trigsrcPC, a.sps_tu_used, repeat=1)
        print("selftest: PC トリガで %d 回撃つ（1 回 = %.3f ms）。" % (a.count, run_s * 1e3))
        print("          WaveForms の Logic で DIO を見るか、スコープでタグの CTRL を見ること。")
        for i in range(a.count):
            dwf.FDwfDeviceTriggerPC(h)
            print("  trigger %d/%d" % (i + 1, a.count))
            time.sleep(max(a.gap, run_s * 1.5))
            dwf.FDwfDigitalOutConfigure(h, ctypes.c_int(1))       # 次の 1 発ぶんを再武装
        print("done.")
    finally:
        ad.close()


# ---------------------------------------------------------------------------
#  verify: 検波もトリガも使わず「リーダが復調できるか」だけを確かめる
# ---------------------------------------------------------------------------
RX_RE  = re.compile(r"\[rx #(\d+)\]\s+sof=(\d+)\s+tags=(\d+)")
TAG_RE = re.compile(r"tag_id=\s*(\d+)\s+data=\s*(\d+)\s+interval=\s*(-?\d+)")


def open_tsv(a, tags, window_tu=None, seed=None):
    """--tsv が指定されていれば TSV ロガーと RoundRecorder を作る（無ければ (None, None)）。

    論文シミュレータ互換の形式で log/YYYYMMDD_HHMMSS/YYYYMMDD_HHMMSS.tsv に残す。詳細は
    [host/pim_tsvlog.py](pim_tsvlog.py)。実験条件（seed 等）は TSV の列を汚さない
    ように同名の .meta.txt へ書く。"""
    if getattr(a, "tsv", None) is None:
        return None, None
    path = a.tsv or None                      # "--tsv" だけなら自動命名
    log = pim_tsvlog.PimTsvLog(path)
    rec = pim_tsvlog.RoundRecorder(log, [t.cycle(a.mult) for t in tags],
                                   window_tu=window_tu)
    meta = {
        "mode": a.mode, "fsc_hz": "%.1f" % pim.FSC, "tu_us": "%.6f" % (pim.TU * 1e6),
        "subcarrier_div": pim.SUBCARRIER_DIV, "data_tu_mult": a.mult,
        "tags": ";".join("dio%s:id%d" % (t.label, t.tag_id) for t in tags),
        "label": getattr(a, "label", None) or "",
        "random_data": bool(getattr(a, "random_data", False)),
        "data_dist": (getattr(a, "data_dist", "uniform")
                      if getattr(a, "random_data", False) else "fixed"),
        "data_mean": getattr(a, "data_mean", ""), "data_std": getattr(a, "data_std", ""),
        # 固定 data のときのタグごとの値（--tags の順）
        "tag_data": ("" if getattr(a, "random_data", False)
                     else ";".join(str(t.data) for t in tags)),
        "target_packets": getattr(a, "target_packets", None) or "",
        "random_backoff": bool(getattr(a, "random_backoff", False)),
        "backoff_max_tu": getattr(a, "backoff_max", None),
        "seed": seed, "rng": "numpy.default_rng (simulator-compatible)",
        "sim_max_tags": getattr(a, "sim_max_tags", None),
        "reader_sps_milli": a.reader_sps,
        "reader_rxconf": ("" if getattr(a, "reader_rxconf", None) is None
                          else "0x%02X" % a.reader_rxconf),
        "reader_agc": "" if getattr(a, "reader_agc", None) is None else a.reader_agc,
        "reader_gain": "" if getattr(a, "reader_gain", None) is None else a.reader_gain,
        "reader_gain2": ("" if getattr(a, "reader_gain2", None) is None
                         else a.reader_gain2),
        "reader_rate": getattr(a, "reader_rate", None),
        "reader_period_ms": a.reader_period, "reader_gap_ms": a.reader_gap,
        "capture_window_tu": "" if window_tu is None else "%.1f" % window_tu,
    }
    mp = pim_tsvlog.write_meta(log.path, meta)
    print("  TSV log     : %s" % log.path)
    print("  meta        : %s" % mp)
    return log, rec


def open_reader(port=None):
    """リーダの UART コンソール（ST-Link VCP）。pim_tune.py の Reader を再利用。"""
    try:
        from pim_tune import Reader
        return Reader(port)
    except SystemExit as e:
        print("  ! リーダのコンソールを開けない: %s" % e)
    except Exception as e:
        # repr だとポート名が落ちるので str（"[Errno 13] ...: '/dev/cu.usbmodemXXXX'"）で出す
        print("  ! リーダのコンソールを開けない: %s: %s" % (type(e).__name__, e))
    return None


def echo_reader(out: str, indent: str = "                ") -> None:
    """リーダの応答のうち [..] で始まる行だけをそのまま出す。

    ★free-run では `auto on` を打たない。`listen on` が本当に効いたのか
      （[listen] ON fs=... が返るか）を目で確認できないと、「タグが届いていない」のか
      「リーダが聞いていない」のかが切り分けられないので、必ず見せる。"""
    for line in (out or "").splitlines():
        line = line.strip()
        if line.startswith("["):
            print(indent + line)


def reader_stat(rd) -> None:
    """リーダの `stat` を打って 1 段目（活動判定）の実測値を見せる。

    読み方（firmware-NFC05A1-424k-ringbuffer/pim_ring424k/pim.ino の seg_feed）:
      listen=0        -> 捕捉が走っていない。'listen on' が効いていない。
      blocks が増えない -> 同上（走査ループが回っていない）。
      max < act_thr   -> 変調がしきい値に届いていない ＝ タグの負荷変調が弱い/届いていない。
                         act_thr を下げる（`thr <n>`）か、結合・変調深度を直すこと。
      segments>0 なのに [rx] が出ない -> 切り出せているが復号できていない（sps/mult/衝突）。
    """
    out = rd.cmd("stat", 0.6)
    print("  ---- reader stat ----")
    echo_reader(out, "  ")


def send_rxconf(rd, a) -> None:
    """--reader-rxconf が指定されていれば、リーダの RX_CONF1 を切り替える。

    ★これは «リーダの受信チャネル» の選択（RX_CONF1 bit7）で、2026-09-22 に
      「タグ 423.7kHz に対しリーダの捕捉列が 844.6kHz = 2 倍になる」問題の原因だった。
      2 倍になると SOF 長まで 2 倍に化け、sps をどう合わせても ID が読めない
      （sof は立つのに tags=0 が延々と続く）。bit7=1（0xA0）で 1:1 に戻る。
      負荷変調が振幅側に乗るか位相側に乗るかは «結合と同調» で決まるので、コイルや
      アンテナを触ったら 0x20 と 0xA0 の両方を試して sof/一致率で選ぶこと。
    ★ファーム側に `rxconf <hex>` コマンドが必要（2026-09-22 に追加）。古いファームだと
      '? unknown' が返るだけで無視される。"""
    if getattr(a, "reader_rxconf", None) is None:
        return
    out = rd.cmd("rxconf %02X" % a.reader_rxconf, 0.5)
    if "unknown" in out:
        print("  ! このファームには 'rxconf' コマンドが無い（--reader-rxconf は無視される）")
    else:
        print("  reader      : RX_CONF1=0x%02X" % a.reader_rxconf)


def send_agc(rd, a) -> None:
    """--reader-agc / --reader-gain / --reader-gain2 をリーダへ送る（RX_CONF2..4）。

    ★AGC と Squelch は «ゲインを下げる» 方向にしか働かない機構で、しかも framing が
      動いている前提（DS11793 p.9-10）。Transparent では framing を通さないので本来
      起動しないはずだが、既定では両方 ON のままなので «受信 FE の詰まり» の原因候補が
      残る。ファーム既定は off（= ゲイン固定）。ここで on に戻して比較できる。
    ★AGC を off にするとゲインは RX_CONF3（1 段目）/ RX_CONF4（2/3 段目）で «手動» に
      なる。--reader-gain 0 で 1 段目フルゲイン、7 で +5.5dB boost。既定 6 = 最小。
    ★実際に減衰が起きているかはリーダの `stat` の gain_red（0x2D）で読める。
    ファーム側に `agc` / `gain` / `gain2` コマンドが必要（2026-09-28 に追加）。"""
    for flag, cmd, label in (("reader_agc",   "agc %s",   "AGC+squelch"),
                             ("reader_gain",  "gain %d",  "1st stage gain"),
                             ("reader_gain2", "gain2 %d", "2nd/3rd stage gain")):
        v = getattr(a, flag, None)
        if v is None:
            continue
        out = rd.cmd(cmd % v, 0.5)
        if "unknown" in out:
            print("  ! このファームには '%s' コマンドが無い（--%s は無視される）"
                  % (cmd.split()[0], flag.replace("_", "-")))
        else:
            print("  reader      : %s = %s" % (label, v))


# リーダの捕捉レート設定（`rate <0..3>`）と、その時の sps [samples/TU] の目安。
#   BR=n -> 公称 PCLK2/(2<<n)。sps = その周波数 / FSC(423.75kHz)
# ★rate ごとの «実測校正値» sps x1000。公称（40Msps/423.75kHz = 94395）ではなく、
#   実機を掃引して «data の誤差が消える» 値を入れる。
#   ここがずれていると誤差は interval に比例するので、**data が大きいほど off-by-1 が
#   増える**（誤差の符号は常に同じ）。これが `NG(want data=...)` の正体。
#   2026-09-25 実測（4 タグ / 40 s / 詰まり検出 resync 有効後。それ «以前» の掃引は
#   受信 FE の全滅バーストに汚染されていて、11805 が最良に見えていた）:
#       sps   | NG 率 | 誤差の符号 | data>=192 の誤り率
#       11805 | 25.4% |  +1        | 66%
#       11820 |  9.4% |  +1        | 16%
#       11833 |  0.4% |  ±1 が 3 件| 1%     <- 実効値
#       11845 |  9.1% |  -1        | 10%
#   ★この値はリーダ個体（STM32 の SPI クロック）に依存する。別の個体では
#     link の最後に出る「sps 校正」の推奨値を使うこと（fit_sps_milli が毎回出す）。
# 間隔の «下駄» [TU]: ファームは body(ID 先頭) 同士の差から
#   interval = dbody/sps - (SOF_POST - SOF_PRE) - PRE_PULSE  を出す。
#   つまり «測っているサンプル距離» は (data*mult + C) TU 相当。sps 校正の式で使う。
def erf_vec(x):
    """numpy 配列の erf（scipy 無しで済ませる）。"""
    return np.vectorize(math.erf, otypes=[float])(x)


C_INTERVAL_TU = pim.PRE_PULSE_TU + (pim.SOF_POST_TU - pim.SOF_PRE_TU)   # = 25

READER_RATE_SPS = {0: 94400, 1: 47200, 2: 23700, 3: 11835}   # 実測校正値
# ★既定は rate 3（公称 5 Msps）。復号 24.7 -> 14.2 ms・リング 52 -> 104 ms になり、
#   磁界 ON の後ろで撃ったタグの取りこぼしが消える（2026-09-22 実測 50 % -> 100 %）。
READER_RATE = 3


def fit_sps_milli(pairs, sps_milli_used: int, mult: int = 1):
    """(送った data, 復号された data) の組から «正しい sps» と «残りジッタ» を推定する。

    モデル
    ------
    リーダは 2 パルスの間隔をサンプル数で測り、sps で割って TU にする:
        data_est = round( ( dsample / sps_used - (SOF差) - PRE_PULSE ) / mult )
    sps_used が真値より eps だけ小さいと dsample は (1+eps) 倍に見えるので
        data_est = round( data_true + eps * (data_true * mult + C) / mult + jitter )
    つまり誤差は **data に比例する決定的な項（eps）** と **ゼロ平均のジッタ** の和。
      * eps != 0  -> 大きい data ほど off-by-1 が増え、符号は常に同じ（`NG(want data=)`）
      * ジッタ    -> data に依らず ±1 が両側に出る。sps を合わせても消えない床

    推定
    ----
    «丸めのある観測» なので最小二乗も «不一致の数え上げ» も使えない
      - 最小二乗: 階段を直線で近似する形になり傾きが 2 倍に出る（真値 +0.24% を +0.50%）
      - 数え上げ: eps*(255*mult+C)/mult < 0.5 の範囲では全点 got=want と «説明できて»
                  しまい、eps が決まらない（±21 の平坦部になる）
    ジッタを含めた最尤推定にする。誤差 k の確率は
        P(k) = Phi((k+0.5-b)/sigma) - Phi((k-0.5-b)/sigma),  b = eps*(data*mult+C)/mult
    で、(eps, sigma) の 2 段グリッド（粗→細）で対数尤度を最大化する。
    実測（同じ 4 つのログを 11805/11820/11833/11845 の各 sps で取ったもの）で
    **どの sps から出発しても 11833〜11837 に収束**した。

    戻り: (推奨 sps_milli, eps, sigma[TU], 点数) / 点が足りなければ None。
    """
    cells = {}                            # (data*mult + C, 誤差) -> 個数 に畳む
    for w, g in pairs:
        if w <= 0 or abs(g - w) > 3:      # 粗い外れ値（ID 誤り・ラウンドずれ）は捨てる
            continue
        cells[(w * mult + C_INTERVAL_TU, g - w)] = cells.get((w * mult + C_INTERVAL_TU,
                                                              g - w), 0) + 1
    if sum(cells.values()) < 50:
        return None
    d = np.array([k[0] for k in cells], dtype=float) / float(max(1, mult))
    k = np.array([k[1] for k in cells], dtype=float)
    c = np.array(list(cells.values()), dtype=float)

    def loglik(eps, sigma):
        b = eps * d
        z1 = (k + 0.5 - b) / (sigma * math.sqrt(2.0))
        z0 = (k - 0.5 - b) / (sigma * math.sqrt(2.0))
        pr = 0.5 * (erf_vec(z1) - erf_vec(z0))
        return float(np.sum(c * np.log(np.maximum(pr, 1e-12))))

    best = None
    for eps_step, sig_step, rng in ((1e-4, 0.05, None), (1e-5, 0.01, 1)):
        if rng is None:
            eps_list = [i * eps_step for i in range(-40, 41)]
            sig_list = [0.05 + i * sig_step for i in range(0, 16)]
        else:                              # 粗い最良点のまわりを細かく
            eps_list = [best[1] + i * eps_step for i in range(-12, 13)]
            sig_list = [max(0.03, best[2] + i * sig_step) for i in range(-6, 7)]
        for eps in eps_list:
            for sg in sig_list:
                ll = loglik(eps, sg)
                if best is None or ll > best[0]:
                    best = (ll, eps, sg)
    _, eps, sigma = best
    return int(round(sps_milli_used * (1.0 + eps))), eps, sigma, int(sum(cells.values()))


def err_floor(sigma: float, mult: int = 1) -> float:
    """sps を完璧に合わせても残る data の誤り率（ジッタ sigma[TU] のみによる）。
    1 TU 刻みの value-to-interval では |jitter| > 0.5*mult TU で ±1 に飛ぶ。"""
    z = 0.5 * mult / (sigma * math.sqrt(2.0))
    return float(1.0 - erf_vec(np.array([z]))[0])


def reader_fs_hz(a) -> float:
    """リーダの捕捉レート [Hz]。rate 0..3 = PCLK2/2 を 2^rate で割った値（公称）。"""
    r = getattr(a, "reader_rate", None)
    return 40e6 / (1 << (r if r is not None else READER_RATE))


def tx_wait_s(a) -> float:
    """検波トリガから実際に撃つまでの待ち [s]。

    既定（--tx-wait auto）は «リーダの磁界 OFF 時間 gap» と同じにする。検波は磁界 OFF の
    過渡でも 1 回トリガするので、そのぶん待てば «磁界が戻った瞬間» に着地して無駄撃ちが
    消える（2026-09-22 実測: gap 20 ms で backoff 2.4/11.8 ms のタグが 49 % -> 100 %）。
    0 を渡すと従来どおりトリガ即送信。
    ★--free-run（検波なし・ホストのタイマで撃つ）では磁界が常時 ON なので待つ理由が
      無い。gap そのものが存在しないので 0 を返す。"""
    if getattr(a, "free_run", False):
        return 0.0
    v = getattr(a, "tx_wait", "auto")
    if v in (None, "auto"):
        return (a.reader_gap or 0) / 1000.0
    return max(0.0, float(v))


def send_reader_rate(rd, a) -> None:
    """リーダの捕捉レートを変える（`rate 0..3` = 40/20/10/5 Msps 公称）。

    ★2026-09-22 実測: レートを 1 段下げる（rate 2 -> 3）と
        1 セグメントの復号時間 24.7 ms -> 14.2 ms、リングの長さ 52 ms -> 104 ms
      になり、«磁界 ON の後ろの方で送ったタグ» が落ちなくなる（50 % -> 100 %）。
      4 タグ同時でも overrun が増えなくなった。サブキャリア 424 kHz に対し
      5 Msps は 11.85 sample/TU で、復号はこの粒度でも成立する。
    ★sps は «レートとセット» で変えること（rate 3 なら --reader-sps 11850）。
      ずれていると間隔が比例してずれ、data が off-by-1 になる（実測 11900 で全件 -1）。"""
    if a.reader_rate is None:
        return
    rd.cmd("rate %d" % a.reader_rate, 0.6)
    want = READER_RATE_SPS.get(a.reader_rate)
    print("  reader      : rate %d (%s)" % (a.reader_rate,
          "公称 %.0f Msps" % (80.0 / (2 << a.reader_rate))))
    if want and a.reader_sps and abs(a.reader_sps - want) > want * 0.1:
        print("  ! sps=%d は rate %d の目安 %d と 10%% 以上ずれています（data がずれます）"
              % (a.reader_sps, a.reader_rate, want))


def send_cycle_timing(rd, a) -> None:
    """タグの送信間隔（= リーダの磁界サイクル）をリーダに設定する。

    period = OFF+ON の 1 周期 = «タグが何 ms ごとに送るか»、gap = そのうち OFF の時間。
    既定は READER_PERIOD_MS / READER_GAP_MS（このファイルの先頭）。

    ★gap は «タグが撃っても届かない死に時間» でもある（2026-09-22 実測）。検波は磁界
      OFF の過渡でも 1 回トリガするので、そこで撃ったタグは gap の間ずっと無磁界の中に
      送ることになり、backoff < gap のタグは半分ほど落ちる。検波の放電は 0.03 ms なので
      gap は 5 ms でも足りる。逆に ON は «最大 backoff + 復号時間» より長く要る。

    ★--free-run（検波なし）のときは磁界サイクルそのものを止める（`cycle off`）。
      タグは検波の立ち上がりでなくホストのタイマで撃つので、磁界は «常時 ON» の方が
      良い（OFF の谷に撃ち込む事故が原理的に消える）。リーダはリングバッファで
      連続捕捉しているので、タグの位相がどこであっても捕捉窓に入る。"""
    if getattr(a, "free_run", False):
        # ★順番が大事: `rate` / `rxconf` はレジスタに触るため pim_need_spi() が走り、
        #   連続捕捉（listen）と Transparent を «止めた» 状態で戻ってくる。検波版は
        #   そのあとの `auto on` が listen を復帰させていたので気付かない。free-run は
        #   `auto on` を打たないので、ここで自分で復帰させないと «リーダが何も聞いて
        #   いない»（[rx #n] が 1 行も出ない）状態になる。
        #   field on -> cycle off -> listen on の順（field on も pim_need_spi を呼ぶ）。
        print("  reader      : field on + cycle off + listen on"
              "（磁界 ON 固定のまま連続捕捉）")
        echo_reader(rd.cmd("field on", 0.4))   # 磁界は常時 ON（OFF の谷を作らない）
        echo_reader(rd.cmd("cycle off", 0.4))  # 磁界サイクルは止める（検波タグ用の機能）
        echo_reader(rd.cmd("listen on", 0.8))  # Transparent + リング連続捕捉を再開
        # ★受信フロントエンドの定期リセット（2026-09-25 追加）。
        #   検波トリガ運用では磁界サイクルが period ごとに SET_DEFAULT を通していて、
        #   それが «受信側を毎周期まっさらにする» 役目も兼ねていた。free-run で磁界を
        #   ON 固定にした結果その初期化が消え、一度復調が落ちると戻らなくなった
        #   （log/20260925_152819: 174 ラウンド目まで 3/3 一致 → 175 以降は弱い 2 タグが
        #    «SOF は見えるのに ID が読めない» まま最後まで復帰せず）。
        #   磁界は切らずに受信だけ入れ直す。0 を渡せば従来どおり無効。
        echo_reader(rd.cmd(("resync off" if not a.reader_resync
                            else "resync %d" % a.reader_resync), 0.5))
        # ★周期 resync は «保険»。本命はこちら（詰まり検出による即 resync）。
        #   受信 FE が «SOF は見えるのに ID が取れない» 状態に落ちるのは全タグ共通の
        #   事象なので、落ちた瞬間から全タグが同時に届かなくなる。周期を待たずに
        #   その場で入れ直すと、死角を増やさずに全滅の長さを 1〜2 セグメントに抑えられる。
        if a.reader_stuck is not None:
            echo_reader(rd.cmd(("stuck off" if a.reader_stuck == 0
                                else "stuck %d" % a.reader_stuck), 0.5))
        # ★2026-09-28: «無音» 検出。入れ直した受信 FE が稀に何も聞こえない状態で上がり、
        #   SOF すら出ないので stuck では拾えず、次の周期 resync まで全タグが落ちていた
        #   （50 ms 周期 5 分で 10 回・最長 37 ラウンド = 全 miss の 6 割）。
        #   タグは周期送信しているので «周期の数倍» 無音なら FE を疑って即入れ直す。
        #   古いファーム（quiet 未対応）は '? unknown' を返すだけで害は無い。
        quiet = a.reader_quiet
        if quiet is None or quiet < 0:
            quiet = 2 * (a.reader_period or 200) + 50
        echo_reader(rd.cmd(("quiet off" if quiet == 0 else "quiet %d" % quiet), 0.5))
        print("  送信間隔     : %d ms ごと（ホストのタイマで送信。検波は使わない）"
              % (a.reader_period or 200))
        return
    if a.reader_gap:
        rd.cmd("gap %d" % a.reader_gap, 0.4)
    if a.reader_period:
        rd.cmd("period %d" % a.reader_period, 0.4)
    if a.reader_period or a.reader_gap:
        print("  送信間隔     : %d ms ごと（磁界 OFF %d ms + ON %d ms）"
              % (a.reader_period, a.reader_gap, a.reader_period - a.reader_gap))


def watch_reader(rd, tags, seconds: float, poll=None, poll_dt: float = 0.1, recorder=None,
                 on_round=None, trigger_count=None):
    """リーダの出力を seconds 秒読み、[rx #n] ごとの復号結果を送信値と突き合わせる。

    poll を渡すと、UART を待つ合間に毎回呼ばれる（link モードが検波トリガの
    再アームをここに載せる）。poll_dt は select の待ち時間 = poll の呼ばれる間隔。
    検波トリガを世話するときは、リーダの auto 周期より十分短くすること。

    recorder を渡すと（host/pim_tsvlog.py の RoundRecorder）、論文シミュレータ互換の
    TSV に記録する。★1 ラウンド = «1 送信機会（検波トリガ 1 回）» であって、[rx #n]
    フレーム 1 個ではない。リーダは backoff がばらけた各タグを別セグメントとして切るので、
    1 回の送信に対し複数フレームを返す（実測 4.3 個/機会）。フレームごとにラウンドを
    切ると «そのフレームに写っていない他タグ» が毎回 rx_miss として記録され、実際は
    95 % 成功しているのに 27 % に見える（2026-09-22 にこれで誤読した）。
    そこで trigger_count が変わるまでは同じラウンドに貯め、変わった時点で確定する。

    on_round を渡すと [rx #n] のたびに呼ばれ、「そのフレームで実際に送信された
    cycles のリスト」と「抽選したが送信しなかった cycles のリスト」の組を返すこと
    （呼ばれた側は次ラウンド用の波形をロードし直す）。
    data がラウンドごとに変わるので、一致判定の期待値もここで更新する。

    trigger_count を渡すと（現在の検波トリガ累計を返す callable）、そのラウンドの
    start_signal 行に trigger= として残る。★「撃てていたのか／届いていたのか」を
    ログ単体で切り分けるための最重要情報なので、link では必ず渡すこと。"""
    want    = {t.tag_id: t.data for t in tags}
    # ★リーダのフレームは «1 つ前の送信» の復号結果として遅れて届くことがある
    #   （4 タグだと 1 機会に 4 フレーム、最後の 1 つは次のトリガ後に届きうる）。
    #   その値を «誤り» に数えると、実際は正しく復号できているのに誤り率が 3 割に
    #   見える（2026-09-22）。1 つ前の抽選の期待値も正解として扱う。
    prev_want = {}
    hits    = {t.tag_id: 0 for t in tags}
    wrong   = {}                      # tag_id -> [(data, 回数)] 相当のカウント
    unknown = {}                      # 送っていない tag_id が出てきた回数
    frames = sof_sum = 0
    pairs  = []                       # (want_data, got_data) — sps 校正の当てはめ用
    buf = b""

    # ★1 送信機会分を貯めるバッファ（上の docstring 参照）。trigger が進むまでは
    #   同じラウンドとして扱い、確定時に start_signal + tx + 受信結果をまとめて書く。
    cur = {"trig": None, "cyc": None, "drop": None, "sof": 0, "ntag": 0, "rx": {},
           "open": False}
    rounds = [0]

    def flush_round():
        if not cur["open"]:
            return
        rounds[0] += 1
        det = [("sof", cur["sof"]), ("tags", cur["ntag"])]
        if cur["trig"] is not None:
            det.append(("trigger", cur["trig"]))
        recorder.begin(rounds[0], cycles=cur["cyc"], detail=det, dropped=cur["drop"])
        for tid, data in cur["rx"].items():
            recorder.rx(tid, data)
        recorder.end()
        cur.update(open=False, sof=0, ntag=0, rx={})

    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        if poll is not None:
            poll()
        if select.select([rd.fd], [], [], poll_dt)[0]:
            try:
                buf += os.read(rd.fd, 8192)
            except OSError:
                pass
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("utf-8", "replace").strip()
            m = RX_RE.search(line)
            if m:
                frames += 1
                sof, ntag = int(m.group(2)), int(m.group(3))
                sof_sum += sof
                cyc = drop = None
                if on_round is not None:
                    cyc, drop = on_round()       # 今フレームの送信内容 + 次を仕込む
                    nw = {c.tag_id: c.data for c in cyc}
                    if nw != want:
                        prev_want = want         # 1 つ前の抽選を覚えておく
                    want = nw                    # 期待値を今回の抽選に更新
                if recorder is not None:
                    trig = int(trigger_count()) if trigger_count is not None else None
                    if cur["open"] and trig != cur["trig"]:
                        flush_round()            # トリガが進んだ = 前の送信機会を確定
                    if not cur["open"]:
                        cur.update(trig=trig, cyc=cyc, drop=drop, open=True)
                    cur["sof"] += sof            # 同じ機会の複数フレームは足し合わせる
                    cur["ntag"] += ntag
                print("  " + line)
                continue
            m = TAG_RE.search(line)
            if m:
                tid, data = int(m.group(1)), int(m.group(2))
                if recorder is not None and cur["open"]:
                    cur["rx"][tid] = data        # 機会がまとまってから書き出す
                if tid in want:
                    pairs.append((want[tid], data))
                if tid not in want:
                    unknown[tid] = unknown.get(tid, 0) + 1
                    mark = "??"
                elif data == want[tid]:
                    hits[tid] += 1
                    mark = "OK"
                elif prev_want.get(tid) == data:
                    hits[tid] += 1               # 1 つ前の送信の復号結果が遅れて届いた
                    mark = "OK(前の抽選)"
                else:
                    wrong[tid] = wrong.get(tid, 0) + 1
                    mark = "NG(want data=%d)" % want[tid]
                print("      %-44s %s" % (line, mark))
    if recorder is not None:
        flush_round()                       # 最後の送信機会を確定
    return frames, sof_sum, hits, wrong, unknown, pairs


# ---------------------------------------------------------------------------
#  ★送信時刻でラウンドを決める（free-run + --tx-engine fast、2026-09-28）
#
#  watch_reader は «[rx] が届いた時点の最新ラウンド» を期待値にする。送信周期 50 ms
#  だと backoff の大きいタグ（発射から ~39 ms 後に送信）の結果は次の発射の後に
#  届くので、別ラウンドの期待値と比べてしまう（1 つ前まで許す救済はあるが、TSV の
#  ラウンド分けは «到着時点» のまま）。
#  ファーム（firmware-NFC05A1-424k-ringbuffer 2026-09-28 版）は [rx]/tag 行に
#  t=<us>（リーダの micros()、preamble 先頭の時刻）を付けるので、
#      ホスト時刻 ≈ a + b · t_reader
#  を当てはめ、各タグを «予定送信時刻 F_k + backoff·TU が最も近いラウンド» に割り当てる。
#  ★data は割り当てに使わない（使うと誤りが «正解に寄せられて» 誤り率が過小に出る）。
#  ★b（時計の比）が要る: リーダは HSI 駆動で AD3/PC と ~0.3 % ずれる（sps 校正の
#    11835 vs 公称 11799 がまさにそれ）。1 秒で 3 ms ずれるので定数オフセットでは足りない。
# ---------------------------------------------------------------------------
T_RE = re.compile(r"\bt=(\d+)")
RESYNC_RE = re.compile(r"\[resync\] why=(\w+) t=(\d+)")


class FireLog:
    """発射の記録（k = 何発目か、ホスト時刻、そのラウンドの cycles）。"""

    def __init__(self, tu_s: float):
        self.tu_s = tu_s                  # 1 TU の実時間 [s]（AD3 の DIO レートから）
        self.rounds = []                  # [(k, t_host, {tid: cycle}, cycles, dropped, shift_tu)]

    def add(self, k: int, t_host: float, cycles, dropped=(), shift_tu: float = 0) -> None:
        """cycles = 実際に送信したもの、dropped = 抽選したが送信しなかったもの。

        shift_tu = FastTx が詰めた起点。cycles の時刻はシミュレータと同じ起点のままなので、
        実際の発射時刻は t_host + (preamble_start_tu - shift_tu) x TU。"""
        self.rounds.append((k, t_host, {c.tag_id: c for c in cycles}, list(cycles),
                            list(dropped), shift_tu))

    def pre_time(self, r, tid: int) -> float | None:
        """ラウンド r で tag tid の preamble が出た予定のホスト時刻。"""
        c = r[2].get(tid)
        return None if c is None else r[1] + (c.preamble_start_tu - r[5]) * self.tu_s


class ClockMap:
    """リーダ時刻 [us] -> ホスト時刻 [s] の一次式。確定した対応点で窓付き最小二乗。"""

    WINDOW = 400          # 当てはめに使う直近の対応点
    TOL_S = 0.0025        # 割り当てを認める残差 [s]（周期 50 ms に対して十分小さい）
    # ★時計の対応は «割り当てに成功した受信» でしか更新されないので、一度 TOL_S の外へ
    #   ずれると二度と戻らない（2026-10-19 実測: ラウンド 8862 以降 +45.5 ms のまま全滅）。
    #   tag 行がこの件数続けて割り当てに失敗したら、bootstrap からやり直す。
    RELOCK_N = 12

    def __init__(self):
        self.pts = []                     # [(t_reader_s, t_host_s)]
        self.a = None                     # host = a + b * reader
        self.b = 1.0
        self._wrap = 0
        self._last = None

    def unwrap(self, t_us: int) -> float:
        """micros() は 2^32 us（71 分）で一周するので連続値に直す。"""
        if self._last is not None and t_us < self._last - (1 << 31):
            self._wrap += 1 << 32
        self._last = t_us
        return (t_us + self._wrap) * 1e-6

    def ready(self) -> bool:
        return self.a is not None

    def save(self):
        return (list(self.pts), self.a, self.b)

    def restore(self, state) -> None:
        self.pts, self.a, self.b = list(state[0]), state[1], state[2]

    def reset(self) -> None:
        """オフセットと傾きを捨てる（micros() の巻き戻り補正 unwrap の状態は残す）。"""
        self.pts, self.a, self.b = [], None, 1.0

    def host(self, tr: float) -> float:
        return self.a + self.b * tr

    def bootstrap(self, dets, fire_log: "FireLog") -> bool:
        """最初の数十件で «ホスト時刻 - リーダ時刻» の最頻値を探す。

        正しい組合せ（タグ x その送信ラウンド）は同じオフセットに ~1 ms 以内で揃い、
        間違った組合せ（別ラウンド）は周期ずつバラける。幅 3 ms の窓に最も多く入る
        位置を採り、その中央値をオフセットにする。"""
        latest = fire_log.rounds[-1][1] if fire_log.rounds else 0.0
        offs = []
        for tid, tr in dets:
            for r in fire_log.rounds:
                if r[1] < latest - 3.0:          # 直近 3 s の発射だけを候補にする
                    continue
                p = fire_log.pre_time(r, tid)
                if p is not None:
                    offs.append(p - tr)
        if len(dets) < 8 or not offs:
            return False
        offs.sort()
        best, bi, bj, j = 0, 0, 0, 0
        for i in range(len(offs)):
            while offs[i] - offs[j] > 0.003:
                j += 1
            if i - j + 1 > best:
                best, bi, bj = i - j + 1, i, j
        if best < 0.6 * len(dets):               # 半分以上が揃うまで待つ
            return False
        cl = offs[bj:bi + 1]
        self.a = cl[len(cl) // 2]
        self.b = 1.0
        return True

    def accept(self, tr: float, th: float) -> None:
        self.pts.append((tr, th))
        if len(self.pts) > self.WINDOW:
            del self.pts[0]
        n = len(self.pts)
        if n >= 20 and self.pts[-1][0] - self.pts[0][0] > 0.5:
            x = np.array([p[0] for p in self.pts]); y = np.array([p[1] for p in self.pts])
            xm, ym = x.mean(), y.mean()
            self.b = float(((x - xm) * (y - ym)).sum() / ((x - xm) ** 2).sum())
            self.a = float(ym - self.b * xm)
        else:                                    # 点が少ないうちはオフセットだけ追う
            self.a += 0.2 * (th - self.host(tr))


def watch_reader_timed(rd, seconds: float, poll, fire_log: FireLog, recorder,
                       period_s: float, poll_dt: float = 0.002,
                       target_packets: int | None = None):
    """watch_reader の «送信時刻で割り当てる» 版。戻り値も同じ形 + 付帯情報。

    target_packets を渡すと、«集計に使えるパケット»（時計が合っていたラウンドで実際に
    送信したタグの数。tx_dropped と sync=0 のラウンドは数えない）がその数に達した
    ラウンドで TSV を打ち切って終わる（seconds は上限として残る）。"""
    cm = ClockMap()
    hits, wrong, unknown = {}, {}, {}
    frames = sof_sum = 0
    pairs = []
    pending = []                  # 時計が決まる前の行: ("seg", sof, ntag, tr) / ("tag", tid, data, tr, line)
    per_round = {}                # k -> {"sof", "ntag", "rx": {tid: data}}
    flushed = [0]                 # ここまでの k は TSV に書き出し済み
    unattributed = [0]
    unsynced_rounds = [0]
    buf = b""
    boot = []                     # (tid, tr) 時計合わせ用
    resyncs = {}                  # why -> 回数（リーダが受信 FE を入れ直した理由別）
    # 時計が外れたときの立て直し。streak = 続けて割り当てに失敗した (tid, tr)、
    # since = その最初のホスト時刻。unsynced = 時計が外れていたホスト時刻の区間で、
    # この間に撃ったラウンドは TSV の start_signal に sync=0 を付ける（受信が割り当て
    # られず rx_miss に見えるだけなので、集計から外すため。集計の際はこれらのラウンドを除外する）。
    lost = {"streak": [], "since": None}
    unsynced = []
    relocks = [0]
    valid_packets = [0]           # 集計に使えるパケット数（target_packets 用）
    LOST_MARGIN_S = 0.5           # 最初の失敗より前に撃ったラウンドも、この幅だけ遡って外す

    def relock():
        old = cm.save()
        cm.reset()
        if cm.bootstrap(lost["streak"], fire_log):
            relocks[0] += 1
            now = time.monotonic()
            unsynced.append((lost["since"] - LOST_MARGIN_S, now))
            print("  * 時計を合わせ直した（%d 回目）: ホスト - リーダ = %.4f s（%+.1f ms 動いた）"
                  % (relocks[0], cm.a, (cm.a - (old[1] or cm.a)) * 1e3))
            lost["streak"].clear()
            lost["since"] = None
        else:
            cm.restore(old)
            del lost["streak"][:-cm.RELOCK_N]   # 直近だけ残して次の失敗でまた試す

    def round_unsynced(t_fire: float) -> bool:
        if any(t0 <= t_fire < t1 for t0, t1 in unsynced):
            return True
        return (lost["since"] is not None and len(lost["streak"]) >= 3
                and t_fire >= lost["since"] - LOST_MARGIN_S)

    def round_of_time(th: float):
        """ホスト時刻 th を含む送信区間 [F_k, F_k+1) のラウンド。"""
        cand = None
        for r in fire_log.rounds:
            if r[1] <= th + 0.001:
                cand = r
            else:
                break
        return cand

    def handle(ev):
        nonlocal frames, sof_sum
        if ev[0] == "seg":
            _, sof, ntag, tr = ev
            r = round_of_time(cm.host(tr)) if tr is not None else None
            if r is not None:
                d = per_round.setdefault(r[0], {"sof": 0, "ntag": 0, "rx": {}})
                d["sof"] += sof; d["ntag"] += ntag
            return
        _, tid, data, tr, line = ev
        best = None
        th = cm.host(tr)
        for r in fire_log.rounds[-200:]:
            p = fire_log.pre_time(r, tid)
            if p is None:
                continue
            e = th - p
            if best is None or abs(e) < abs(best[1]):
                best = (r, e, p)
        if best is None or abs(best[1]) > cm.TOL_S:
            if best is None and tid not in {t for r in fire_log.rounds[-5:] for t in r[2]}:
                unknown[tid] = unknown.get(tid, 0) + 1
                mark = "??"
            else:
                unattributed[0] += 1
                mark = "?(送信時刻が合わない %+.1f ms)" % (best[1] * 1e3 if best else 0.0)
                lost["streak"].append((tid, tr))
                if lost["since"] is None:
                    lost["since"] = time.monotonic()
            print("      %-58s %s" % (line, mark))
            if len(lost["streak"]) >= cm.RELOCK_N:
                relock()
            return
        r, e, p = best
        lost["streak"].clear()
        lost["since"] = None
        cm.accept(tr, p)
        want = r[2][tid].data
        pairs.append((want, data))
        d = per_round.setdefault(r[0], {"sof": 0, "ntag": 0, "rx": {}})
        d["rx"][tid] = data
        if data == want:
            hits[tid] = hits.get(tid, 0) + 1
            mark = "OK  (round %d, %+.2f ms)" % (r[0], e * 1e3)
        else:
            wrong[tid] = wrong.get(tid, 0) + 1
            mark = "NG(want data=%d, round %d)" % (want, r[0])
        print("      %-58s %s" % (line, mark))

    def flush(upto_time: float):
        """発射から十分経ったラウンドを順に TSV へ（受信ゼロのラウンドも必ず書く）。"""
        if recorder is None:
            return
        for r in fire_log.rounds:
            k = r[0]
            if k <= flushed[0]:
                continue
            if target_packets and valid_packets[0] >= target_packets:
                break                 # 目標に達した後のラウンドは書かない
            # 1.0 s 待つのは、時計が外れたことに気づく（streak >= 3）前に書かないため
            if r[1] + period_s + 1.0 > upto_time:
                break
            d = per_round.pop(k, {"sof": 0, "ntag": 0, "rx": {}})
            detail = [("sof", d["sof"]), ("tags", d["ntag"]), ("trigger", k)]
            if round_unsynced(r[1]):
                detail.append(("sync", 0))
                unsynced_rounds[0] += 1
            else:
                valid_packets[0] += len(r[3])
            recorder.begin(k, cycles=r[3], dropped=r[4], detail=detail)
            for tid, data in d["rx"].items():
                recorder.rx(tid, data)
            recorder.end()
            flushed[0] = k

    t0 = time.monotonic()
    last_report = [t0]
    while True:
        now = time.monotonic()
        if now - t0 >= seconds:
            break
        if target_packets and valid_packets[0] >= target_packets:
            print("  * 目標パケット数に到達: %d / %d（%.0f s）"
                  % (valid_packets[0], target_packets, now - t0))
            break
        if target_packets and now - last_report[0] >= 600:
            last_report[0] = now
            print("  * 進捗: %d / %d パケット（%.1f %%, %.0f s）"
                  % (valid_packets[0], target_packets,
                     100.0 * valid_packets[0] / target_packets, now - t0))
        if poll is not None:
            poll()
        if select.select([rd.fd], [], [], poll_dt)[0]:
            try:
                buf += os.read(rd.fd, 8192)
            except OSError:
                pass
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("utf-8", "replace").strip()
            m = RESYNC_RE.search(line)
            if m:
                resyncs[m.group(1)] = resyncs.get(m.group(1), 0) + 1
                print("  " + line)
                continue
            m = RX_RE.search(line)
            if m:
                frames += 1
                sof, ntag = int(m.group(2)), int(m.group(3))
                sof_sum += sof
                mt = T_RE.search(line)
                tr = cm.unwrap(int(mt.group(1))) if mt else None
                print("  " + line)
                ev = ("seg", sof, ntag, tr)
                (handle(ev) if cm.ready() else pending.append(ev))
                continue
            m = TAG_RE.search(line)
            if m:
                mt = T_RE.search(line)
                if not mt:
                    raise SystemExit("リーダの tag 行に t= がありません。firmware-NFC05A1-424k-"
                                     "ringbuffer を 2026-09-28 版に書き直すか --tx-engine legacy を使うこと")
                tid, data = int(m.group(1)), int(m.group(2))
                tr = cm.unwrap(int(mt.group(1)))
                ev = ("tag", tid, data, tr, line)
                if cm.ready():
                    handle(ev)
                else:
                    pending.append(ev); boot.append((tid, tr))
                    if cm.bootstrap(boot, fire_log):
                        print("  * 時計合わせ完了（%d 件）: ホスト - リーダ = %.4f s"
                              % (len(boot), cm.a))
                        for e in pending:
                            handle(e)
                        pending.clear()
        flush(now)
    # 終了: 残りを処理して、発射済みのラウンドを全部書く
    for e in pending:
        if cm.ready():
            handle(e)
    flush(float("inf"))
    info = {"unattributed": unattributed[0], "clock_ratio": cm.b,
            "rounds": len(fire_log.rounds), "resyncs": resyncs,
            "relocks": relocks[0], "unsynced_rounds": unsynced_rounds[0],
            "valid_packets": valid_packets[0], "rounds_written": flushed[0],
            "elapsed_s": time.monotonic() - t0}
    return frames, sof_sum, hits, wrong, unknown, pairs, info


def mode_verify(a):
    """★検波もトリガも使わずに「リーダがタグのデータを復調できるか」だけを確かめる。

    タグ側は DIO をノンストップでループ再生する（trigger 無し / repeat 無限）。
    リーダの捕捉窓 13.1 ms にはパターンが何周も入るので、両者のタイミングが
    合っているかに関係なく、変調が届いてさえいれば必ず復号できる。
      復号できた  -> 復調系（変調深度・sps・Manchester・間隔）は健全。
                     残る変数は検波トリガのタイミングだけ、と確定できる。
      復号できない-> タイミングではなく「変調が届いていない or 設定違い」。
                     sof=0 なら変調深度（pim_tune.py で測る）、sof>0 で tags=0 なら
                     sps/mult か衝突を疑う。
    """
    ad, h, tags, bitmap, div, rate, nbits = _prepare(a)
    rd = None
    try:
        run_s = program_digitalout(h, bitmap, div, rate, trigsrcNone, a.sps_tu_used)
        print("---- フリーラン開始 ----")
        print("  トリガ無しで %.3f ms のパターンをループ再生（捕捉窓 13.1 ms に約 %.1f 周）"
              % (run_s * 1e3, 13.1e-3 / run_s))
        rd = open_reader(a.port)
        if rd is None:
            print("  リーダのコンソールに繋がらないので、送信だけ続ける。Ctrl-C で終了。")
            while True:
                time.sleep(0.2)
        print("  reader port : %s" % rd.port)
        # ★--reader-sps は link だけでなく verify でも効かせる。'sps auto' は捕捉レートを
        #   公称 40 MHz として計算するので 0.4 % ずれ、data が間隔に比例して +1 TU ずれる
        #   （PLAN.md 実測: 真値 250 -> 251）。校正値を渡せるようにしておく。
        sps_cmd = "sps %d" % a.reader_sps if a.reader_sps else "sps auto"
        rd.cmd(sps_cmd, 0.4)
        send_rxconf(rd, a)
        send_agc(rd, a)
        send_reader_rate(rd, a)
        send_cycle_timing(rd, a)
        if not a.free_run:
            rd.cmd("auto on", 0.4)
        print("  reader      : %s + %s"
              % (sps_cmd, "field ON 固定 (free-run)" if a.free_run else "auto on"))
        tsv, rec = open_tsv(a, tags,
                            window_tu=pim_tsvlog.capture_window_tu(
                                sps=(a.reader_sps / 1000.0) if a.reader_sps else None))
        print()
        frames = sof_sum = 0
        hits, wrong, unknown = {}, {}, {}
        try:
            frames, sof_sum, hits, wrong, unknown, _pairs = watch_reader(rd, tags, a.seconds,
                                                                 recorder=rec)
        finally:
            if tsv is not None:
                # ★実験«後»の値を残す（verify は検波トリガを使わないので trigger は無し）
                pim_tsvlog.append_meta(tsv.path, {
                    "seconds": "%.0f" % a.seconds, "frames": frames,
                    "sof_sum": sof_sum, "matched": sum(hits.values()),
                    "wrong": sum(wrong.values()),
                }, header="results (free-run verify)")
                tsv.close()

        print("\n---- 結果（%.0f 秒 / %d フレーム） ----" % (a.seconds, frames))
        if frames == 0:
            print("  リーダから [rx #n] が出ていない。'auto on' が回っているか確認すること。")
            return
        print("  平均 sof/frame : %.2f （期待値 = タグ数 x 2 = %d）"
              % (sof_sum / frames, 2 * len(tags)))
        ok_all = True
        for t in sorted(tags, key=lambda x: x.tag_id):
            n_ok, n_ng = hits[t.tag_id], wrong.get(t.tag_id, 0)
            print("  tag_id=%3d data=%3d (DIO%s): 一致 %d / 誤り %d / %d フレーム  %s"
                  % (t.tag_id, t.data, t.label, n_ok, n_ng, frames,
                     "PASS" if n_ok else "FAIL"))
            ok_all = ok_all and n_ok > 0
        for tid, n in sorted(unknown.items()):
            print("  ! 送っていない tag_id=%d が %d 回出た（ノイズ or ID 誤り）" % (tid, n))
        print()
        if ok_all:
            print("  => PASS: リーダはタグのデータを復調できている。")
            print("     残る課題は検波トリガのタイミングだけ（run モードで確認）。")
        elif sof_sum == 0:
            print("  => FAIL(sof=0): 変調がリーダに届いていない。復調ロジックの手前の問題。")
            print("     host/pim_tune.py --scan で各タグの変調深度を測り >= 10 LSB にすること。")
        else:
            print("  => FAIL(sof>0, 復号不能): 変調は届いているが解釈できていない。")
            print("     sps（リーダの 'sps <n>'）と --mult、タグ同士の衝突を疑う。")
    except KeyboardInterrupt:
        print("\n  stopped.")
    finally:
        dwf.FDwfDigitalOutReset(h)     # 全 DIO を Low に戻す
        if rd is not None:
            rd.close()
        ad.close()


# ---------------------------------------------------------------------------
#  link: 検波トリガ → タグ送信 → リーダ復調 を 1 本で通しで確認する
# ---------------------------------------------------------------------------
def parse_sweep(text: str | None) -> list[int]:
    """--sweep "0,128,256" -> [0,128,256]。全タグの backoff にこの値を足して試す。"""
    if not text:
        return [0]
    try:
        return [int(x, 0) for x in text.replace(" ", "").split(",") if x != ""]
    except ValueError:
        raise SystemExit("bad --sweep %r (want 整数のカンマ区切り, 例 0,128,256)" % text)


def mode_link(a):
    """★本命: 検波回路 → タグ送信 → リーダ復調 の全経路を 1 本のコマンドで確認する。

    run と verify を合体させたもの。何が違うか:
      verify : タグはフリーラン（常時送信）。検波もトリガも使わない。
               「リーダが復調できるか」だけを見る。
      run    : 検波トリガで撃つが、リーダの結果を見ていない（撃ちっぱなし）。
      link   : 検波トリガで撃ち、同時にリーダの UART を読んで送信値と突き合わせる。
               磁界 ON の検出・送信タイミング・復調が全部揃って初めて PASS になる。

    タイミングの鎖（ここが唯一の新しい変数）:
      リーダ field ON --> 検波の立ち上がり --> (--min-width の幅判定) --> DIO 発射
                     --> backoff TU 待ち --> preamble/postamble
      リーダ field ON --> Transparent 突入 --> 捕捉窓 13.1 ms 開始
    タグのパルスがこの窓の中に入っていないと sof=0 になる。窓の手前で撃ってしまう
    のが典型的な失敗なので、うまくいかないときは --sweep で backoff を振ること。

    読み方:
      trigger=0             -> 検波トリガが発火していない（--level/--hyst/--vrange/
                               --min-width）。detector モードで実測値を取り直す。
      trigger>0, sof=0      -> 撃ってはいるが捕捉窓に入っていない or 変調が弱い。
                               まず verify で復調系の健全性を切り分ける。
      sof>0, data 一致      -> PASS。疑似タグとして成立している。
    """
    tags = parse_tags(a.tags)
    validate(tags, a.mult, getattr(a, "random_backoff", False))
    offsets = parse_sweep(a.sweep)
    # ★ランダム backoff（論文の 2^14 TU）は既定構成のバッファ 1024 TU に入らない。
    #   main() が a.dio_config=4 を立てるので、それを必ず反映して開くこと。
    ad = AD(config=a.dio_config); h = ad.h
    rd = None
    tsv = None
    fast = None          # --free-run --tx-engine fast のときの FastTx
    nbuf = 1024
    try:
        fdio = dio_internal_clock(h)
        sps_tu, div, fsc_act, ppm = pick_sps_tu(fdio, a.sps_tu)
        rate = fdio / div
        cap = dio_buffer_bits(h, tags[0].pins[0])

        rd = open_reader(a.port)
        if rd is None:
            raise SystemExit("link はリーダのコンソールが必須（復調結果を読むため）。"
                             "\n  読まずに撃つだけなら run モードを使うこと。")
        print("  reader port : %s" % rd.port)
        # ★sps はリーダの捕捉レート実測値。auto は PCLK2/2 の公称 40MHz を前提にするが、
        #   STM32 の SYSCLK が公称からずれていると間隔がその分だけ比例して伸縮し、
        #   data が長い間隔ほど大きくずれる（実測 2026-08-20: auto=94393 では
        #   真値 250 TU が 251 TU、data=200 が 201。94800 で一致）。
        rd.cmd(("sps %d" % a.reader_sps) if a.reader_sps else "sps auto", 0.4)
        # ★--mult はタグの «間隔の倍率» なので、リーダ側の復号にも同じ値が要る。
        #   送り忘れると「タグは x2 で撃つがリーダは x1 で割る」= data が 2 倍に
        #   復号され、全件 NG になる（2026-09-25 にこれで 376/380 誤りを出した）。
        echo_reader(rd.cmd("mult %d" % a.mult, 0.4))
        send_rxconf(rd, a)
        send_agc(rd, a)
        send_reader_rate(rd, a)
        send_cycle_timing(rd, a)
        if not a.free_run:
            rd.cmd("auto on", 0.4)
        print("  reader      : %s + %s%s%s\n"
              % (("sps %d" % a.reader_sps) if a.reader_sps else "sps auto",
                 "field ON 固定 (free-run)" if a.free_run else "auto on",
                 "" if a.free_run or not a.reader_gap else " + gap %dms" % a.reader_gap,
                 "" if a.free_run or not a.reader_period
                 else " + period %dms" % a.reader_period))

        # ★検波は磁界 OFF の過渡でも 1 回トリガする。そこで撃つと gap の間ずっと
        #   無磁界に送ることになるので、gap ぶん待ってから撃つ（tx_wait_s の注参照）。
        tx_wait = tx_wait_s(a)
        if tx_wait:
            print("  送信待ち     : トリガから %.0f ms 後に撃つ"
                  "（磁界 OFF の過渡で撃っても ON に着地する。--tx-wait 0 で無効）"
                  % (tx_wait * 1e3))

        # ---- ランダム化の準備（--random-data / --random-backoff）----
        randomize = bool(a.random_data or a.random_backoff)
        seed = a.seed
        rng = SimTagRng(seed, a.sim_max_tags, data_dist=a.data_dist,
                        data_mean=a.data_mean, data_std=a.data_std)
        # リーダの捕捉窓 [TU]。校正値 --reader-sps があればそちらが正確。
        window_tu = pim_tsvlog.capture_window_tu(
            sps=(a.reader_sps / 1000.0) if a.reader_sps else None)
        loaded  = {"tags": None,      # いま DIO に載っている抽選内容
                   "dropped": set()}  #   そのうちバッファに収まらず送信しない tag_id
        last_tx = {"cycles": None,    # 直近のトリガで «実際に送信した» 内容
                   "dropped": []}     #   抽選したが送信しなかった内容（TSV の tx_dropped）
        def mark_sent():
            """いま載っている抽選を «直近の送信» として確定する（撃った直後に呼ぶ）。"""
            last_tx["cycles"], last_tx["dropped"] = split_sent(
                loaded["tags"], loaded["dropped"], a.mult)
        if randomize:
            print("  ランダム化   : data=%s%s backoff=%s (max %d TU) seed=%d"
                  % (a.random_data,
                     "(gauss mean=%g std=%g)" % (a.data_mean, a.data_std)
                     if a.random_data and a.data_dist == "gauss" else "",
                     a.random_backoff, a.backoff_max, seed))
            print("  捕捉窓       : %.0f TU（これを超えた抽選は out_of_window として記録）"
                  % window_tu)
            # ★毎ラウンド抽選し直す場合だけ、AD3 への波形載せ替え（実測 ~84 ms）が
            #   次の磁界 ON までに終わっている必要がある。固定 data ならこの制約は無い。
            #   free-run + --tx-engine fast は載せ替え ~5 ms なのでこの制約は無い。
            if a.reader_period and a.reader_period < 200 and \
                    not (a.free_run and a.tx_engine == "fast"):
                print("  ! 推奨: 乱数を引くなら --reader-period 200 以上"
                      "（波形の載せ替えに約 84 ms 掛かるため。現在 %d ms）"
                      % a.reader_period)

        n_trig = 0
        n_dropped = 0        # バッファに入らず «無送信» になった抽選の数
        tx_busy = {"until": 0.0, "run_s": 0.0, "trig_t": 0.0}  # 送信が終わる時刻
        rearm = {"at": 0.0}    # 検波を武装し直す時刻（0 = すぐ武装してよい）
        def poll():
            """UART を待つ合間に検波トリガの後始末をする。AnalogIn は 1 トリガで
            Done に落ちるので、毎回アームし直さないと次の磁界 ON を拾えない。

            ★2026-09-15 実測の急所: 再武装（FDwfDigitalOutConfigure / 再プログラム）は
              «送信が終わってから» 行うこと。AnalogIn は取込 1 ms + poll 周期 2 ms で
              Done に落ちるので、そこで即 DigitalOut を触ると、まだ backoff のアイドルを
              再生している最中の波形を叩き切ってしまう。実測では backoff が 2-4 ms
              (約 900-1800 TU) を超えた瞬間にパケットが 1 発も出なくなった
              （sweep: 128TU->59frame, 1920TU->9frame, 3968TU->0frame）。
              送信中は AnalogIn も再武装しないので、1 磁界サイクル 2 発の無駄撃ちも
              自然に潰れる（--holdoff と同じ効果）。"""
            nonlocal n_trig, n_dropped
            now = time.monotonic()
            if tx_busy["until"]:
                if now < tx_busy["until"]:
                    return                      # ★送信中: DigitalOut に触らない
                tx_busy["until"] = 0.0
                if randomize:
                    # ★ラウンドごとに data/backoff を引き直す場合の載せ替えは «ここ»
                    #   （＝送信完了後）。この直後はリーダが捕捉+復号している数百 ms の
                    #   空きがあり、再プログラム(実測 84 ms)が間に合う。
                    #   [rx #n] が届いてから載せ替えると、次の磁界 ON まで gap(既定
                    #   20 ms)しか無く間に合わない。
                    nxt = draw_tags(tags, rng, a.random_data, a.random_backoff,
                                    a.backoff_max)
                    bm, _tu, _nb, dr = build_bits(nxt, sps_tu, a.mult,
                                                  cap // sps_tu if cap else None)
                    # ★落ちたタグは «この回まるごと無送信» になる。黙って捨てると
                    #   「撃ったのに届かなかった」と見分けが付かないので必ず知らせる。
                    if dr:
                        n_dropped += len(dr)
                        print("  ! backoff がバッファ上限 %d TU を超過 -> この回は無送信"
                              "（tag_id=%s, backoff=%s TU）"
                              % (cap // sps_tu if cap else 0,
                                 ",".join(str(x) for x in dr),
                                 ",".join(str(t.backoff_tu) for t in nxt
                                          if t.tag_id in dr)))
                    tx_busy["run_s"] = tx_wait + program_digitalout(
                        h, bm, div, rate, trigsrcDetectorAnalogIn, sps_tu, repeat=1,
                        wait_s=tx_wait)
                    loaded["tags"], loaded["dropped"] = nxt, set(dr)
                else:
                    # ★DigitalOut は repeat=1 で 1 送信ごとに Done に落ちる。ここで
                    #   武装し直すことで「1 磁界イベント = 1 送信」を保証する。
                    dwf.FDwfDigitalOutConfigure(h, ctypes.c_int(1))
                # ★再武装は «次の磁界サイクルが来る直前» まで待つ。検波は磁界 OFF の
                #   過渡でも 1 回トリガするので、すぐ武装すると 1 サイクルに 2 回撃つ
                #   ことになる。2 回撃つと、その間にホストが次の抽選を載せ替えるため
                #   «送った値» と «ホストの期待値» が 1〜2 ラウンドずれ、誤り率が 3 割に
                #   見える（2026-09-22。TSV は送信機会単位なので影響なし）。
                #   period の 80 % まで待てば、次に拾うのは必ず本物の磁界 ON になる。
                rearm["at"] = tx_busy["trig_t"] + 0.8 * (a.reader_period or 200) / 1000.0
                return
            if rearm["at"]:
                if now < rearm["at"]:
                    return                      # ★次のサイクルまで武装しない
                rearm["at"] = 0.0
                rearm_detector(h)
                return
            if analogin_done(h):
                n_trig += 1
                v = read_ch0(h, nbuf)
                if a.verbose:
                    print("  * detector trigger #%-4d  mean=%+.3f V  max=%+.3f V"
                          % (n_trig, v.mean(), v.max()))
                # 「このトリガで実際に送信された内容」を確定してから送信完了を待つ。
                mark_sent()
                tx_busy["trig_t"] = now
                tx_busy["until"] = now + tx_busy["run_s"] + 0.002

        # ------------------------------------------------------------------
        #  ★--free-run: 検波回路を使わない版の poll。
        #
        #  検波版との違いは «何を合図に撃つか» だけ:
        #      検波版   : 磁界 OFF->ON の立ち上がり（AnalogIn のトリガ）
        #      free-run : ホストの時計（period_ms ごと）
        #  リーダはリングバッファで «連続» 捕捉しているので、タグの位相が磁界サイクルと
        #  揃っている必要はない。だから検波も磁界の ON/OFF も要らなくなる。
        #  1 ラウンド = «ここで 1 回撃つこと» なので、TSV の round 境界（trigger_count）
        #  の意味は検波版と完全に同じまま。
        # ------------------------------------------------------------------
        period_s = (a.reader_period or 200) / 1000.0
        next_tx = {"at": 0.0}
        # ★--tx-engine fast（既定）: FastTx で «撃つ» と «載せ替え» を分ける。
        #     周期の区切り : 武装済みの内容を PC トリガで発射（載せ替え時間が乗らない）
        #     再生が終わったら : 次の抽選を載せて武装（~5 ms。周期の残りで間に合う）
        #   legacy は従来どおり «区切りで載せ替え = 発射»（載せ替え ~84 ms が毎回乗る）。
        if a.free_run and a.tx_engine == "fast":
            fast = FastTx(h, [p for t in tags for p in t.pins], div, rate, sps_tu, a.mult)
        n_late = 0           # 周期の区切りに載せ替えが間に合わず、撃てなかった回数
        fire = {"log": None}  # FireLog（送信時刻でラウンドを決める。オフセットごとに作り直す）
        def load_next():
            """次のラウンドを抽選して FastTx に載せる（武装まで）。"""
            nonlocal n_dropped
            nxt = (draw_tags(tags, rng, a.random_data, a.random_backoff, a.backoff_max)
                   if randomize else loaded["tags"])
            dr = fast.load(nxt)
            if dr:
                n_dropped += len(dr)
                print("  ! backoff がバッファ上限 %d TU を超過 -> この回は無送信"
                      "（tag_id=%s）" % (fast.cap // sps_tu, ",".join(str(x) for x in dr)))
            loaded["tags"], loaded["dropped"] = nxt, set(dr)
        def poll_fast():
            nonlocal n_trig, n_late
            now = time.monotonic()
            if not fast.armed and not fast.busy():
                if randomize or n_trig == 0:
                    load_next()
                else:
                    fast.rearm()
            if now < next_tx["at"]:
                return
            next_tx["at"] = max(next_tx["at"] + period_s, now + 0.001)
            if not fast.armed:
                # 前の再生がまだ終わっていない = 1 サイクルが周期より長い。撃つと切れる。
                n_late += 1
                if a.verbose or n_late <= 3:
                    print("  ! 周期 %.0f ms の区切りで未武装（前の再生中）-> この回は送信しない"
                          % (period_s * 1e3))
                return
            fast.fire()
            n_trig += 1
            mark_sent()
            fire["log"].add(n_trig, fast.fired_at, last_tx["cycles"], last_tx["dropped"],
                                fast.shift_tu)
            if a.verbose:
                print("  * free-run tx #%-4d (t=%.3f s, 1 サイクル %.1f ms)"
                      % (n_trig, now, fast.run_s * 1e3))
        def poll_free():
            nonlocal n_trig, n_dropped
            if fast is not None:
                return poll_fast()
            now = time.monotonic()
            if now < next_tx["at"]:
                return
            # 次の発射時刻は «周期の格子» に乗せ直す（載せ替えの遅れを引きずらない）。
            next_tx["at"] = max(next_tx["at"] + period_s, now + 0.001)
            if randomize:
                # ★載せ替え（実測 ~84 ms）がそのまま «撃つ» ことになる: trigsrcNone
                #   なので FDwfDigitalOutConfigure(h,1) で即再生が始まる。
                nxt = draw_tags(tags, rng, a.random_data, a.random_backoff,
                                a.backoff_max)
                bm, _tu, _nb, dr = build_bits(nxt, sps_tu, a.mult,
                                              cap // sps_tu if cap else None)
                if dr:
                    n_dropped += len(dr)
                    print("  ! backoff がバッファ上限 %d TU を超過 -> この回は無送信"
                          "（tag_id=%s）"
                          % (cap // sps_tu if cap else 0,
                             ",".join(str(x) for x in dr)))
                program_digitalout(h, bm, div, rate, trigsrcNone, sps_tu, repeat=1)
                loaded["tags"], loaded["dropped"] = nxt, set(dr)
            else:
                dwf.FDwfDigitalOutConfigure(h, ctypes.c_int(1))   # 1 回ぶん再生
            n_trig += 1
            mark_sent()
            if a.verbose:
                print("  * free-run tx #%-4d (t=%.3f s)" % (n_trig, now))

        tsv, _ = open_tsv(a, tags, window_tu=window_tu,
                          seed=seed if randomize else None)
        rounds_done = 0
        table = []
        for off in offsets:
            shifted = [TagSpec(t.pins, t.tag_id, t.data, t.backoff_tu + off) for t in tags]
            if randomize:
                shifted = draw_tags(tags, rng, a.random_data, a.random_backoff,
                                    a.backoff_max)
            bitmap, total_tu, nbits, dropped = build_bits(shifted, sps_tu, a.mult,
                                                          cap // sps_tu if cap else None)
            loaded["tags"] = shifted          # 最初の 1 ラウンド分は今ここで載る
            loaded["dropped"] = set(dropped)
            mark_sent()
            if dropped and fast is None:       # FastTx は下で自分の判定で数える
                n_dropped += len(dropped)
                print("  ! 初回の抽選がバッファ上限 %d TU を超過 -> 無送信"
                      "（tag_id=%s）" % (cap // sps_tu if cap else 0,
                                        ",".join(str(x) for x in dropped)))
            if len(offsets) == 1:
                report_plan(shifted, sps_tu, div, fsc_act, ppm, total_tu, nbits, rate,
                            a.mult, cap, reader_fs=reader_fs_hz(a))
            else:
                print("==== backoff +%d TU  (合計 %d TU = %d bit / 上限 %d bit) ===="
                      % (off, total_tu, nbits, cap))
            if cap and nbits > cap:
                print("  ! バッファ上限超過。この backoff は飛ばす"
                      "（上限 %d TU）。\n" % (cap // sps_tu))
                table.append((off, None, 0, 0, 0, 0))
                continue
            if fast is not None:
                # 初回も «載せる → 撃つ» の順。以降は poll_fast() が回す。
                # ★FastTx は起点を詰めるので build_bits と判定が違う（TAIL も要る一方、
                #   最早タグまでの遅延は要らない）。«送ったか» と件数は FastTx の戻り値で決め直す。
                loaded["dropped"] = set(fast.load(shifted))
                n_dropped += len(loaded["dropped"])
                if loaded["dropped"]:
                    print("  ! 初回の抽選がバッファに収まらない -> 無送信（tag_id=%s）"
                          % ",".join(str(x) for x in sorted(loaded["dropped"])))
                mark_sent()
                fire["log"] = FireLog(tu_s=sps_tu / rate)
                fast.fire()
                n_trig = 1
                fire["log"].add(1, fast.fired_at, last_tx["cycles"], last_tx["dropped"],
                                fast.shift_tu)
                next_tx["at"] = fast.fired_at + period_s
                # 乱数 backoff だと 1 サイクルは毎回変わる。周期に効くのは最悪値
                # = バッファ全長 + 載せ替え（実測最大 5.6 ms）。
                run_s = (fast.cap / rate if randomize else fast.run_s) + 0.006
            elif a.free_run:
                # trigsrcNone = Configure(1) した瞬間に 1 回再生される。
                # つまり «この 1 行目が 1 発目の送信» なので n_trig は 1 から始める。
                run_s = program_digitalout(h, bitmap, div, rate, trigsrcNone,
                                           sps_tu, repeat=1)
                n_trig = 1
                next_tx["at"] = time.monotonic() + period_s
            else:
                run_s = tx_wait + program_digitalout(
                    h, bitmap, div, rate, trigsrcDetectorAnalogIn, sps_tu, repeat=1,
                    wait_s=tx_wait)
                tx_busy["until"], tx_busy["run_s"] = 0.0, run_s  # poll() の送信待ちを初期化
                arm_detector(h, a.level, a.vrange, a.hyst, a.det_fs, nbuf,
                             a.min_width, a.holdoff)
                n_trig = 0
            print("---- 待機中 (%.0f s) ----" % a.seconds)
            if a.free_run:
                print("  送信トリガ: ホストのタイマ（%d ms ごと）。検波回路も磁界サイクルも使わない"
                      % (a.reader_period or 200))
                print("  送信エンジン: %s" % (
                    "fast（パケットのみ載せ替え + CounterInit backoff + PC トリガ発射）"
                    if fast is not None else "legacy（毎回フルバッファを載せ替え = 発射）"))
                # ★タグを増やすと backoff が伸び、1 サイクルが送信間隔を追い越しうる。
                #   そうなると «前の再生が終わる前に» 次を撃つことになり、後ろのタグが
                #   毎回切り落とされる（原因が backoff に見えて分かりにくい）。
                if run_s >= period_s:
                    print("  ! 1 サイクル %.1f ms が送信間隔 %.0f ms 以上あります。"
                          "--reader-period を %.0f ms 以上にするか backoff を詰めること"
                          % (run_s * 1e3, period_s * 1e3,
                             math.ceil(run_s * 1e3 * 1.2 / 10.0) * 10))
            elif a.min_width > 0:
                print("  検波トリガ: Scope 1+ が %.3f V を %.0f us 以上超え続けたら送信"
                      % (a.level, a.min_width * 1e6))
            else:
                print("  検波トリガ: Scope 1+ が %.3f V を上向きに横切ったら送信" % a.level)
            if a.free_run:
                # 連続捕捉なので «固定の捕捉窓» が無い。長さの上限は DigitalOut の
                # バッファだけで、位相がどこでもリーダは拾う。
                print("  1 サイクル : %.3f ms（リーダは連続捕捉なので捕捉窓の制約は無い）"
                      % (run_s * 1e3))
            elif randomize:
                # ランダム backoff では窓より長いパターンが正常（窓外の抽選は
                # out_of_window として記録され、統計の分母から外れる）。
                print("  1 サイクル : %.3f ms（うちリーダが見えるのは先頭 13.1 ms = %.0f TU）"
                      % (run_s * 1e3, window_tu))
            else:
                print("  1 サイクル : %.3f ms（捕捉窓 13.1 ms に収まること）" % (run_s * 1e3))
            rec = None
            if tsv is not None:      # backoff を振っても round 番号は連番のまま繋げる
                rec = pim_tsvlog.RoundRecorder(tsv, last_tx["cycles"],
                                               round_offset=rounds_done,
                                               window_tu=window_tu,
                                               dropped=last_tx["dropped"])

            # 実際に載せ替えるのは poll()（= 検波トリガ直後）。ここでは
            # 「直近のトリガで送信された内容」を [rx #n] のときに拾うだけ。
            # 余分なトリガ（磁界 OFF 側でも 1 発出るのが正常 = PLAN.md ★0）が
            # 来ても、常に «最後に送信した内容» を使うのでズレない。
            def next_round():
                return last_tx["cycles"], last_tx["dropped"]

            timed = None
            if fast is not None:
                # ★送信時刻で割り当てる（周期より遅れて届いても取り違えない）
                frames, sof_sum, hits, wrong, unknown, pairs, timed = watch_reader_timed(
                    rd, a.seconds, poll_free, fire["log"], rec, period_s,
                    target_packets=a.target_packets)
                rounds_done += n_trig
                print("  送信時刻での割り当て: 発射 %d 回 / 時刻が合わず捨てた %d 件 / "
                      "時計比 リーダ:PC = %.5f"
                      % (timed["rounds"], timed["unattributed"], timed["clock_ratio"]))
                print("  時計の合わせ直し: %d 回 / 時計が外れていたラウンド %d（TSV に sync=0、"
                      "集計からは外すこと）" % (timed["relocks"], timed["unsynced_rounds"]))
                print("  リーダの受信 FE 入れ直し: %s" % (", ".join(
                    "%s=%d" % kv for kv in sorted(timed["resyncs"].items())) or "なし"))
            else:
                frames, sof_sum, hits, wrong, unknown, pairs = watch_reader(
                    rd, shifted, a.seconds, poll=(poll_free if a.free_run else poll),
                    poll_dt=0.002, recorder=rec,
                    on_round=next_round if randomize else None,
                    trigger_count=lambda: n_trig)
                rounds_done += frames
            reader_stat(rd)          # 「聞いていない」のか「届いていない」のかの切り分け
            n_ok  = sum(hits.values())
            n_ng  = sum(wrong.values())
            table.append((off, n_trig, frames, sof_sum, n_ok, n_ng))
            print("  -> trigger=%d  frame=%d  sof/frame=%.2f  一致=%d  誤り=%d"
                  % (n_trig, frames, (sof_sum / frames if frames else 0.0), n_ok, n_ng))
            if n_late:
                print("  ! 周期の区切りで撃てなかった回 = %d（1 サイクルが周期より長い）" % n_late)
            # ★sps 校正の当てはめ。誤り（NG）が «data に比例する off-by-1» なら、
            #   ここが推奨値を出す。個体ごとに違う値なので表よりこちらを信じること。
            used = a.reader_sps or READER_RATE_SPS.get(a.reader_rate)
            fit = fit_sps_milli(pairs, used, a.mult) if used else None
            if fit:
                rec, eps, sigma, npt = fit
                floor = err_floor(sigma, a.mult)
                # ★«直すべきか» は sps の差ではなく «一番大きい data でどれだけ TU が
                #   ずれるか» で決める。0.25 TU（±1 に飛ぶ 0.5 TU の半分）を超えたら助言。
                bias_max = abs(eps) * (255.0 * a.mult + C_INTERVAL_TU) / a.mult
                print("     sps 校正: 推奨 %d (使用 %d, ずれ %+.3f%% = data 255 で %.2f TU, %d 点)  %s"
                      % (rec, used, eps * 100.0, bias_max, npt,
                         "OK（このままでよい）" if bias_max < 0.25 else
                         "-> --reader-sps %d（data の off-by-1 が消える）" % rec))
                print("     残ジッタ : %.2f TU -> sps を合わせても data の誤り率は %.1f%% 残る%s"
                      % (sigma, floor * 100.0,
                         "" if a.mult > 1 else
                         "（--mult 2 なら %.2f%% まで落ちる）"
                         % (err_floor(sigma, 2) * 100.0)))
            print()
            for tid, n in sorted(unknown.items()):
                print("  ! 送っていない tag_id=%d が %d 回出た" % (tid, n))
            # ★実験«後»にしか分からない値をログに残す（ログ単体で切り分け可能にする）
            if tsv is not None:
                meta = {"seconds": "%.0f" % (timed["elapsed_s"] if timed is not None
                                             else a.seconds)}
                if a.free_run:
                    # 検波を使わない = 送信トリガはホストのタイマ。設定値もそれに合わせる。
                    meta["tx_trigger"] = "host-timer(free-run)"
                    meta["tx_period_ms"] = a.reader_period or 200
                    meta["tx_engine"] = "fast" if fast is not None else "legacy"
                    meta["tx_late_skipped"] = n_late
                    if timed is not None:
                        meta["rx_attribution"] = "reader-timestamp"
                        meta["rx_unattributed"] = timed["unattributed"]
                        meta["reader_clock_ratio"] = "%.6f" % timed["clock_ratio"]
                        meta["clock_relocks"] = timed["relocks"]
                        meta["unsynced_rounds"] = timed["unsynced_rounds"]
                        meta["rounds_written"] = timed["rounds_written"]
                        meta["valid_packets"] = timed["valid_packets"]
                        for why, n in sorted(timed["resyncs"].items()):
                            meta["reader_resyncs_%s" % why] = n
                    meta["reader_resync_ms"] = a.reader_resync
                    meta["reader_stuck_max"] = a.reader_stuck
                else:
                    meta.update(tx_trigger="detector",
                                detector_level_v=a.level, detector_hyst_v=a.hyst,
                                detector_holdoff_s=a.holdoff,
                                detector_vrange_v=a.vrange)
                meta.update({
                    "trigger_total": n_trig, "frames": frames, "sof_sum": sof_sum,
                    "matched": n_ok, "wrong": n_ng, "no_tx_dropped": n_dropped,
                })
                pim_tsvlog.append_meta(tsv.path, meta,
                                       header="results (backoff offset +%d TU)" % off)

        # ---- まとめ ----
        print("==== まとめ（%d タグ / 各 %.0f 秒） ====" % (len(tags), a.seconds))
        print("   backoff+   trigger   frame   sof/frame   一致   誤り   判定")
        best = None
        for off, ntr, frames, sof_sum, n_ok, n_ng in table:
            if ntr is None:
                print("   %8d   %s" % (off, "(バッファ超過でスキップ)"))
                continue
            verdict = ("PASS" if n_ok else ("sof=0" if sof_sum == 0 else "復号不能"))
            print("   %8d   %7d   %5d   %9.2f   %4d   %4d   %s"
                  % (off, ntr, frames, (sof_sum / frames if frames else 0.0),
                     n_ok, n_ng, verdict))
            if n_ok and (best is None or n_ok > best[1]):
                best = (off, n_ok)
        print()
        if best is not None:
            print("  => PASS: リーダが復調できた（最良 backoff = +%d TU）。" % best[0])
            if a.free_run:
                print("     タグは %d ms ごとに自走送信し、リーダは磁界 ON 固定のまま"
                      "連続捕捉で読めている（検波回路なし）。" % (a.reader_period or 200))
            else:
                print("     検波回路が磁界 ON を捕まえ、その合図でタグが送り、リーダが読めている。")
        elif all(t[1] in (None, 0) for t in table):
            if a.free_run:
                print("  => FAIL: 1 度も撃てていない（--seconds が周期より短い？）。")
            else:
                print("  => FAIL: 検波トリガが 1 度も発火していない。")
                print("     detector モードで段差を測り直し、--level/--hyst/--vrange を合わせること。")
        elif all((t[3] or 0) == 0 for t in table if t[1]):
            print("  => FAIL(sof=0): 撃ってはいるがリーダに届いていない。")
            if a.free_run:
                # free-run は «窓» が無いのでタイミングは容疑者から外れる。上の
                # reader stat がそのまま切り分けになる。
                print("     上の reader stat を読むこと:")
                print("       listen=0 / blocks が増えない -> リーダが聞いていない")
                print("       max < act_thr               -> 変調が弱くて活動判定に届かない")
                print("         → pim_tune.py --scan で変調深度を測る / リーダで 'thr <n>' を下げる")
                print("       segments>0 なのに [rx] 無し  -> 切り出せているが復号できていない")
                print("     受信 FE 設定も疑うこと: --reader-rxconf 8C / A0 / 20 を試す")
                print("       (A0=PM ch+2MHz LP = best measured, 8C=600kHz LP splits SOF, 20=AM ch)")
            else:
                print("     1) verify モードで復調系が健全か切り分ける（フリーラン）")
                print("     2) 健全なら --sweep で backoff をもっと広く振る")
        else:
            print("  => FAIL: 変調は届いているが復号できていない。sps と --mult を疑う。")
    except KeyboardInterrupt:
        print("\n  stopped.")
    finally:
        if fast is not None:
            fast.close()               # AutoConfigure を 1 に戻す
        dwf.FDwfDigitalOutReset(h)     # 全 DIO を Low に戻す
        if tsv is not None:
            tsv.close()
        if rd is not None:
            rd.close()
        ad.close()


def mode_run(a):
    """本番: 検波出力の立ち上がりでトリガし、延々と繰り返す。
    --free-run なら検波を使わず、ホストのタイマで --reader-period ms ごとに撃つ。"""
    ad, h, tags, bitmap, div, rate, nbits = _prepare(a)
    n = 0
    if a.free_run:
        try:
            run_s = program_digitalout(h, bitmap, div, rate, trigsrcNone,
                                       a.sps_tu_used, repeat=1)
            period_s = (a.reader_period or 200) / 1000.0
            n = 1                                  # Configure(1) で 1 発目が出ている
            nxt = time.monotonic() + period_s
            print("---- free-run 送信中 ----")
            print("  検波回路は使わない。%d ms ごとに 1 サイクル（%.3f ms）を撃つ。"
                  % (a.reader_period or 200, run_s * 1e3))
            print("  リーダ側は磁界 ON 固定 + 連続捕捉（'cycle off'）にしておくこと。")
            print("  Ctrl-C で終了（終了時 DIO は Low に戻る）。\n")
            t0 = time.monotonic()
            while True:
                now = time.monotonic()
                if now >= nxt:
                    nxt = max(nxt + period_s, now + 0.001)
                    dwf.FDwfDigitalOutConfigure(h, ctypes.c_int(1))
                    n += 1
                    print("  [%8.2f s] tx #%d" % (now - t0, n))
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("\n  stopped. tx = %d" % n)
        finally:
            dwf.FDwfDigitalOutReset(h)   # 全 DIO を Low に戻す
            ad.close()
        return
    try:
        tx_wait = tx_wait_s(a)
        run_s = tx_wait + program_digitalout(
            h, bitmap, div, rate, trigsrcDetectorAnalogIn, a.sps_tu_used, repeat=1,
            wait_s=tx_wait)
        if tx_wait:
            print("  送信待ち   : トリガから %.0f ms 待ってから撃つ"
                  "（磁界 OFF の過渡で撃っても ON に着地させる）" % (tx_wait * 1e3))
        nbuf = 1024
        arm_detector(h, a.level, a.vrange, a.hyst, a.det_fs, nbuf, a.min_width, a.holdoff)
        print("---- 待機中 ----")
        if a.min_width > 0:
            print("  検波トリガ: Scope 1+ が %.3f V を超えた状態が %.0f us 以上続いたら全タグが送信"
                  % (a.level, a.min_width * 1e6))
        else:
            print("  検波トリガ: Scope 1+ が %.3f V を上向きに横切ったら全タグが送信" % a.level)
        if a.holdoff > 0:
            print("  ホールドオフ: トリガ後 %.0f ms は次のトリガを無視" % (a.holdoff * 1e3))
        print("  1 サイクル: %.3f ms" % (run_s * 1e3))
        print("  リーダ側で 'auto on' を打つと磁界 ON/OFF が繰り返され、その都度ここが光る。")
        print("  Ctrl-C で終了（終了時 DIO は Low に戻る）。\n")
        t0 = time.monotonic()
        while True:
            if analogin_done(h):
                n += 1
                v = read_ch0(h, nbuf)
                print("  [%8.2f s] trigger #%-5d  detector: mean=%+.3f V  max=%+.3f V"
                      % (time.monotonic() - t0, n, v.mean(), v.max()))
                rearm_detector(h)      # ★フル再設定はしない（AD3 が鳴る + 遅い）
                dwf.FDwfDigitalOutConfigure(h, ctypes.c_int(1))   # 1 トリガ = 1 送信
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n  stopped. triggers = %d" % n)
    finally:
        dwf.FDwfDigitalOutReset(h)     # 全 DIO を Low に戻す
        ad.close()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=("info", "detector", "carrier", "selftest", "verify",
                                    "run", "link"))
    p.add_argument("--tags", help='"dio:id:data[:backoff]" のカンマ区切り。省略時は DEFAULT_TAGS')
    p.add_argument("--mult",   type=int, default=pim.DATA_TU_MULT, help="value->interval 倍率")
    p.add_argument("--sps-tu", type=int, default=None, dest="sps_tu",
                   help="1 TU あたりサンプル数。省略時は誤差最小を自動選択")
    # ★実測(2026-08-18, 最新): 検波出力の段差は field OFF +0.0041V -> ON +0.0276V = 23.5mV。
    #   既定 0.04V は ON レベルより高く、過渡スパイクでしか発火しなかった（誤発火 48回/秒）。
    #   段差は配線・同調で変わるので、必ず `detector` モードで自分の環境を測り、
    #   出てくる推奨値を --level / --hyst に渡すこと。
    p.add_argument("--level",  type=float, default=DETECT_LEVEL_V,
                   help="検波トリガ電圧 [V] (既定 %.3f = DETECT_LEVEL_V)" % DETECT_LEVEL_V)
    p.add_argument("--hyst",   type=float, default=DETECT_HYST_V,
                   help="トリガのヒステリシス [V] (既定 %.3f = DETECT_HYST_V)" % DETECT_HYST_V)
    # ★実測(2026-08-18): 磁界 ON/OFF の過渡スパイク（幅 <1us, 振幅 0.1-0.27V）が
    #   包絡線の段差 23mV の 10 倍あり、素のエッジトリガでは 100% 誤発火する。
    #   本物の field ON は最低でも数百 us 続くので、パルス幅で選別する。
    #   0 を渡すと従来どおりのエッジトリガに戻る。
    # ★2026-08-20: 既定を 0（エッジ）に変更。パルス幅トリガは「幅の条件を満たした
    #   パルスが終わった瞬間」＝ **磁界 OFF の立ち下がり**で発火する（実測）。これだと
    #   タグは 19 ms の無磁界の谷に撃つことになり、リーダには一切届かない（sof=0）。
    #   幅トリガが必要だったのは過大入力で段差が 23 mV に潰れていた頃の話で、
    #   レンジを正しく取れば素のエッジで問題ない。
    p.add_argument("--min-width", type=float, default=0.0, dest="min_width",
                   help="この時間以上 --level を超え続けたときだけトリガ [s] "
                        "(default 0 = エッジトリガ。>0 にすると磁界 OFF 側で発火するので注意)")
    p.add_argument("--holdoff", type=float, default=0.0,
                   help="トリガ後この時間は次のトリガを禁止 [s] "
                        "(0 = 禁止しない。1 サイクル 2 発の無駄撃ちを潰すなら 0.05 程度)")
    # ★AD3 のレンジは 5 V / 50 V の 2 段のみ（peak-to-peak 表記）。検波出力は実測
    #   6.9 V なので 50 が正解。5.0 だと ±2.5 V で飽和する。
    p.add_argument("--vrange", type=float, default=DETECT_VRANGE_V,
                   help="Scope CH1 のレンジ [V, peak-to-peak] (既定 %.1f = DETECT_VRANGE_V。"
                        "AD3 は 5 と 50 の 2 段しかない)" % DETECT_VRANGE_V)
    p.add_argument("--det-fs", type=float, default=1e6, dest="det_fs",
                   help="Scope CH1（検波）のサンプルレート")
    # ---- CH2 = アンテナの生 13.56 MHz（2026-09-22 の配線変更）----
    p.add_argument("--carrier-ch", type=int, default=CARRIER_CH, dest="carrier_ch",
                   help="搬送波を見るチャネル。dwf は 0 起点なので CH2 = 1 (default)")
    p.add_argument("--carrier-fs", type=float, default=100e6, dest="carrier_fs",
                   help="CH2 のサンプルレート [Hz] (default 100e6 = 13.56MHz の 7.4 倍。"
                        "これ未満だと山の頂点を踏み外して Vpp が小さく出る)")
    p.add_argument("--carrier-nbuf", type=int, default=8192, dest="carrier_nbuf",
                   help="CH2 の 1 回の取得サンプル数 (default 8192 = 164 us)")
    p.add_argument("--carrier-atten", type=float, default=1.0, dest="carrier_atten",
                   help="CH2 に入れた分圧/プローブの比 (×10 プローブなら 10)。"
                        "AD3 の入力は ±25 V が限界なのでアンテナ端が大きいときは必須")
    p.add_argument("--reader-rege", type=int, default=None, dest="reader_rege",
                   help="carrier: VDD_RF を手で決める (0..15, REGULATOR_CONTROL の rege)。"
                        "5V 電源では 3600+100*rege [mV]")
    p.add_argument("--rege-sweep", action="store_true", dest="rege_sweep",
                   help="carrier: rege を 0..15 まで振って、各段の実測電圧を並べる")
    p.add_argument("--reader-rfo", type=int, default=None, dest="reader_rfo",
                   help="carrier: RFO レベル 0..255 を直接書く（0x27 = ファームの txpwr）。"
                        "0x00=最大, 0xFF=実質 OFF。出力を «下げる» にはこちらが効く")
    p.add_argument("--rfo-sweep", action="store_true", dest="rfo_sweep",
                   help="carrier: 0x27 を 0x00..0xFF まで振って実測電圧を並べる")
    p.add_argument("--keep-field", action="store_true", dest="keep_field",
                   help="carrier: 測り終わっても磁界を ON のまま残す（既定は field off）")
    p.add_argument("--count",  type=int, default=5, help="selftest のトリガ回数")
    p.add_argument("--gap",    type=float, default=0.5, help="selftest のトリガ間隔 [s]")
    p.add_argument("--seconds", type=float, default=None,
                   help="detector / verify / link の観測時間 [s]（既定 5。"
                        "--target-packets があるときは上限、既定は無制限）")
    p.add_argument("--target-packets", type=int, default=None, dest="target_packets",
                   help="link --free-run: 集計に使えるパケット（時計が合っていた"
                        "ラウンドで実際に送った タグ数×ラウンド）がこの数に達したら終わる"
                        "（例 1000000）")
    p.add_argument("--label", default=None,
                   help="実験条件の名前（.meta.txt の label に残す。例 A_fixed100）")
    p.add_argument("--port", help="リーダのコンソール (/dev/cu.usbmodem...)。verify / link で使う")
    p.add_argument("--tsv", nargs="?", const="", default=None, metavar="PATH",
                   help="通信ログを論文シミュレータ互換の TSV に保存する（verify / link）。"
                        "パス省略時は log/YYYYMMDD_HHMMSS/YYYYMMDD_HHMMSS.tsv。詳細は host/pim_tsvlog.py")
    # ---- ラウンドごとのランダム化（link モード専用。論文の評価と同じ形にする）----
    p.add_argument("--random-data", action="store_true", dest="random_data",
                   help="link: ラウンドごとに各タグの data を 1..255 から引き直す")
    p.add_argument("--data-dist", choices=("uniform", "gauss"), default="uniform",
                   dest="data_dist",
                   help="--random-data の分布。uniform = 1..255 一様（既定）、gauss = "
                        "N(--data-mean, --data-std) を四捨五入して 1..255 に丸める"
                        "（シミュレータ config の data_dist と同じ）")
    p.add_argument("--data-mean", type=float, default=DATA_MEAN_DEFAULT, dest="data_mean",
                   help="--data-dist gauss の平均（既定 %g）" % DATA_MEAN_DEFAULT)
    p.add_argument("--data-std", type=float, default=DATA_STD_DEFAULT, dest="data_std",
                   help="--data-dist gauss の標準偏差（既定 %g）" % DATA_STD_DEFAULT)
    p.add_argument("--random-backoff", action="store_true", dest="random_backoff",
                   help="link: ラウンドごとに各タグの backoff を [0,--backoff-max) から引き直す")
    p.add_argument("--backoff-max", type=int, default=BACKOFF_MAX_DEFAULT, dest="backoff_max",
                   help="ランダム backoff の上限 [TU]（既定 16384 = 論文の 2^14）。"
                        "捕捉窓（424k で約 5554 TU）を超えた抽選は out_of_window として記録する")
    p.add_argument("--seed", type=int, default=42,
                   help="乱数シード（既定 42 で固定）。変えたいときだけ指定（.meta.txt に残る）")
    p.add_argument("--sim-max-tags", type=int, default=SIM_MAX_TAGS_DEFAULT, dest="sim_max_tags",
                   help="シミュレータ config の max_tags（既定 %d）。乱数の消費をシミュレータと"
                        "揃えるために使う" % SIM_MAX_TAGS_DEFAULT)
    p.add_argument("--dio-config", type=int, default=None, dest="dio_config",
                   help="AD3 のデバイス構成番号。省略時、ランダム backoff なら自動で %d "
                        "（DigitalOut 32768 bit/ch）を選ぶ" % DIO_CONFIG_BIGBUF)
    p.add_argument("--sweep", help="link: 全タグの backoff にこの値[TU]を足して順に試す "
                                   "(例 \"0,128,256,512\")")
    p.add_argument("--reader-sps", type=int, default=None, dest="reader_sps",
                   help="link: リーダの sps x1000 を明示する（未指定なら 'sps auto'）。"
                        "捕捉レートの実測校正値。ここが違うと data が間隔に比例してずれる")
    p.add_argument("--reader-rxconf", type=lambda x: int(x, 16), default=None,
                   dest="reader_rxconf", metavar="HEX",
                   help="verify/link: リーダの RX_CONF1 を 16 進で指定（例 A0）。"
                        "bit7=受信チャネル(0=AM/1=PM)、bit5:3=ローパス、bit2=h200。"
                        "bit7=0 だと捕捉列がタグの 2 倍になり «sof は立つが ID が読めない»。"
                        "ファーム既定 A0 = PM ch + 2MHz LP（実測で最良。8C は SOF が分裂する）")
    p.add_argument("--reader-agc", choices=("on", "off"), default=None,
                   dest="reader_agc",
                   help="verify/link: リーダの AGC + 自動 squelch (RX_CONF2 bit4/bit1)。"
                        "ファーム既定は off（ゲインを RX_CONF3/4 に固定）。"
                        "AGC/squelch は «ゲインを下げる» 方向にしか働かないので、"
                        "詰まりを疑うときに on/off を比較する")
    p.add_argument("--reader-gain", type=int, default=None, dest="reader_gain",
                   metavar="0..7",
                   help="verify/link: 受信 1 段目 PM ゲイン (RX_CONF3 rg1_pm)。"
                        "0=フルゲイン, 1-6=2.5dB/step 減衰, 7=+5.5dB boost。"
                        "チップ既定は 6 = 最小ゲインなので上げる余地が 15dB ある")
    p.add_argument("--reader-gain2", type=int, default=None, dest="reader_gain2",
                   metavar="0..10",
                   help="verify/link: 受信 2/3 段目 PM ゲイン + digitizer 窓 (RX_CONF4 rg2_pm)。"
                        "0=最大ゲイン/最小窓（既定）, 1-4=窓を 3dB ずつ広げる, "
                        "5-10=さらに 3dB ずつ減衰")
    p.add_argument("--tx-wait", default="auto", dest="tx_wait", metavar="SEC",
                   help="検波トリガから撃つまでの待ち [s]。既定 auto = --reader-gap と同じ"
                        "（磁界 OFF の過渡で撃っても ON に着地する）。0 で無効")
    p.add_argument("--reader-rate", type=int, default=READER_RATE, dest="reader_rate",
                   choices=(0, 1, 2, 3),
                   help="リーダの捕捉レート 0=40M 1=20M 2=10M 3=5M sps（公称）。"
                        "3 にすると復号が約半分の時間で終わり、遅い backoff の取りこぼしが"
                        "消える（--reader-sps 11850 とセットで）")
    p.add_argument("--reader-period", type=int, default=READER_PERIOD_MS,
                   dest="reader_period",
                   help="タグの送信間隔 [ms] = リーダの磁界サイクル周期 "
                        "(既定 %d = READER_PERIOD_MS)" % READER_PERIOD_MS)
    p.add_argument("--reader-gap", type=int, default=READER_GAP_MS, dest="reader_gap",
                   help="そのうち磁界 OFF の時間 [ms]。検波が放電しきる長さにすること "
                        "(既定 %d = READER_GAP_MS)" % READER_GAP_MS)
    # ★検波回路を使わない運用（2026-09-25 追加）。
    #   検波トリガは «リーダの磁界 ON» をタグに知らせるためだけの仕掛けだった。
    #   リーダのファーム（firmware-NFC05A1-424k-ringbuffer）は磁界 ON 固定のまま
    #   リングバッファで «連続» 捕捉できるので、タグの送信がいつ来ても拾える。
    #   よってタグ側は自分の時計で --reader-period ms ごとに撃てばよく、検波回路
    #   （Scope 1+/1-）もリーダの磁界 ON/OFF サイクルも丸ごと不要になる。
    p.add_argument("--reader-stuck", type=int, default=1, dest="reader_stuck",
                   help="link: «SOF は見えるのに 1 タグも取れない» セグメントが n 個続いたら "
                        "受信 FE を即入れ直す（既定 1、0 で無効）。これが «1 タグ落ちると "
                        "全タグ同時に落ちる» の対処。実測 40s/4 タグ: off=一致569 失敗19%%, "
                        "3=一致662, 2=一致673, 1=一致714 失敗2%%")
    p.add_argument("--reader-quiet", type=int, default=-1, dest="reader_quiet", metavar="MS",
                   help="free-run: 活動が MS ms 途絶えたらリーダの受信 FE を即入れ直す "
                        "（既定 -1 = 2x周期+50 ms、0 で無効）。入れ直した FE が稀に "
                        "«何も聞こえない» 状態で上がるのを周期 resync を待たずに直す")
    p.add_argument("--reader-resync", type=int, default=2000, dest="reader_resync",
                   metavar="MS",
                   help="free-run: リーダの受信フロントエンドを何 ms ごとに入れ直すか "
                        "(既定 2000。0 で無効)。磁界は切らず、数 ms だけ聞き逃す。"
                        "一度復調が落ちたまま戻らなくなるのを防ぐ")
    p.add_argument("--free-run", action="store_true", dest="free_run",
                   help="検波回路を使わず、--reader-period ms ごとにホストのタイマで送信する "
                        "(run / link / verify)。リーダには 'cycle off' を送り磁界 ON 固定にする")
    p.add_argument("--tx-engine", choices=("fast", "legacy"), default="fast",
                   dest="tx_engine",
                   help="link --free-run の送信方式。fast（既定）= パケットだけ載せ替え、"
                        "backoff は DigitalOut の CounterInit、PC トリガで周期どおり発射"
                        "（載せ替え ~5 ms -> 50/100 ms 周期が可能）。legacy = 従来の "
                        "フルバッファ載せ替え（~84 ms、200 ms 以上必要）。docs/PLAN_TX_PERIOD.md")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="link: 送信トリガを 1 発ずつ表示する")
    a = p.parse_args()
    if a.seconds is None:
        a.seconds = float("inf") if a.target_packets else 5.0
    if a.target_packets and not (a.mode == "link" and a.free_run and a.tx_engine == "fast"):
        raise SystemExit("--target-packets は link --free-run（--tx-engine fast）専用です。")
    if a.data_dist != "uniform" and not a.random_data:
        raise SystemExit("--data-dist は --random-data と一緒に使うこと（固定 data には効かない）。")
    # backoff を大きく振るには DigitalOut のバッファが足りない（既定構成 2048 bit/ch
    # = 1024 TU）。明示指定が無ければ大バッファ構成へ自動で切り替える。
    if a.dio_config is None and a.random_backoff and a.backoff_max > 1024:
        a.dio_config = DIO_CONFIG_BIGBUF
    if a.random_backoff and a.sweep:
        raise SystemExit("--random-backoff と --sweep は併用できません"
                         "（どちらも backoff を決めるため）")
    if a.reader_gain is not None and not 0 <= a.reader_gain <= 7:
        raise SystemExit("--reader-gain は 0..7（0=full, 1-6=-2.5dB/step, 7=boost）")
    if a.reader_gain2 is not None and not 0 <= a.reader_gain2 <= 10:
        raise SystemExit("--reader-gain2 は 0..10（RX_CONF4 は 0h..Ah のみ有効）")
    if (a.random_data or a.random_backoff) and a.mode != "link":
        raise SystemExit("--random-data / --random-backoff は link モード専用です。\n"
                         "  verify はループ再生でラウンドの区切りが無く、1 つの捕捉窓に\n"
                         "  新旧の抽選が混ざるため、記録と送信内容が一致しません。")
    if a.reader_sps is None and a.reader_rate is not None:
        # sps は «レートとセット»。未指定なら実測校正値（READER_RATE_SPS）を使う。
        a.reader_sps = READER_RATE_SPS.get(a.reader_rate)
    if a.free_run and a.mode not in ("run", "link", "verify"):
        raise SystemExit("--free-run は run / link / verify モード専用です。")
    if a.free_run and not a.reader_period:
        raise SystemExit("--free-run には送信間隔が要ります（--reader-period <ms>）。")
    if a.reader_period and a.reader_gap and not a.free_run \
            and a.reader_gap >= a.reader_period:
        raise SystemExit("--reader-gap (%d ms) は --reader-period (%d ms) より短いこと。\n"
                         "  gap は «1 周期のうち磁界 OFF の時間» なので、ON の時間が"
                         " 無くなるとタグは一度も撃てません。"
                         % (a.reader_gap, a.reader_period))
    if a.reader_rege is not None and not (0 <= a.reader_rege <= 15):
        raise SystemExit("--reader-rege は 0..15（REGULATOR_CONTROL の rege は 4 bit）")
    if a.reader_rfo is not None and not (0 <= a.reader_rfo <= 255):
        raise SystemExit("--reader-rfo は 0..255（0x27 は 8 bit）")
    if (a.rege_sweep or a.rfo_sweep or a.reader_rege is not None
            or a.reader_rfo is not None) and a.mode != "carrier":
        raise SystemExit("--reader-rege / --reader-rfo / 各 --*-sweep は carrier モード\n"
                         "  専用です。link/verify 中は磁界サイクルごとに SET_DEFAULT が\n"
                         "  走って消えるため、恒久的に変えるならファーム側に書くこと。")
    {"info": mode_info, "detector": mode_detector, "carrier": mode_carrier,
     "selftest": mode_selftest, "verify": mode_verify, "run": mode_run,
     "link": mode_link}[a.mode](a)


if __name__ == "__main__":
    main()
