"""PIM (Packet Interval Modulation) の物理層コーデック — 1 箇所に集約。

参照論文: S. Ishida, R. Takahashi, S. Ido,
  "Packet Interval Modulation-based NFC for Scalable IoT around Meandered
   E-textile," IEEE Globecom 2026.
参照シミュレータ: github.com/yokota-cronos/2026_globecom_pim_for_NFC

────────────────────────────────────────────────────────────────────────────
PIM の考え方（1 対 1 の最小構成）
────────────────────────────────────────────────────────────────────────────
  * リーダは 13.56 MHz の連続磁界を出すだけ（ダウンリンク・コマンド無し）。
  * タグは自律的に「2 発のパルス」を送る:
        パルス1 = preamble  = 短い SOF + Tag ID(1 byte)
        （間隔 = センサ値をエンコード）
        パルス2 = postamble = 長い SOF + Tag ID(1 byte)
  * 2 発のパルスの「時間間隔」に 8 bit のセンサ値を載せる（value-to-interval）。

物理層は ISO/IEC 15693 (NFC Type-V) がベース:
  * サブキャリア fsc = fc/32 ≈ 424 kHz（負荷変調の副搬送波）
  * タグは RF スイッチで負荷変調 = サブキャリアの ON/OFF（AM 包絡線でリーダが受信）
  * ID は Manchester 符号（1 bit = 2 chip、1 chip = 1 サブキャリア周期 = 1 TU）

★1 パルスの構造（SOF が preamble/postamble の区別も兼ねる）:
      [SOF: solid-ON N TU][gap: OFF 1 TU][ID: 8 bit Manchester = 16 TU]
    - SOF = サブキャリアを連続 ON したバースト。Manchester の ON 連続は最大 2 TU
      なので、それより長い（≥3 TU）連続 ON は SOF だけ → パケット開始を曖昧なく検出。
    - SOF の「長さ」で preamble / postamble を区別する（短い=preamble, 長い=postamble）。
      これで独立した delimiter バイト（16 TU）が不要になり、1 パルスが短くなる
      （= チャネル占有↓ = PIM の狙いに直結）。
    - SOF 直後の gap(1 TU OFF) は、続く ID の先頭 chip が ON でも SOF の塊が伸びない
      ようにして SOF 長を正確に測るための保証。

時間の単位:
  * TU (Time Unit) = 1/fsc ≈ 2.36 µs（サブキャリア 1 周期）
  * preamble パルス長 = SOF_PRE(4) + gap(1) + ID(16) = 21 TU
  * postamble パルス長 = SOF_POST(8) + gap(1) + ID(16) = 25 TU
  * パルス間隔（= センサ値）= data(1..255) × DATA_TU_MULT [TU]
  * バックオフ = random(0, BACKOFF_MAX) [TU]（最初の preamble までの待ち）

このモジュールは numpy だけに依存。ハード無しで `python host/pim.py` を実行すると
生成→復号のループバック自己テストが走る（物理層の定義が自己整合か検証）。
"""
from __future__ import annotations
from dataclasses import dataclass
import os
import numpy as np


# ============================================================================
#  物理層パラメータ（論文 + シミュレータ config と一致）
# ============================================================================
FC              = 13.56e6           # NFC 搬送波 [Hz]
# サブキャリア分周 fc/32（ISO15693 標準・既定）。データレート評価（848k=fc/16, 1696k=fc/8）
# 用に PIM_SUBCARRIER_DIV 環境変数で上書き可能（docs/PLAN_DATARATE.md 参照）。
SUBCARRIER_DIV  = int(os.environ.get("PIM_SUBCARRIER_DIV", 32))
FSC             = FC / SUBCARRIER_DIV        # サブキャリア ≈ 423.75 kHz
TU              = 1.0 / FSC                  # Time Unit ≈ 2.36 µs（= 1 サブキャリア周期）

CHIPS_PER_BIT   = 2                 # Manchester: 1 bit = 2 chip
BIT_RATE        = FSC / CHIPS_PER_BIT        # 情報レート ≈ 212 kbps

ID_BITS         = 8                 # Tag ID = 1 byte（最大 256 タグ）
ID_TU           = ID_BITS * CHIPS_PER_BIT    # = 16 TU
DATA_BITS       = 8                 # センサ値 = 8 bit（1..255）

# --- SOF（Start-Of-Frame）: framing ＋ preamble/postamble 区別を兼ねる ---
SOF_PRE_TU      = 4                 # preamble の SOF 長 [TU]（短い）
SOF_POST_TU     = 8                 # postamble の SOF 長 [TU]（長い）
SOF_MIN_TU      = 3                 # SOF とみなす最小 ON 連続（Manchester 最大 2 TU 超）
SOF_CLASS_TU    = (SOF_PRE_TU + SOF_POST_TU) / 2.0   # 分類しきい値（= 6 TU）
GAP_TU          = 1                 # SOF 直後の OFF（SOF 長を綺麗に測るための保証）

PRE_PULSE_TU    = SOF_PRE_TU  + GAP_TU + ID_TU       # = 21 TU（preamble パルス長）
POST_PULSE_TU   = SOF_POST_TU + GAP_TU + ID_TU       # = 25 TU（postamble パルス長）

# --- 復号の判定パラメータ（物理層ではなく «受信機» 側の定数）---
# ID の 1 bit は Manchester なので «ON 1 chip + OFF 1 chip» で必ず出来ている。
# だからビットの中の 2 chip を «比べる» だけで復号できる（絶対しきい値は要らない）。
# 下は「その 2 chip の振幅差がこれ未満なら判定を諦める」という最低コントラスト
# （× 閾値）。0.1〜0.4 のどこでも実測結果は同じだった（2026-09-25 実機ダンプ）。
ID_CHIP_CONTRAST = 0.25
# ID のサンプル位置を探す範囲 [chip]。±0.5 chip = ±半チップ。これより広げると
# «1 chip ずれ» を掴むことがあって実測で悪化した（2026-09-25: 17/20 -> 17/20 だが
# 単独タグは 19/22 -> 7/22）。
ID_SEARCH_CHIPS  = 0.5

DATA_TU_MULT    = 1                 # センサ値→間隔 の TU 倍率（redundancy factor x1)
BACKOFF_MAX     = 4096              # ランダムバックオフ最大 [TU]（論文最適は 2^14）

# preamble / postamble の識別（SOF 長で表現。値は「種別」を表す定数）
KIND_PREAMBLE   = 0
KIND_POSTAMBLE  = 1


# ============================================================================
#  1) 論理層 — バイト ⇔ Manchester、パルス時刻 ⇔ センサ値
# ============================================================================
def byte_to_bits(byte: int) -> list[int]:
    """1 バイトを MSB first の 8 bit リストへ。"""
    return [(byte >> (7 - i)) & 1 for i in range(8)]


def bits_to_byte(bits: list[int]) -> int:
    """MSB first の 8 bit リストを 1 バイトへ。"""
    v = 0
    for b in bits:
        v = (v << 1) | (b & 1)
    return v


def manchester_encode(bits: list[int]) -> np.ndarray:
    """ビット列を Manchester chip 列へ（1 = サブキャリア有り, 0 = 無し）。
    規約 (IEEE 802.3): bit 1 -> [0,1](low→high), bit 0 -> [1,0](high→low)。"""
    chips = []
    for b in bits:
        chips += [0, 1] if b else [1, 0]
    return np.array(chips, dtype=np.int8)


def manchester_decode(chips: np.ndarray) -> list[int]:
    """Manchester chip 列（1 chip/TU）をビット列へ復号。ペアが同値なら -1（違反）。"""
    bits = []
    for i in range(0, len(chips) - 1, 2):
        a, b = int(chips[i]), int(chips[i + 1])
        if a == 0 and b == 1:
            bits.append(1)
        elif a == 1 and b == 0:
            bits.append(0)
        else:
            bits.append(-1)
    return bits


def sof_tu_of(kind: int) -> int:
    """種別（preamble/postamble）に対応する SOF 長 [TU]。"""
    return SOF_PRE_TU if kind == KIND_PREAMBLE else SOF_POST_TU


@dataclass
class PimCycle:
    """タグ 1 サイクル分の送信内容（1 対 1 では 1 タグ）。"""
    tag_id: int          # 0..255
    data:   int          # 1..255（8 bit センサ値）
    backoff_tu: int = 0  # 最初の preamble までの待ち [TU]
    data_tu_mult: int = DATA_TU_MULT

    @property
    def preamble_start_tu(self) -> float:
        return float(self.backoff_tu)

    @property
    def preamble_end_tu(self) -> float:
        return self.preamble_start_tu + PRE_PULSE_TU

    @property
    def interval_tu(self) -> float:
        """preamble 終端 → postamble 始端 の間隔（= センサ値をエンコード）。"""
        return self.data * self.data_tu_mult

    @property
    def postamble_start_tu(self) -> float:
        return self.preamble_end_tu + self.interval_tu

    @property
    def postamble_end_tu(self) -> float:
        return self.postamble_start_tu + POST_PULSE_TU


def decode_value_from_starts(preamble_start_tu: float, postamble_start_tu: float,
                             data_tu_mult: int = DATA_TU_MULT) -> int:
    """2 パルスの開始時刻 [TU] からセンサ値を復号。
    postamble_start - preamble_start = PRE_PULSE_TU + data×mult。"""
    interval_tu = (postamble_start_tu - preamble_start_tu) - PRE_PULSE_TU
    return int(round(interval_tu / data_tu_mult))


# ============================================================================
#  2) 波形層 — サイクル → RF スイッチ制御波形（AD3 用, ±1）
# ============================================================================
def build_switch_waveform(cycle: PimCycle, sps_tu: int,
                          idle_tu_after: int = 64) -> np.ndarray:
    """1 サイクルの RF スイッチ制御波形（±1）を生成。

    * +1 / -1 は RF スイッチの 2 状態。サブキャリア ON 区間はサンプル毎に ±1 を
      交互（= fsc の方形波）にして負荷変調の副搬送波を作る。OFF 区間は -1 固定。
    * sps_tu = 1 TU (= 1 サブキャリア周期) あたりのサンプル数（偶数推奨）。

    戻り値は AD3 の wavegen_custom_diff() にそのまま渡せる ±1 バッファ。
    """
    total_tu = int(np.ceil(cycle.postamble_end_tu)) + idle_tu_after
    n = total_tu * sps_tu
    wave = -np.ones(n, dtype=float)      # 既定 = スイッチ OFF（変調なし）

    half = sps_tu // 2
    sc_cycle = np.concatenate([np.ones(half), -np.ones(sps_tu - half)])  # 1 TU 分の副搬送波

    def put_tu(tu_index: int):
        seg = tu_index * sps_tu
        wave[seg:seg + sps_tu] = sc_cycle

    def stamp_pulse(kind: int, start_tu: float):
        base = int(round(start_tu))
        sof_len = sof_tu_of(kind)
        for t in range(sof_len):                       # (1) SOF: solid-ON（種別=長さ）
            put_tu(base + t)
        # (2) gap(1 TU OFF) はそのまま（既定 -1）→ (3) ID を Manchester で
        body = base + sof_len + GAP_TU
        chips = manchester_encode(byte_to_bits(cycle.tag_id))   # 16 chip
        for k, chip in enumerate(chips):
            if chip:
                put_tu(body + k)

    stamp_pulse(KIND_PREAMBLE,  cycle.preamble_start_tu)
    stamp_pulse(KIND_POSTAMBLE, cycle.postamble_start_tu)
    return wave


# ============================================================================
#  3) 復号 front-end — 1-bit サブキャリア列 → AM 包絡線
# ============================================================================
def capture_to_magnitude(subcarrier_bits: np.ndarray, sps_tu: int) -> np.ndarray:
    """★実機 front-end★ 1-bit サブキャリア列 → サブキャリア振幅（連続値）。

    ST25R3911B の transparent モードは AM 復調後の「1-bit デジタイズ済み
    サブキャリア列」を返す（サブキャリア有り→fsc でトグル, 無し→一定 or ノイズ）。
    「その区間にサブキャリアが有るか？」を fsc での I/Q ダウンコンバート＋1 TU LPF の
    振幅で判定する（レベル/エッジ閾値より遥かにノイズに強い）。ファーム pim_iq_mag と同一。

    ★振幅を «1-bit に潰さず» 返すのが肝。ID の 1 TU チップは SOF（4〜8 TU 連続）ほど
      振幅が出ないので、SOF 由来の絶対しきい値で 0/1 にするとチップだけが系統的に
      落ちる（2026-09-25 実機: SOF は 199% なのにチップが 46% で全 ID 失敗）。
      判定はビット内 2 チップの «比較» で行う（_read_id 参照）。"""
    b = np.where(np.asarray(subcarrier_bits) > 0, 1.0, -1.0)
    s = np.arange(len(b))
    ph = 2.0 * np.pi * s / sps_tu
    w = max(1, int(round(sps_tu)))
    k = np.ones(w) / w
    ilp = np.convolve(b * np.cos(ph), k, mode="same")
    qlp = np.convolve(b * np.sin(ph), k, mode="same")
    return 2.0 * np.sqrt(ilp * ilp + qlp * qlp)


def capture_to_envelope(subcarrier_bits: np.ndarray, sps_tu: int) -> np.ndarray:
    """（互換 / プロット用）上の振幅を 0.5×最大で 2 値化した 1-bit 包絡線。"""
    mag = capture_to_magnitude(subcarrier_bits, sps_tu)
    thr = 0.5 * float(np.max(mag)) if mag.size and np.max(mag) > 0 else 0.5
    return (mag > thr).astype(np.int8)


def subcarrier_iq_amplitude(analog: np.ndarray, sps_tu: int) -> np.ndarray:
    """（解析/プロット用）アナログ AM 波形 → サブキャリア振幅（I/Q 復調, 0..~1）。"""
    s = np.arange(len(analog))
    ph = 2.0 * np.pi * s / sps_tu
    k = np.ones(sps_tu) / sps_tu
    ilp = np.convolve(analog * np.cos(ph), k, mode="same")
    qlp = np.convolve(analog * np.sin(ph), k, mode="same")
    return 2.0 * np.sqrt(ilp * ilp + qlp * qlp)


def envelope_from_switch(wave: np.ndarray, sps_tu: int) -> np.ndarray:
    """（自己テスト用）RF スイッチ波形 → 1-bit 包絡線を実機と同じ経路で模擬。"""
    return capture_to_envelope((wave > 0).astype(np.int8), sps_tu)


# ============================================================================
#  4) 復号 — AM 包絡線 → サイクル（SOF 検出 + 長さで種別判定）
# ============================================================================
def decode_envelope(env: np.ndarray, sps_tu: int,
                    data_tu_mult: int = DATA_TU_MULT) -> dict | None:
    """（互換 API）1-bit AM 包絡線から 1 サイクルを復号。

    0/1 をそのまま «振幅» として decode_magnitude に渡す。ビット内 2 チップの比較は
    0/1 でも成立するので（1 対 0 は必ずコントラストを満たす）、自己テストの意味は
    従来どおり保たれる。実機の捕捉は capture_to_magnitude + decode_magnitude を使う
    こと（1 TU チップの弱い振幅を捨てないため）。"""
    return decode_magnitude(np.asarray(env, dtype=float), sps_tu,
                            data_tu_mult=data_tu_mult)


def decode_magnitude(mag: np.ndarray, sps_tu: float,
                     data_tu_mult: int = DATA_TU_MULT) -> dict | None:
    """サブキャリア振幅列（capture_to_magnitude の出力）から 1 サイクルを復号。

    戻り: {"tag_id","data","preamble_id","postamble_id","interval_tu",
           "preamble_start_tu","postamble_start_tu","ok"} / 失敗で None。
    """
    mag = np.asarray(mag, dtype=float)
    peak = float(np.max(mag)) if mag.size else 0.0
    if peak <= 0.0:
        return None
    thr = 0.5 * peak
    sofs = _find_sofs(mag > thr, sps_tu)  # [(start_sample, kind), ...]
    if len(sofs) < 2:
        return None

    # タグ波形は連続再生されるので、捕捉がサイクル途中から始まると先頭が postamble の
    # ことがある。「preamble → 同 ID の postamble」の最初のペアを探す。
    pre = post = None
    for (s, kind) in sofs[:8]:
        tid = _read_id(mag, thr, s, kind, sps_tu)
        if tid is None:
            continue
        if kind == KIND_PREAMBLE and pre is None:
            pre = (s, tid)
        elif kind == KIND_POSTAMBLE and pre is not None and tid == pre[1]:
            post = (s, tid)
            break
    if pre is None or post is None:
        return None

    preamble_start_tu  = pre[0]  / sps_tu
    postamble_start_tu = post[0] / sps_tu
    data = decode_value_from_starts(preamble_start_tu, postamble_start_tu, data_tu_mult)
    return {
        "tag_id":             pre[1],
        "preamble_id":        pre[1],
        "postamble_id":       post[1],
        "data":               data,
        "interval_tu":        (postamble_start_tu - preamble_start_tu) - PRE_PULSE_TU,
        "preamble_start_tu":  preamble_start_tu,
        "postamble_start_tu": postamble_start_tu,
        "ok":                 (pre[1] == post[1]) and (1 <= data <= 255),
    }


def _find_sofs(high: np.ndarray, sps_tu: float) -> list[tuple[int, int]]:
    """包絡線から SOF（ON が SOF_MIN_TU 以上連続する塊）を列挙し、その「長さ」で
    preamble/postamble を分類。戻り: [(start_sample, kind), ...]。
    Manchester の ON 連続は最大 2 TU なので、SOF_MIN_TU=3 で SOF だけ拾える。

    ★SOF だけは «絶対しきい値» で拾ってよい。4〜8 TU の連続バーストは 1 TU の LPF を
      完全に満たすので振幅が飽和するし、SOF 検出は実機でも一度も外していない。"""
    high = np.asarray(high) > 0
    min_run = int(SOF_MIN_TU * sps_tu)
    out = []
    i, n = 0, len(high)
    while i < n:
        if high[i]:
            j = i
            while j < n and high[j]:
                j += 1
            if j - i >= min_run:
                len_tu = (j - i) / sps_tu
                kind = KIND_PREAMBLE if len_tu < SOF_CLASS_TU else KIND_POSTAMBLE
                out.append((i, kind))
            i = j
        else:
            i += 1
    return out


def _chip_amps(mag: np.ndarray, body0: float, off: float,
               sps_tu: float) -> np.ndarray | None:
    """ID 16 チップの «中央 1 サンプル» の振幅。範囲外なら None。
    （チップ全体の平均や最大にすると実測で悪化する: 隣チップの裾を拾ってしまう）"""
    out = np.empty(ID_BITS * CHIPS_PER_BIT)
    for k in range(ID_BITS * CHIPS_PER_BIT):
        c = int(round(body0 + off + (k + 0.5) * sps_tu))
        if c < 0 or c >= len(mag):
            return None
        out[k] = mag[c]
    return out


def _read_id(mag: np.ndarray, thr: float, sof_start: int, kind: int,
             sps_tu: float) -> int | None:
    """SOF 開始位置 + SOF 長 + gap の直後から ID 8 bit を Manchester 復号。
    sps_tu は小数可（実機の 40MHz/423.75kHz=94.4 等）。

    ファーム pim_read_id と同一のアルゴリズム:
      1) タイミング再生 — 公称位置の ±ID_SEARCH_CHIPS チップを探し、「ビット内 2
         チップの振幅差の総和」が最大になる位置を選ぶ。SOF の閾値交差は変調の強さで
         前後するので、公称位置だけでは 1/4 チップ以上ずれる。
      2) 判定 — ビット内 2 チップの «どちらが大きいか» でビットを決める。Manchester は
         ON 1 個 + OFF 1 個なのでこれが正しい判定規則で、しかも «1 TU チップの振幅は
         SOF より小さい» という系統誤差に影響されない（絶対しきい値だと全滅する）。
    """
    body0 = sof_start + (sof_tu_of(kind) + GAP_TU) * sps_tu
    span = int(round(ID_SEARCH_CHIPS * sps_tu))
    best, best_score = None, -1.0
    for off in range(-span, span + 1):
        a = _chip_amps(mag, body0, off, sps_tu)
        if a is None:
            continue
        score = float(np.sum(np.abs(a[0::2] - a[1::2])))   # ビット内 2 チップの差
        if score > best_score:
            best, best_score = a, score
    if best is None:
        return None
    bits = []
    for i in range(ID_BITS):
        d = best[2 * i + 1] - best[2 * i]                  # bit 1 -> [0,1]
        if abs(d) < ID_CHIP_CONTRAST * thr:                # 差が無い = 判定不能
            return None
        bits.append(1 if d > 0 else 0)
    return bits_to_byte(bits)


# ============================================================================
#  自己テスト — ハード不要（生成 → 包絡線模擬 → 復号 が往復一致するか）
# ============================================================================
def _run_cases(sps_tu: int, ncase: int, rng, noise: float = 0.0,
               data_tu_mult: int = DATA_TU_MULT) -> int:
    fails = 0
    for _ in range(ncase):
        tag_id  = int(rng.integers(0, 256))
        data    = int(rng.integers(1, 256))
        backoff = int(rng.integers(0, 200))
        cyc  = PimCycle(tag_id=tag_id, data=data, backoff_tu=backoff,
                        data_tu_mult=data_tu_mult)
        wave = build_switch_waveform(cyc, sps_tu)
        if noise > 0:
            wave = wave + rng.normal(0, noise, len(wave))
        env  = envelope_from_switch(wave, sps_tu)
        got  = decode_envelope(env, sps_tu, data_tu_mult=data_tu_mult)
        ok = (got is not None and got["ok"]
              and got["tag_id"] == tag_id and got["data"] == data)
        if not ok:
            fails += 1
            if fails <= 3:
                print(f"    FAIL id={tag_id} data={data} backoff={backoff} -> {got}")
    return fails


def _selftest() -> int:
    print(f"PIM physical layer:  fsc={FSC/1e3:.2f} kHz  TU={TU*1e6:.3f} us  "
          f"bit_rate={BIT_RATE/1e3:.1f} kbps  pulse=pre {PRE_PULSE_TU}TU / "
          f"post {POST_PULSE_TU}TU  (SOF {SOF_PRE_TU}/{SOF_POST_TU})")
    rng = np.random.default_rng(0)
    total_fail = 0
    for label, sps, ncase, noise, mult in [
        ("sps=16  clean           ", 16, 300, 0.0,  1),
        ("sps=94  clean (40Msps)   ", 94, 200, 0.0,  1),
        ("sps=94  noise=0.3        ", 94, 200, 0.30, 1),
        ("sps=94  mult=2 (x2 redund)", 94, 150, 0.0,  2),
    ]:
        f = _run_cases(sps, ncase, rng, noise=noise, data_tu_mult=mult)
        total_fail += f
        print(f"  {label}: {ncase - f}/{ncase} OK"
              + ("  [PASS]" if f == 0 else f"  [FAIL {f}]"))

    # 連続再生（捕捉がサイクル途中から始まる）ケース
    sps = 94; f = 0
    for _ in range(120):
        tag_id = int(rng.integers(0, 256)); data = int(rng.integers(1, 256))
        cyc = PimCycle(tag_id=tag_id, data=data, backoff_tu=30)
        w1 = build_switch_waveform(cyc, sps)
        two = np.concatenate([w1, w1])
        roll = int(rng.integers(0, len(w1)))
        env = envelope_from_switch(np.roll(two, -roll), sps)
        got = decode_envelope(env, sps)
        if not (got and got["ok"] and got["tag_id"] == tag_id and got["data"] == data):
            f += 1
    total_fail += f
    print(f"  sps=94  continuous-replay (rolled): {120 - f}/120 OK"
          + ("  [PASS]" if f == 0 else f"  [FAIL {f}]"))

    print("loopback self-test: " + ("ALL PASS" if total_fail == 0
                                     else f"{total_fail} FAILED"))
    return 1 if total_fail else 0


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
