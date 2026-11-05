"""PIM 通信ログを論文シミュレータと同じ TSV 形式で記録する。

参照シミュレータ: github.com/yokota-cronos/2026_globecom_pim_for_NFC
実機（本リポジトリ）の測定結果を、シミュレータの出力と同じ列・同じ書式で残すことで、
シミュレーションと実測をそのまま並べて比較できるようにする。

────────────────────────────────────────────────────────────────────────────
出力形式（タブ区切り, 1 行 1 イベント）
────────────────────────────────────────────────────────────────────────────
  time_tu  round  phase  event  tag_id  detail  result

  time_tu : イベント時刻 [TU]。ラウンド先頭（start_signal）を 0 とする相対時刻。
  round   : ラウンド番号（1 始まり）。実機ではリーダの 1 捕捉フレーム = 1 ラウンド。
  phase   : シミュレータ互換のフェーズ名。通常は "transmission"。
  event   : start_signal / tx_preamble / tx_postamble / rx_success / collision / rx_miss
            / rx_error（実機専用: 重なりなしで復号したが data が違った）
            / tx_dropped（実機専用: 抽選したが送信しなかった。tx_* 行は書かない）
  tag_id  : 2 桁小文字 16 進（例 a0）。該当しない行は "-"。
  detail  : "k=v,k=v" 形式の補足。該当しない行は "-"。
  result  : success / collision / miss / error。該当しない行は "-"。

行の順序は「時刻順」ではなく「論理順」（シミュレータと同じ）:
  start_signal → 全タグの tx_preamble → 全タグの tx_postamble → 各タグの受信結果

────────────────────────────────────────────────────────────────────────────
実機で記録するときの考え方
────────────────────────────────────────────────────────────────────────────
* 送信側（tx_preamble / tx_postamble）は **ホストが波形を作っているので完全に既知**。
  [PimCycle](pim.py) の preamble_start_tu / postamble_end_tu 等をそのまま書き出す。
* 受信側（rx_success）は **リーダが実際に復調できたものだけ**。時刻はシミュレータに
  合わせて「その タグの postamble 終端」を使う（実機は捕捉窓内の絶対時刻を返さないため）。
* 判定はシミュレータと同じ: 送信スケジュール上で他タグとパケットが重なっていれば
  **復号できていても collision**（detail に decoded= を残す）。重なっていないタグは
  rx_success（正しく復号）/ rx_error（値が違う）/ rx_miss（取れなかった ＝ 電波品質の
  問題）に振り分ける。**rx_miss はシミュレータには無い実機専用のイベント**で、
  メアンダコイル服のようなリンク品質評価ではこの 2 つの区別が本質的に重要になる。

使い方:
    import pim_tsvlog
    with pim_tsvlog.PimTsvLog() as log:          # log/YYYYMMDD_HHMMSS/YYYYMMDD_HHMMSS.tsv へ書く
        rec = pim_tsvlog.RoundRecorder(log, cycles)   # cycles = list[pim.PimCycle]
        rec.begin(1)                             # start_signal + 全タグの tx_*
        rec.rx(tag_id=0x66, data=89)             # リーダが復調できたぶんだけ
        rec.end()                                # 残りを collision / rx_miss として確定
"""
from __future__ import annotations

import datetime
import os

import pim

COLUMNS = ("time_tu", "round", "phase", "event", "tag_id", "detail", "result")

# このリポジトリのルート（host/ の 1 つ上）
_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

NONE = "-"          # 空欄の表現（シミュレータ互換）


def _fmt_tu(v) -> str:
    """時刻・長さ [TU] は小数 6 桁（シミュレータ出力と同じ書式）。"""
    return "%.6f" % float(v)


def _fmt_tag(tag_id) -> str:
    return NONE if tag_id is None else "%02x" % int(tag_id)


def _fmt_detail(pairs) -> str:
    """[(key, value), ...] → "k=v,k=v"。空なら "-"。"""
    if not pairs:
        return NONE
    return ",".join("%s=%s" % (k, v) for k, v in pairs)


def default_path(log_dir: str | None = None, when: datetime.datetime | None = None) -> str:
    """log/YYYYMMDD_HHMMSS/YYYYMMDD_HHMMSS.tsv（実行ごとにサブフォルダへまとめる）を返す。"""
    when = when or datetime.datetime.now()
    d = log_dir or os.path.join(_ROOT, "log")
    name = when.strftime("%Y%m%d_%H%M%S")
    return os.path.join(d, name, name + ".tsv")


# リーダの捕捉バッファ: g_capture = 32768 × uint16 = 524288 サンプル（1 bit/サンプル）
CAPTURE_BITS = 32768 * 16


def capture_window_tu(sps: float | None = None, fsc: float | None = None,
                      capture_fs: float = 40e6) -> float:
    """リーダが 1 回の捕捉で見られる長さ [TU]。

    捕捉は 40 Msps 固定・524288 サンプルなので時間長は 13.107 ms で不変。
    TU に直すと レートで変わる: 424k で約 5554 TU、848k で約 11108 TU。
    sps（= サンプル/TU、校正値 `--reader-sps`/1000）が分かればそれを使うほうが正確。
    """
    if sps is None:
        sps = capture_fs / (fsc or pim.FSC)
    return CAPTURE_BITS / float(sps)


def write_meta(tsv_path: str, meta: dict) -> str:
    """TSV 本体は論文と同じ列のまま保つため、実験条件は別ファイルに残す。
    <same-name>.meta.txt に "key: value" で書き出す。"""
    path = os.path.splitext(tsv_path)[0] + ".meta.txt"
    with open(path, "w", encoding="utf-8") as f:
        for k, v in meta.items():
            f.write("%s: %s\n" % (k, v))
    return path


def append_meta(tsv_path: str, meta: dict, header: str = "") -> str:
    """★実験«後»にしか分からない値（トリガ総数・sof 合計など）を .meta.txt に足す。

    実験条件だけを先に書いた write_meta() の続きに追記する。これが無いと
    「ログだけ見ても、そもそもタグが撃てていたのか」が後から分からない
    （2026-09-15 の切り分けはこれが無くて TSV の乱数列から推測する羽目になった）。"""
    path = os.path.splitext(tsv_path)[0] + ".meta.txt"
    with open(path, "a", encoding="utf-8") as f:
        if header:
            f.write("# %s\n" % header)
        for k, v in meta.items():
            f.write("%s: %s\n" % (k, v))
    return path


class PimTsvLog:
    """TSV を 1 行ずつ書き出すだけの薄いライター（イベントの意味付けはしない）。"""

    def __init__(self, path: str | None = None, log_dir: str | None = None,
                 phase: str = "transmission"):
        self.path = path or default_path(log_dir)
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.phase = phase
        self._f = open(self.path, "w", encoding="utf-8", newline="\n")
        self._f.write("\t".join(COLUMNS) + "\n")
        self._f.flush()
        self.n_rows = 0

    # -- 低レベル ------------------------------------------------------------
    def write(self, time_tu, round_no: int, event: str, tag_id=None,
              detail=None, result=None, phase: str | None = None) -> None:
        """1 行書く。detail は "k=v,.." 文字列か [(k, v), ...] のどちらでも可。"""
        if not isinstance(detail, (str, type(None))):
            detail = _fmt_detail(detail)
        row = (_fmt_tu(time_tu), str(int(round_no)), phase or self.phase, event,
               _fmt_tag(tag_id), detail or NONE, result or NONE)
        self._f.write("\t".join(row) + "\n")
        self._f.flush()          # 実験中に落ちても途中まで残るように毎行 flush
        self.n_rows += 1

    # -- 論文シミュレータと同じイベント --------------------------------------
    def start_signal(self, round_no: int, time_tu: float = 0.0, detail=None) -> None:
        """ラウンド開始（リーダの磁界 ON = タグが送信を始める合図）。

        detail には ★受信側の診断値★（sof=/tags=/trigger=）を入れる。列は
        シミュレータ互換のまま増やさず、この 1 行の detail だけで
        「撃てていたか（trigger）」「届いていたか（sof）」が後から分かるようにする。"""
        self.write(time_tu, round_no, "start_signal", detail=detail)

    def tx_preamble(self, round_no: int, tag_id: int,
                    start_tu: float, end_tu: float) -> None:
        self.write(start_tu, round_no, "tx_preamble", tag_id,
                   [("start", _fmt_tu(start_tu)), ("end", _fmt_tu(end_tu)),
                    ("packet_tu", _fmt_tu(end_tu - start_tu))])

    def tx_postamble(self, round_no: int, tag_id: int, start_tu: float, end_tu: float,
                     data: int, wait_tu: float) -> None:
        """wait_tu = preamble 終端 → postamble 始端 の間隔（= data × DATA_TU_MULT）。"""
        self.write(start_tu, round_no, "tx_postamble", tag_id,
                   [("start", _fmt_tu(start_tu)), ("end", _fmt_tu(end_tu)),
                    ("packet_tu", _fmt_tu(end_tu - start_tu)),
                    ("data", int(data)), ("wait", _fmt_tu(wait_tu))])

    def rx_success(self, round_no: int, tag_id: int, time_tu: float, data: int) -> None:
        self.write(time_tu, round_no, "rx_success", tag_id,
                   [("data", int(data))], "success")

    def collision(self, round_no: int, tag_id: int, time_tu: float, kind: str,
                  decoded: int | None = None, data_ok: bool = False) -> None:
        """kind = "preamble_collision" / "postamble_collision"。

        decoded: 重なっていたがリーダが復号できた値（できなければ None）。
        シミュレータと同じく失敗として数えるが、«重なったが復号できた» 割合を
        後から出せるように detail に残す（例 "preamble_collision,decoded=89,data_ok=1"）。"""
        detail = kind
        if decoded is not None:
            detail += ",decoded=%d,data_ok=%d" % (int(decoded), int(bool(data_ok)))
        self.write(time_tu, round_no, "collision", tag_id, detail, "collision")

    def rx_error(self, round_no: int, tag_id: int, time_tu: float, data: int,
                 sent: int) -> None:
        """★実機専用: 重なっていないのに、復号した data が送った値と違った（失敗扱い）。"""
        self.write(time_tu, round_no, "rx_error", tag_id,
                   [("data", int(data)), ("sent", int(sent))], "error")

    def rx_miss(self, round_no: int, tag_id: int, time_tu: float, reason: str = "no_collision") -> None:
        """★実機専用: 衝突していないのにリーダが復調できなかった（= 電波品質の問題）。"""
        self.write(time_tu, round_no, "rx_miss", tag_id, reason, "miss")

    def out_of_window(self, round_no: int, tag_id: int, time_tu: float,
                      window_tu: float, reason: str = "after_capture_window") -> None:
        """★実機専用: リーダの捕捉窓（13.1 ms 固定）より後ろに送信が落ちた。

        通信の失敗ではなく «観測範囲外» なので、成功率・衝突率の分母から外すこと。
        backoff を論文どおり 2^14 TU まで振ると、424k では窓が 5554 TU しか無いため
        抽選の約 2/3 がここに来る（docs/PLAN_DATARATE.md 参照）。"""
        self.write(time_tu, round_no, "out_of_window", tag_id,
                   [("reason", reason), ("window_tu", _fmt_tu(window_tu))], "out_of_window")

    def tx_dropped(self, round_no: int, tag_id: int, backoff_tu: float, data: int,
                   reason: str = "dio_buffer_full") -> None:
        """★実機専用: 抽選はしたが «送信しなかった»（AD3 のバッファに収まらない等）。

        電波を出していないので tx_preamble / tx_postamble は書かない。これで
        «送信試行 = tx_preamble の行数» というシミュレータの数え方がそのまま使え、
        衝突判定にも «出ていないパルス» が混ざらない。"""
        self.write(backoff_tu, round_no, "tx_dropped", tag_id,
                   [("reason", reason), ("backoff", _fmt_tu(backoff_tu)), ("data", int(data))],
                   "dropped")

    # -- 後始末 --------------------------------------------------------------
    def close(self) -> None:
        if self._f and not self._f.closed:
            self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# ============================================================================
#  衝突判定（送信スケジュールから幾何的に求める。シミュレータと同じ考え方）
# ============================================================================
def _overlaps(a0, a1, b0, b1) -> bool:
    return a0 < b1 and b0 < a1


def classify_collisions(cycles) -> dict:
    """各タグについて、他タグのパケットと時間的に重なっているかを判定する。

    戻り値: {tag_id: None | "preamble_collision" | "postamble_collision"}
    どちらも重なっていれば preamble 側を優先して報告する（先に壊れる方）。
    """
    out = {}
    for c in cycles:
        pre = (c.preamble_start_tu, c.preamble_end_tu)
        post = (c.postamble_start_tu, c.postamble_end_tu)
        hit_pre = hit_post = False
        for o in cycles:
            if o is c:
                continue
            for seg in ((o.preamble_start_tu, o.preamble_end_tu),
                        (o.postamble_start_tu, o.postamble_end_tu)):
                if _overlaps(pre[0], pre[1], seg[0], seg[1]):
                    hit_pre = True
                if _overlaps(post[0], post[1], seg[0], seg[1]):
                    hit_post = True
        out[c.tag_id] = ("preamble_collision" if hit_pre else
                         "postamble_collision" if hit_post else None)
    return out


class RoundRecorder:
    """1 ラウンド（= リーダの 1 捕捉フレーム）分を組み立てて TSV へ流す。

    使い方は begin() → rx() を復調できたぶんだけ → end()。
    end() で「送ったのに rx() が来なかったタグ」を collision / rx_miss に振り分ける。
    """

    def __init__(self, log: PimTsvLog, cycles, round_offset: int = 0,
                 window_tu: float | None = None, dropped=()):
        self.log = log
        # dropped: 抽選したが送信しなかったタグの cycles（cycles には含めないこと）。
        # begin() で dropped を渡さないラウンドはこの既定値を使う。
        self.dropped = list(dropped)
        # window_tu: リーダの捕捉窓 [TU]。これを超えた送信は観測できないので
        # rx_miss ではなく out_of_window として記録する（統計の分母から外すため）。
        self.window_tu = window_tu
        self._set_cycles(cycles)
        # 1 本の TSV に複数条件を続けて書くとき（link の --sweep 等）にラウンド番号を
        # 連番のまま繋げるためのオフセット。
        self.round_offset = round_offset
        self.round_no = 0
        self._seen = {}          # tag_id -> 復調された data
        self._open = False

    def _set_cycles(self, cycles) -> None:
        self.cycles = list(cycles)
        self._by_id = {c.tag_id: c for c in self.cycles}
        self._collision = classify_collisions(self.cycles)

    def begin(self, round_no: int, cycles=None, detail=None, dropped=None) -> None:
        """start_signal と、全タグの tx_preamble / tx_postamble を書く。

        cycles を渡すと、そのラウンドで実際に送った内容へ差し替える
        （data / backoff をラウンドごとに抽選し直す場合に使う）。
        detail はリーダ側の診断値（sof=/tags=/trigger=）で、start_signal 行に残る。
        dropped は «抽選したが送信しなかった» タグの cycles。cycles とは別に渡すこと
        （cycles に入れると、出ていないパルスで衝突判定し、rx_miss として数えてしまう）。"""
        if self._open:
            self.end()
        if cycles is not None:
            self._set_cycles(cycles)
        if dropped is not None:
            self.dropped = list(dropped)
        round_no += self.round_offset
        self.round_no = round_no
        self._seen = {}
        self._open = True
        self.log.start_signal(round_no, detail=detail)
        for c in self.cycles:
            self.log.tx_preamble(round_no, c.tag_id, c.preamble_start_tu, c.preamble_end_tu)
        for c in self.cycles:
            self.log.tx_postamble(round_no, c.tag_id, c.postamble_start_tu,
                                  c.postamble_end_tu, c.data, c.interval_tu)
        for c in self.dropped:
            self.log.tx_dropped(round_no, c.tag_id, c.preamble_start_tu, c.data)

    def rx(self, tag_id: int, data: int) -> None:
        """リーダが復調できた 1 件を記録する（end() でまとめて書き出す）。"""
        if self._open:
            self._seen[int(tag_id)] = int(data)

    def end(self) -> None:
        """このラウンドの受信結果（success / collision / miss）を確定して書く。"""
        if not self._open:
            return
        # ★判定はシミュレータ（simulation/src/pim/reader.py）に合わせる（2026-10-17）:
        #   スケジュール上で他タグと重なったら、復号できていても collision（失敗）。
        #   復号できていたことは detail の decoded= / data_ok= に残す。
        #   重なっていないタグだけを、正しく復号 -> rx_success / 値が違う -> rx_error /
        #   復号なし -> rx_miss に分ける（rx_error / rx_miss は実機専用）。
        for c in self.cycles:
            t = c.postamble_end_tu
            got = self._seen.get(c.tag_id)
            if self.window_tu is not None and t > self.window_tu and got is None:
                # 窓の外＝そもそも観測できない。衝突判定より先に見ること
                # （観測できていない以上「衝突で落ちた」とは言えない）。
                self.log.out_of_window(self.round_no, c.tag_id, t, self.window_tu)
            elif self._collision[c.tag_id]:
                self.log.collision(self.round_no, c.tag_id, t, self._collision[c.tag_id],
                                   decoded=got, data_ok=(got == c.data))
            elif got is None:
                self.log.rx_miss(self.round_no, c.tag_id, t)
            elif got != c.data:
                self.log.rx_error(self.round_no, c.tag_id, t, got, c.data)
            else:
                self.log.rx_success(self.round_no, c.tag_id, t, got)
        # 送っていない tag_id をリーダが報告した場合（ノイズ/ID 誤り）も残す
        for tid, data in self._seen.items():
            if tid not in self._by_id:
                self.log.write(0.0, self.round_no, "rx_unknown", tid,
                               [("data", int(data))], "unknown")
        self._open = False


# ============================================================================
#  自己テスト（ハード不要）: python3 host/pim_tsvlog.py
# ============================================================================
if __name__ == "__main__":
    import tempfile

    cycles = [pim.PimCycle(tag_id=0xa0, data=125, backoff_tu=3638, data_tu_mult=2),
              pim.PimCycle(tag_id=0x66, data=89,  backoff_tu=3773, data_tu_mult=2),
              pim.PimCycle(tag_id=0x04, data=249, backoff_tu=3917, data_tu_mult=2)]
    path = os.path.join(tempfile.gettempdir(), "pim_tsvlog_selftest.tsv")
    with PimTsvLog(path) as log:
        rec = RoundRecorder(log, cycles)
        rec.begin(1)
        rec.rx(0x66, 89)          # 1 台だけ復調できた想定
        rec.end()
        # 2 ラウンド目: 0x04 は «送信しなかった»（cycles から外して dropped へ）
        rec.begin(2, cycles=cycles[:2], dropped=cycles[2:])
        rec.end()
        # 3 ラウンド目: 0xa0 は 0x04 と重なっているが復号できた -> シミュレータと同じく collision
        #               0x66 は重なりなしだが data を取り違えた -> rx_error
        rec.begin(3, cycles=cycles, dropped=[])
        rec.rx(0xa0, 125)
        rec.rx(0x66, 90)
        rec.end()
    text = open(path, encoding="utf-8").read()
    print(text)
    rows = [l.split("\t") for l in text.strip().split("\n")]
    assert rows[0] == list(COLUMNS), rows[0]
    assert all(len(r) == len(COLUMNS) for r in rows), "列数が揃っていない"
    events = [r[3] for r in rows[1:]]
    assert events[0] == "start_signal"
    assert events.count("tx_preamble") == 8 and events.count("tx_postamble") == 8
    assert events.count("tx_dropped") == 1
    r2 = [r for r in rows[1:] if r[1] == "2"]
    assert not any(r[4] == "04" and r[3] in ("tx_preamble", "rx_miss", "collision")
                   for r in r2), "送信しなかったタグが送信扱いになっている"
    assert "rx_success" in events, events
    r3 = {(r[3], r[4]): r for r in rows[1:] if r[1] == "3"}
    assert ("collision", "a0") in r3 and "decoded=125,data_ok=1" in r3[("collision", "a0")][5], r3
    assert ("rx_success", "a0") not in r3, "重なったのに success になっている"
    assert ("rx_error", "66") in r3, r3
    print("saved: %s" % path)
    print("self-test: OK (%d rows)" % (len(rows) - 1))
