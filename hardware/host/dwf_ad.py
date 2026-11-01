"""Minimal ctypes wrapper around the WaveForms `dwf` library, loaded straight
from the installed app bundle (pydwf hard-codes /Library/Frameworks which the
drag-installed WaveForms.app does not populate).

Exposes just what the VHBR experiment needs: open device, drive W1 (square or
custom BPSK buffer), capture a scope channel.
"""
from __future__ import annotations
import ctypes, ctypes.util, sys, os, time
import numpy as np


def _load_dwf():
    """Load the Digilent WaveForms `dwf` library on macOS / Windows / Linux
    without hard-coding an install path. Order: env override -> OS default
    location -> loader search (ctypes.util.find_library / PATH). Returns the
    loaded ctypes library; raises a helpful error if WaveForms is not found."""
    env = os.environ.get("DWF_LIB")                 # explicit override wins
    cands = [env] if env else []
    if sys.platform == "darwin":                    # macOS: WaveForms.app bundle
        fw = "/Library/Frameworks/dwf.framework/dwf"
        app = "/Applications/WaveForms.app/Contents/Frameworks/dwf.framework/dwf"
        cands += [fw, app]
        # add the SDK python samples dir if present (dwfconstants etc.)
        for base in ("/Applications/WaveForms.app/Contents/Resources/SDK",
                     "/Library/Frameworks/dwf.framework/Versions/Current"):
            s = os.path.join(base, "samples", "py")
            if os.path.isdir(s):
                sys.path.insert(0, s)
    elif sys.platform.startswith("win"):            # Windows: dwf.dll (System32 / SDK)
        cands += [
            "dwf",                                   # on PATH (System32)
            r"C:\Windows\System32\dwf.dll",
            r"C:\Program Files\Digilent\WaveForms3\dwf.dll",
            r"C:\Program Files (x86)\Digilent\WaveForms3\dwf.dll",
        ]
        for base in (r"C:\Program Files\Digilent\WaveFormsSDK",
                     r"C:\Program Files (x86)\Digilent\WaveFormsSDK"):
            s = os.path.join(base, "samples", "py")
            if os.path.isdir(s):
                sys.path.insert(0, s)
    else:                                            # Linux: libdwf.so
        cands += ["libdwf.so", "libdwf.so.3", "/usr/lib/libdwf.so"]

    cands = [c for c in cands if c]
    # also try the platform loader search as a last resort
    found = ctypes.util.find_library("dwf")
    if found:
        cands.append(found)
    errs = []
    # Digilent's dwf uses the cdecl convention on every platform (the official
    # Windows samples load it via ctypes.cdll, not windll).
    for c in cands:
        try:
            return ctypes.cdll.LoadLibrary(c)
        except OSError as e:
            errs.append("%s: %s" % (c, e))
    raise OSError("WaveForms `dwf` library not found. Install WaveForms and/or "
                  "set DWF_LIB to the dwf library path.\nTried:\n  " +
                  "\n  ".join(errs))


dwf = _load_dwf()

# function + state constants (from Digilent dwfconstants)
funcSquare = ctypes.c_int(2)     # 1=Sine, 2=Square (was wrong)
funcCustom = ctypes.c_int(30)
AnalogOutNodeCarrier = ctypes.c_int(0)
DwfStateDone = 2
acqmodeSingle = ctypes.c_int(0)
acqmodeRecord = ctypes.c_int(3)  # 連続ストリーム（USB へ逐次転送、デバイスバッファ超の長尺可）
# DwfState（record ループのプリフィル判定用）
stsArmed, stsConfig, stsPrefill = 1, 4, 5


def version() -> str:
    v = ctypes.create_string_buffer(32); dwf.FDwfGetVersion(v); return v.value.decode()


def enum_devices():
    c = ctypes.c_int(); dwf.FDwfEnum(ctypes.c_int(0), ctypes.byref(c))
    out = []
    for i in range(c.value):
        name = ctypes.create_string_buffer(64); sn = ctypes.create_string_buffer(64)
        dwf.FDwfEnumDeviceName(ctypes.c_int(i), name); dwf.FDwfEnumSN(ctypes.c_int(i), sn)
        out.append((name.value.decode(), sn.value.decode()))
    return out


def last_error() -> str:
    e = ctypes.create_string_buffer(512); dwf.FDwfGetLastErrorMsg(e)
    return e.value.decode().strip()


class AD:
    """config: デバイス構成の番号。既定(None)は WaveForms の既定構成で開く。

    AD3 は構成によって各インストゥルメントのバッファ配分が変わる（実測 2026-09-09）:
        cfg 0 (既定): DigitalOut  2048 bit/ch, AnalogIn 16384
        cfg 4       : DigitalOut 32768 bit/ch, AnalogIn  4096
    多タグのランダム backoff（論文の 2^14 TU）は cfg 0 のバッファ 1024 TU に入らないため
    cfg 4 が要る。AnalogIn は 4096 に減るが、検波トリガは元々 nbuf=1024 で運用しており
    （PLAN.md ★0: nbuf=16384 だと再武装が間に合わない）実害は無い。
    構成一覧は FDwfEnumConfigInfo で確認できる。"""

    def __init__(self, ext_clock_hz=None, sys_clock_hz=100e6, config=None):
        # Clock-sync mode: lock the AD timebase to an external reference on the
        # TRIG1 pin (driven by the STM32 MCO). Per the SDK *_Devices.py samples
        # the correct sequence is:
        #   - BEFORE open (global):  ExtFreq(9)=ref Hz, Frequency(8)=system Hz
        #   - AFTER open (handle):   FDwfDeviceParamSet(ClockMode(10)=2 input)
        # Setting ClockMode via the global FDwfParamSet before open does NOT
        # take effect (the device stays on its internal clock) -- that was the
        # bug the ExtFreq-lie test exposed (Tb unchanged => not locked).
        if ext_clock_hz:
            dwf.FDwfParamSet(ctypes.c_int(9), ctypes.c_int(int(ext_clock_hz)))   # ExtFreq
            dwf.FDwfParamSet(ctypes.c_int(8), ctypes.c_int(int(sys_clock_hz)))   # system Frequency
        else:
            # 前プロセス(extref)のグローバル ExtFreq が残っていると、内部クロックの
            # つもりでも ext 設定で open され AnalogIn が固まる(取得が走らず stale
            # Done を返し続ける / state=0 で永久待ち)。open 前に必ずクリア。
            dwf.FDwfParamSet(ctypes.c_int(9), ctypes.c_int(0))                   # ExtFreq clear
        self.h = ctypes.c_int()
        if config is None:
            ok = dwf.FDwfDeviceOpen(ctypes.c_int(-1), ctypes.byref(self.h))
        else:
            # FDwfDeviceConfigOpen は「列挙済みのデバイス番号」を取るので、先に必ず
            # FDwfEnum を呼ぶこと（呼ばないと Invalid device index で落ちる）。
            n = ctypes.c_int()
            dwf.FDwfEnum(ctypes.c_int(0), ctypes.byref(n))
            if n.value == 0:
                raise RuntimeError("AD が見つかりません（FDwfEnum = 0 台）")
            ok = dwf.FDwfDeviceConfigOpen(ctypes.c_int(0), ctypes.c_int(int(config)),
                                          ctypes.byref(self.h))
        if ok == 0 or self.h.value == 0:
            raise RuntimeError("FDwfDeviceOpen failed: " + last_error() +
                               "  (is the AD connected, powered, and NOT open in the WaveForms GUI?)")
        # 前セッションの残留状態(OnClose=continue で走り続ける AWG 等)を必ず一掃する。
        # これ無しだと AWG がレート変更を黙って無視する固着状態に入ることがある
        # (2026-07-09 実測: wavegen_custom の周波数変更が効かず旧レートを再生し続けた)。
        dwf.FDwfDeviceReset(self.h)
        if ext_clock_hz:
            # Reset はデバイス側の周波数設定も飛ばすので、device-param で再適用してから
            # ClockMode=input にする(グローバル param は open 時にしか反映されない)。
            dwf.FDwfDeviceParamSet(self.h, ctypes.c_int(9), ctypes.c_int(int(ext_clock_hz)))
            dwf.FDwfDeviceParamSet(self.h, ctypes.c_int(8), ctypes.c_int(int(sys_clock_hz)))
            dwf.FDwfDeviceParamSet(self.h, ctypes.c_int(10), ctypes.c_int(2))    # ClockMode = input (TRIG1)
        else:
            # 内部クロックへ明示リセット。直前に ext-ref を使ったプロセスのグローバル
            # パラメータ(ClockMode/ExtFreq)が残留して内部運用を壊すのを防ぐ。
            dwf.FDwfDeviceParamSet(self.h, ctypes.c_int(10), ctypes.c_int(0))    # ClockMode = internal
            dwf.FDwfParamSet(ctypes.c_int(9), ctypes.c_int(0))                   # ExtFreq clear

    def close(self):
        dwf.FDwfDeviceCloseAll()

    # ---- W1 / W2 analog output ------------------------------------------
    def wavegen_square(self, ch, freq, vhi=3.3):
        c = ctypes.c_int(ch); n = AnalogOutNodeCarrier
        dwf.FDwfAnalogOutNodeEnableSet(self.h, c, n, ctypes.c_int(1))
        dwf.FDwfAnalogOutNodeFunctionSet(self.h, c, n, funcSquare)
        dwf.FDwfAnalogOutNodeFrequencySet(self.h, c, n, ctypes.c_double(freq))
        dwf.FDwfAnalogOutNodeAmplitudeSet(self.h, c, n, ctypes.c_double(vhi / 2))
        dwf.FDwfAnalogOutNodeOffsetSet(self.h, c, n, ctypes.c_double(vhi / 2))
        dwf.FDwfAnalogOutIdleSet(self.h, c, ctypes.c_int(1))     # idle = offset
        dwf.FDwfAnalogOutRepeatSet(self.h, c, ctypes.c_int(0))
        dwf.FDwfAnalogOutConfigure(self.h, c, ctypes.c_int(1))

    def wavegen_custom(self, ch, buf_pm1, play_freq, vhi=3.3):
        c = ctypes.c_int(ch); n = AnalogOutNodeCarrier
        data = (ctypes.c_double * len(buf_pm1))(*buf_pm1.astype(float))
        dwf.FDwfAnalogOutNodeEnableSet(self.h, c, n, ctypes.c_int(1))
        dwf.FDwfAnalogOutNodeFunctionSet(self.h, c, n, funcCustom)
        dwf.FDwfAnalogOutNodeDataSet(self.h, c, n, data, ctypes.c_int(len(buf_pm1)))
        dwf.FDwfAnalogOutNodeFrequencySet(self.h, c, n, ctypes.c_double(play_freq))
        dwf.FDwfAnalogOutNodeAmplitudeSet(self.h, c, n, ctypes.c_double(vhi / 2))
        dwf.FDwfAnalogOutNodeOffsetSet(self.h, c, n, ctypes.c_double(vhi / 2))
        dwf.FDwfAnalogOutRepeatSet(self.h, c, ctypes.c_int(0))
        dwf.FDwfAnalogOutConfigure(self.h, c, ctypes.c_int(1))

    def wavegen_custom_diff(self, buf_pm1, play_freq, vhi=3.3):
        """RFスイッチの相補制御用: W1(Ch0)=buf, W2(Ch1)=反転buf を 0..vhi V で
        位相同期再生。Ch1 を Ch0 の slave(MasterSet)にし、Ch0 の Configure 一発で
        両chが同時スタート(サンプル位相ロック)。buf は ±1 スケール。
        W1 = vhi/2 + (vhi/2)*buf, W2 = vhi/2 - (vhi/2)*buf → 常に W2 = vhi - W1。"""
        data = np.asarray(buf_pm1, dtype=float)
        n = AnalogOutNodeCarrier
        for ch, d in ((0, data), (1, -data)):
            c = ctypes.c_int(ch)
            arr = (ctypes.c_double * len(d))(*d)
            dwf.FDwfAnalogOutNodeEnableSet(self.h, c, n, ctypes.c_int(1))
            dwf.FDwfAnalogOutNodeFunctionSet(self.h, c, n, funcCustom)
            dwf.FDwfAnalogOutNodeDataSet(self.h, c, n, arr, ctypes.c_int(len(d)))
            dwf.FDwfAnalogOutNodeFrequencySet(self.h, c, n, ctypes.c_double(play_freq))
            dwf.FDwfAnalogOutNodeAmplitudeSet(self.h, c, n, ctypes.c_double(vhi / 2))
            dwf.FDwfAnalogOutNodeOffsetSet(self.h, c, n, ctypes.c_double(vhi / 2))
            dwf.FDwfAnalogOutRepeatSet(self.h, c, ctypes.c_int(0))
        dwf.FDwfAnalogOutMasterSet(self.h, ctypes.c_int(1), ctypes.c_int(0))
        dwf.FDwfAnalogOutConfigure(self.h, ctypes.c_int(0), ctypes.c_int(1))

    def wavegen_stop(self, ch):
        dwf.FDwfAnalogOutReset(self.h, ctypes.c_int(ch))

    def buffer_size_max(self):
        lo = ctypes.c_int(); hi = ctypes.c_int()
        dwf.FDwfAnalogInBufferSizeInfo(self.h, ctypes.byref(lo), ctypes.byref(hi))
        return hi.value

    # ---- scope capture ---------------------------------------------------
    def scope_capture(self, channel=0, fs=50e6, nsamples=8192, vrange=5.0, atten=1.0):
        """バッファ単発取得。atten=10 で ×10 プローブ補正(振幅を実電圧へ換算+レンジ適正化)。
        nsamples が大きすぎる場合はデバイス最大バッファにクランプ。"""
        c = ctypes.c_int(channel)
        nmax = self.buffer_size_max()
        if nmax and nsamples > nmax: nsamples = nmax
        dwf.FDwfAnalogInChannelEnableSet(self.h, c, ctypes.c_int(1))
        dwf.FDwfAnalogInChannelAttenuationSet(self.h, c, ctypes.c_double(atten))  # ×10プローブ
        dwf.FDwfAnalogInChannelRangeSet(self.h, c, ctypes.c_double(vrange))
        dwf.FDwfAnalogInFrequencySet(self.h, ctypes.c_double(fs))
        dwf.FDwfAnalogInBufferSizeSet(self.h, ctypes.c_int(nsamples))
        dwf.FDwfAnalogInAcquisitionModeSet(self.h, acqmodeSingle)
        got = ctypes.c_double(); dwf.FDwfAnalogInFrequencyGet(self.h, ctypes.byref(got))
        time.sleep(0.1)
        dwf.FDwfAnalogInConfigure(self.h, ctypes.c_int(1), ctypes.c_int(1))
        sts = ctypes.c_byte()
        deadline = time.time() + 5.0 + 2.0 * (nsamples / fs)
        while True:
            dwf.FDwfAnalogInStatus(self.h, ctypes.c_int(1), ctypes.byref(sts))
            if sts.value == DwfStateDone:
                break
            if time.time() > deadline:
                raise RuntimeError("scope_capture timeout (state=%d). AD の接続/電源を確認: %s"
                                   % (sts.value, last_error()))
            time.sleep(0.001)
        buf = (ctypes.c_double * nsamples)()
        dwf.FDwfAnalogInStatusData(self.h, c, buf, ctypes.c_int(nsamples))
        return np.array(buf, dtype=float), got.value

    def scope_capture2(self, fs=50e6, nsamples=8192, vrange=5.0):
        """Ch1+Ch2 同時単発取得(同一 acquisition = 同一時間軸)。W1/W2 相補出力の
        検証用。戻り値: (ch1, ch2, 実fs)。"""
        nmax = self.buffer_size_max()
        if nmax and nsamples > nmax: nsamples = nmax
        for ch in (0, 1):
            c = ctypes.c_int(ch)
            dwf.FDwfAnalogInChannelEnableSet(self.h, c, ctypes.c_int(1))
            dwf.FDwfAnalogInChannelAttenuationSet(self.h, c, ctypes.c_double(1.0))
            dwf.FDwfAnalogInChannelRangeSet(self.h, c, ctypes.c_double(vrange))
        dwf.FDwfAnalogInFrequencySet(self.h, ctypes.c_double(fs))
        dwf.FDwfAnalogInBufferSizeSet(self.h, ctypes.c_int(nsamples))
        dwf.FDwfAnalogInAcquisitionModeSet(self.h, acqmodeSingle)
        got = ctypes.c_double(); dwf.FDwfAnalogInFrequencyGet(self.h, ctypes.byref(got))
        time.sleep(0.1)
        dwf.FDwfAnalogInConfigure(self.h, ctypes.c_int(1), ctypes.c_int(1))
        sts = ctypes.c_byte()
        deadline = time.time() + 5.0 + 2.0 * (nsamples / fs)
        while True:
            dwf.FDwfAnalogInStatus(self.h, ctypes.c_int(1), ctypes.byref(sts))
            if sts.value == DwfStateDone:
                break
            if time.time() > deadline:
                raise RuntimeError("scope_capture2 timeout (state=%d). AD の接続/電源を確認: %s"
                                   % (sts.value, last_error()))
            time.sleep(0.001)
        out = []
        for ch in (0, 1):
            buf = (ctypes.c_double * nsamples)()
            dwf.FDwfAnalogInStatusData(self.h, ctypes.c_int(ch), buf, ctypes.c_int(nsamples))
            out.append(np.array(buf, dtype=float))
        return out[0], out[1], got.value

    def scope_record(self, channel=0, fs=8e6, nsamples=400000, vrange=5.0):
        """連続ストリーム取得（Record モード）。デバイスバッファに収まらない長尺を USB へ
        逐次転送して取得。戻り値: (data[nsamples], 実fs, lost, corrupt)。
        lost/corrupt>0 はそのレートが連続転送に追いつけていない（fs を下げる/短くする）。"""
        c = ctypes.c_int(channel)
        dwf.FDwfAnalogInChannelEnableSet(self.h, c, ctypes.c_int(1))
        dwf.FDwfAnalogInChannelRangeSet(self.h, c, ctypes.c_double(vrange))
        dwf.FDwfAnalogInAcquisitionModeSet(self.h, acqmodeRecord)
        dwf.FDwfAnalogInFrequencySet(self.h, ctypes.c_double(fs))
        dwf.FDwfAnalogInRecordLengthSet(self.h, ctypes.c_double(nsamples / fs))
        dwf.FDwfAnalogInTriggerSourceSet(self.h, ctypes.c_int(0))   # trigsrcNone=即時開始(待たない)
        got = ctypes.c_double(); dwf.FDwfAnalogInFrequencyGet(self.h, ctypes.byref(got))
        time.sleep(0.1)
        dwf.FDwfAnalogInConfigure(self.h, ctypes.c_int(0), ctypes.c_int(1))   # reconfig=0, start=1
        data = np.zeros(nsamples, dtype=float); pos = 0
        sts = ctypes.c_byte()
        cAvail = ctypes.c_int(); cLost = ctypes.c_int(); cCorrupt = ctypes.c_int()
        lost = corrupt = 0
        deadline = time.time() + 10.0 + 4.0*(nsamples/fs)          # ハング防止タイムアウト
        while pos < nsamples:
            if time.time() > deadline:
                raise RuntimeError("scope_record timeout (got %d/%d samples). "
                                   "fs を下げる/信号・トリガ確認。" % (pos, nsamples))
            if dwf.FDwfAnalogInStatus(self.h, ctypes.c_int(1), ctypes.byref(sts)) == 0:
                raise RuntimeError("FDwfAnalogInStatus failed: " + last_error())
            if pos == 0 and sts.value in (stsConfig, stsPrefill, stsArmed):
                time.sleep(0.0005); continue   # 開始前プリフィル中はスキップ
            dwf.FDwfAnalogInStatusRecord(self.h, ctypes.byref(cAvail),
                                         ctypes.byref(cLost), ctypes.byref(cCorrupt))
            if cLost.value:    lost += cLost.value;    pos += cLost.value   # タイミング維持
            if cCorrupt.value: corrupt += cCorrupt.value
            avail = cAvail.value
            if avail == 0:
                time.sleep(0.0005); continue
            if pos + avail > nsamples: avail = nsamples - pos
            chunk = (ctypes.c_double * avail)()
            dwf.FDwfAnalogInStatusData(self.h, c, chunk, ctypes.c_int(avail))
            data[pos:pos+avail] = np.frombuffer(chunk, dtype=float); pos += avail
        return data, got.value, lost, corrupt
