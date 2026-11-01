# firmware — 424 kHz PIM Reader (Ring-Buffer, Continuous Capture)

> **Board: X-NUCLEO-NFC05A1 (ST25R3911B) only.** It does not run on X-NUCLEO-NFC08A1 (ST25R3916B)
> because the register addresses differ.

This firmware runs on a Nucleo-L476RG with an X-NUCLEO-NFC05A1 expansion board. It keeps the
ST25R3911B in Transparent mode, **captures continuously through a ring buffer**, and decodes PIM
packets on the MCU. The decoder (running-sum IQ, SOF detection, Manchester decoding, and greedy
preamble/postamble pairing for multiple tags) is the same as in our earlier single-snapshot firmware.

> **Status: verified on hardware (2026-09-14).** `host/pim_multitag.py link --random-data --random-backoff`
> with randomized data and backoff and no backoff sweep gave `一致=23 誤り=0` (23 matches, 0 errors)
> with `--reader-sps 23700`. After the first flash, check the reader with the steps in §5.

---

## 1. Why a ring buffer

The single-snapshot firmware captured **only 13.1 ms** after each field OFF→ON and then decoded that
window. A detector-triggered tag (`link`/`run`) that sends once per field cycle was lost if its
transmission fell outside the window, so the backoff had to be tuned to fit the window.

This firmware runs SPI1 + DMA in circular mode without stopping, and the main loop follows the DMA
and decodes. **There is no fixed window, so a tag is captured whenever it transmits.**

## 2. How it works

```
ST25R3911B (Transparent) --AM bit--> SPI1 RX --circular DMA--> g_ring[32768 words] (always overwritten)
                                                                   │
          Stage 1 (every 64 words, light): count bit transitions ──┘
            many transitions while a subcarrier is present, almost 0 otherwise
                         │ activity
                         ▼
          Open a segment → close it after silence longer than the longest interval (255×mult TU)
                         │ (one cycle = preamble to postamble, captured whole)
                         ▼
          Stage 2 (per packet, heavy): copy linearly into g_seg[] → decode → print on UART
```

- Stage 1 runs faster than real time, so it catches up after a temporary delay from printing or decoding.
- If the reader falls more than one ring lap behind, it skips to the newest data and counts `overruns`
  (it never silently reads stale data).
- Ring laps are counted in the DMA transfer-complete interrupt ([ring_isr.cpp](pim_ring424k/ring_isr.cpp)).
  It is in a separate file because, in a `.ino`, arduino-builder auto-generates a C++-linkage prototype
  that conflicts with it.

## 3. Differences from the single-snapshot firmware

| | Single-snapshot | This firmware |
|---|---|---|
| Capture | One 13.1 ms shot per field ON | Continuous (circular DMA), from boot |
| Default capture rate | 40 Msps (sps ≈ 94.4) | **10 Msps (sps ≈ 23.6)**; change with `rate` |
| Decoding unit | Whole window | One cycle in which activity is detected |
| Output | One line per cycle (even with no packet) | **Only segments that contain a SOF** |
| `auto on` | Repeats OFF→ON→capture→decode | Only toggles the field OFF→ON periodically (capture continues); same as `cycle on` |
| Backoff tuning | Needed (to fit the window) | Not needed (but the first ~100 µs after field ON is not captured, §8) |
| `sps_milli=0` | Fixed to the value computed at the first decode | Not written back (follows `rate` changes) |
| SWD `g_capture` | Yes | **No** (`g_ring` is circular and cannot be analyzed linearly) |
| RAM (of 96 KB) | about 71% | about 80% (64 KB ring + 8 KB segment) |

**Why 10 Msps by default:** at 40 Msps, decoding one segment takes about as long as one ring lap
(13.1 ms), so signals that arrive during decoding can be lost. At 10 Msps, one ring lap is 52 ms and the
decoding work is 1/4. The round-trip self-test in `host/pim.py` passes at sps = 16, so sps ≈ 23.6 has
a 1.5× margin.

## 4. Build, flash, and console

```bash
cd hardware/firmware
./build.sh      # → build/pim_ring424k.ino.elf
./flash.sh      # flash over SWD
./console.sh    # open the console with kermit at 115200
```

Requirements: `arduino-cli` with the `STMicroelectronics:stm32` core, `openocd` (Homebrew or xPack;
`flash.sh` finds either), and `kermit`. See [../README.md](../README.md) for setup.

## 5. First check on hardware

1. After flashing, the boot log shows `[init] field ON + Transparent + continuous capture.`
2. `stat` → `listen=1`, `fs=10000000Hz`, `sps=23.600(auto)`, and `blocks` keeps increasing.
3. Run `stat` a few times with the tag idle → `block transitions: max` is small (0 to a few).
   If it is large, silence is detected as activity; raise `thr`.
4. Make the tag free-run (`host/pim_multitag.py verify`) → `[rx #..] tags=1 tag_id=.. data=..` lines appear.
   An increasing `truncated` count in `stat` is normal in free-run (§8).
5. Detector-triggered mode (`host/pim_multitag.py link`, **without `--reader-sps`**) → packets are
   received without sweeping the backoff. **This is the purpose of this firmware.** If not, check
   `overruns` and `last_decode` in `stat`.
   (This checks SOF detection and the backoff. To match the data values exactly, the
   `--reader-sps 23700` calibration in §7 is also needed.)

**When evaluating with randomized data and backoff** (`link --random-data --random-backoff`), add
`--reader-sps 23700`. With `sps auto`, only rounds with large data (a long preamble-to-postamble
interval) are systematically off by +1 TU and counted as errors (see §7).

```bash
python3 host/pim_multitag.py link --random-data --random-backoff --seed 42 \
  --reader-gap 150 --seconds 60 --tsv --reader-sps 23700
```

## 6. Console commands

| Command | Meaning |
|---|---|
| `listen on` / `listen off` | Start/stop continuous capture and decoding (on at boot) |
| `cycle on` / `cycle off` (= `auto on/off`) | Toggle the field OFF→ON every `period`, for detector-triggered tags. Capture stops only while the field is OFF |
| `run` | Toggle the field OFF→ON once now (keeps listening) |
| `period <ms>` / `gap <ms>` | Cycle period (default 200) / field-OFF time (default 20) |
| `rate <0..3>` | Capture rate 0=40M 1=20M **2=10M (default)** 3=5M sps |
| `thr <n>` | Stage-1 activity threshold [transitions/block] (default 4) |
| `mult <n>` | data_tu_mult (also sets the silence length that closes a segment) |
| `sps auto` / `sps <n>` | sps×1000. `auto` computes it from the capture rate. Warns if it is off by more than ±10% |
| `quiet <ms>` / `quiet off` | Restart the receive front end when there has been no activity for `<ms>` (default off; the host sends 2×period+50) |
| `stat` | Scanned blocks, decoded segments, overruns, truncated, last decode time, transition counts, and timing statistics |
| `bringup` `field on/off` `tpm` `tpx` `meas` `dump` `r` `w` | Low-level commands. **They access registers, so continuous capture stops** (restart with `listen on`) |

## 7. Host compatibility and notes

- `verify` and `link` in `host/pim_multitag.py` work as they are (the `[rx #N] sof= tags=` and
  `tag_id= data= interval=` formats are the same as before, and `auto on` / `sps` / `gap` / `period` are accepted).
- **`--reader-sps 94800` is the calibration for 40 Msps.** At 10 Msps, **`--reader-sps 23700` was
  measured to give zero errors** (2026-09-14, almost exactly 1/4 of 94800). At 5 Msps (`rate 3`), use
  `--reader-sps 11835`. Without it, the reader uses `sps auto` (the nominal value), and rounds with larger
  data (longer intervals) are systematically off by +1 TU, because the actual clock of the STM32 board
  differs slightly from the nominal one. The host's hint to type `sps 94400` assumes 40 Msps; ignore it
  with this firmware and use `--reader-sps`.
- Segments without packets are not printed, so the host's `frame` count and `sof/frame` mean something
  different from the single-snapshot firmware. Look at `一致` (match) and `誤り` (error) instead.
- `g_pim.trigger=1` over SWD only decodes the newest 4096 words of the ring and writes the result to
  `g_pim`/`g_tags`.

## 8. Known limitations

- **During field cycling, there is a gap of about 100 µs between field ON and the start of capture**
  (the time to re-enter Transparent mode over SPI). The detector circuit rises more slowly, so this should
  not matter in practice, but a backoff of a few tens of TU is safer than 0. There is no upper limit
  (the old 13.1 ms window is gone).
- **If activity never stops (free-run transmission or constant noise)**, segments are cut at 4096 words
  and pairs that span the cut are lost (`truncated`). This does not happen with one transmission per cycle.
- Maximum cycle length in one segment: 4096 words, i.e. mult ≤ 2 at 40 Msps and mult ≤ about 12 at 10 Msps.
- Stage 1 stops while stage 2 decodes. If decoding takes longer than one ring lap, `overruns` increases
  and signals during that time are lost. Check `last_decode` in `stat`.

## 9. 50 ms transmission period (2026-09-28)

- **Build**: `-O2` (FQBN `opt=o2std` in `build.sh`) and a 2048-byte UART transmit buffer
  (`pim_ring424k/build_opt.h`). With the default `-Os`, decoding one segment took 15 ms on average and
  could not keep up with a 50 ms period. With the default 64-byte buffer, printing one `[rx]` line blocked
  for about 4 ms and stopped scanning. After changing `build_opt.h`, run `rm -rf build` before `./build.sh`,
  because the core must be rebuilt with the same flags.
- **Faster decoding** (same results as before): fixed-point envelope, skipping silent words, and an
  amplitude cache for the ID search. One segment went from 15.2 ms to 2.6 ms. ID timing uses the center of
  the plateau, which cuts the interval jitter to 1/3.
- **Output**: `[rx #n] sof= tags= t=<us>` and `t=<us>` at the end of each tag line (the reader's
  `micros()` at the start of the preamble). The host (`pim_multitag.py link --free-run`) uses this to assign
  rounds. Resyncs are printed as `[resync] why=stuck|periodic|quiet t=<us>`.
- **Extra `stat` lines**: segment count and length, `decode avg/max`, `print avg/max`, `busy`, decode
  breakdown, `scan lag max`, and `quiet_resyncs`. When shortening the period, watch `busy` and `scan lag`
  (an overrun happens when the lag reaches the 32768-word ring).

## 10. Files

| File | Role |
|---|---|
| [pim_ring424k/pim_ring424k.ino](pim_ring424k/pim_ring424k.ino) | Base: clock, `setup`/`loop`, SPI register access, `g_pim` (fields from [16] on are for continuous capture), buffer definitions |
| [pim_ring424k/pim.ino](pim_ring424k/pim.ino) | Main part: bring-up/Transparent, ring capture, stage-1 activity detection, stage-2 decoding, field cycling, console |
| [pim_ring424k/ring_isr.cpp](pim_ring424k/ring_isr.cpp) | DMA transfer-complete interrupt (ring lap counter) |
| [pim_ring424k/pim_phy.h](pim_ring424k/pim_phy.h) | PHY constants generated from `host/pim.py`. Do not edit by hand; keep it consistent with `host/pim.py` |
| [build.sh](build.sh) / [flash.sh](flash.sh) / [console.sh](console.sh) | Build / SWD flash / console |
