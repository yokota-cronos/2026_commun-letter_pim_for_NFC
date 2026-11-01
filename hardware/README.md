# PIM Hardware Testbed

Hardware implementation of Packet Interval Modulation (PIM) over NFC (13.56 MHz carrier, fc/32 ≈ 424 kHz subcarrier).
Up to 16 emulated tags are driven by the digital outputs of a Digilent Analog Discovery 3 (AD3), and a reader
built on an ST25R3911B decodes their packets. The host records each round in the same TSV format as the
simulator in [../simulation/](../simulation/), so measured and simulated results can be compared directly.

## Directory Structure

```
firmware/                    # Reader firmware (Nucleo-L476RG + X-NUCLEO-NFC05A1)
├── pim_ring424k/            # Arduino sketch: continuous ring-buffer capture + on-MCU PIM decoding
├── build.sh                 # Build with arduino-cli
├── flash.sh                 # Flash over SWD with openocd
├── console.sh               # Open the reader console with kermit
└── README.md                # Firmware details (operation, console commands, limitations)
host/                        # Host-side Python scripts
├── pim.py                   # PIM physical layer (pulse structure, waveform generation, decoding)
├── pim_multitag.py          # Drives N emulated tags with AD3 DIO and checks the reader's results
├── pim_tsvlog.py            # Writes communication logs in the simulator's TSV format
├── pim_tune.py              # Shows the tag's load-modulation depth in real time (for tuning)
├── dwf_ad.py                # Minimal ctypes wrapper for the WaveForms dwf library
└── requirements.txt         # Host dependencies
```

## Setup

The commands below are run from this `hardware/` directory unless noted otherwise.

| Purpose | Requirement | Tested version |
|--|--|--|
| Host scripts | Python 3.12+, NumPy, Matplotlib | Python 3.12, NumPy 2.2.6, Matplotlib 3.10.9 |
| Driving AD3 | [Digilent WaveForms](https://digilent.com/reference/software/waveforms/waveforms-3/start) runtime (`host/dwf_ad.py` finds the dwf library on macOS / Windows / Linux; set `DWF_LIB` to override) | — |
| Building the firmware | `arduino-cli` + `STMicroelectronics:stm32` core | arduino-cli 1.5.1, stm32 2.12.0 |
| Flashing | `openocd` (Homebrew or xPack) | 0.12.0 |
| Reader console | `kermit` | — |

```bash
pip install -r host/requirements.txt     # or the top-level requirements.txt
```

Installing the STM32 core:

```bash
arduino-cli config set board_manager.additional_urls https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
arduino-cli core update-index
arduino-cli core install STMicroelectronics:stm32@2.12.0
```

### OS notes

- The host talks to the reader's UART with `termios`, looking for `/dev/cu.usbmodem*` (macOS) or
  `/dev/ttyACM*` (Linux). On Linux, add your user to the `dialout` group
  (`sudo usermod -aG dialout $USER`, then log in again). Windows is not supported.
- Pass `--port` to choose the reader's serial port explicitly.

## Wiring

| Signal | Connected to |
|--|--|
| AD3 DIO0..15 | CTRL (MOSFET gate) of each emulated tag |
| AD3 Scope CH1 | Envelope-detector output (DC), used as the trigger for the reader's field ON. Not used with `--free-run` |
| AD3 Scope CH2 | Raw antenna waveform, used by the `carrier` mode to measure the 13.56 MHz carrier |
| Reader | Nucleo-L476RG + X-NUCLEO-NFC05A1 over USB (ST-Link VCP for the console, SWD for flashing) |

Connect the grounds of 1-/2- and the AD3 to the tags and the Nucleo. If they float, the MOSFETs can switch unexpectedly.

## How to Run

### 1. Build and flash the reader

```bash
firmware/build.sh
firmware/flash.sh
```

See [firmware/README.md](firmware/README.md) for checking the reader on its console.

### 2. Self-test without hardware

```bash
python3 host/pim.py            # PHY generate → decode loopback
python3 host/pim_tsvlog.py     # TSV logging
```

### 3. Tune the tag coupling

```bash
python3 host/pim_tune.py --pins 0+2      # load-modulation depth for DIO0 and DIO2
python3 host/pim_tune.py --scan          # find which DIO pins are wired to a gate
```

Move the coil or adjust the tuning capacitor to maximize the bar. Aim for a difference of at least 10 LSB.

### 4. Multi-tag measurement

```bash
python3 host/pim_multitag.py link --free-run --random-data --random-backoff --seed 42 \
  --reader-period 100 --dio-config 4 --reader-rate 3 --reader-sps 11835 \
  --tags "0:40:250, 2:42:250, 3:43:250, 4:44:250" --seconds 1800 --tsv
```

| Option | Meaning |
|--|--|
| `--tags "dio:id:data[:backoff], ..."` | Emulated tags: DIO pin, tag ID, data, and optional backoff [TU] |
| `--free-run` | Do not use the detector; the host sends every `--reader-period` ms and the reader keeps the field ON |
| `--random-data` / `--random-backoff` | Draw new data (1..255) and backoff (`[0, --backoff-max)`, default 16384 TU) for each tag every round |
| `--data-dist uniform\|gauss` | Distribution of `--random-data` |
| `--seed` / `--sim-max-tags` | Random seed and the simulator's `max_tags`, used to draw random numbers in the same order as the simulator |
| `--reader-rate 0..3` | Reader capture rate 40M / 20M / 10M / 5M sps |
| `--reader-sps` | Calibrated samples per TU ×1000 (23700 for 10 Msps, 11835 for 5 Msps). Without it, data drifts by +1 TU for long intervals |
| `--dio-config` | AD3 device configuration (4 gives 32768 bits/channel for DigitalOut, chosen automatically with `--random-backoff`) |
| `--seconds` / `--target-packets` | Stop after this time, or after this many usable packets |
| `--tsv [PATH]` | Save the log; the default path is `log/YYYYMMDD_HHMMSS/YYYYMMDD_HHMMSS.tsv` |

Other modes of `pim_multitag.py`: `info` (AD3 capabilities), `detector` (detector trigger on CH1), `carrier` (carrier amplitude on CH2),
`selftest` (PC trigger), `verify` (free-run decoding check), and `run` (transmit only). Run
`python3 host/pim_multitag.py -h` for all options.

## Log Output

`--tsv` writes the same columns as the simulator's `events_<N>.tsv`
(`time_tu`, `round`, `phase`, `event`, `tag_id`, `detail`, `result`; see [../simulation/README.md](../simulation/README.md)).
As in the simulator, a tag whose packets overlap another tag's in the transmission schedule is counted as a
`collision` even if it was decoded. Hardware-only events are:

| Event | Meaning |
|--|--|
| `rx_miss` | No overlap, but the reader did not receive the tag |
| `rx_error` | No overlap and decoded, but the data value was wrong |
| `tx_dropped` | Drawn for the round but not sent because it did not fit in the AD3 buffer (excluded from the totals) |

Exclude rounds whose `start_signal` has `sync=0` in `detail` when computing statistics. In those rounds the host's clock was not synchronized with the reader's, so received packets could not be assigned to the round and would only appear as `rx_miss`.

## Matching the Simulator

- **Pulse length**: the hardware sends a preamble of 21 TU (4 TU SOF + 1 TU gap + 16 TU Manchester ID) and
  a postamble of 25 TU (8 TU SOF + 1 TU gap + 16 TU ID). The SOF length tells the preamble and postamble apart,
  so there is no separate delimiter byte. Set `preamble_tu: 21` and `postamble_tu: 25` in the simulator config
  (already set in `config_template.yaml`).
- **TU**: both use 1 TU = one subcarrier period = 32 / 13.56 MHz ≈ 2.36 µs (`tu`, `datarate: 211875` in the config).
- **Backoff and redundancy**: use the same `backoff_max` (`--backoff-max`, default 16384 TU) and `data_tu_mult` (`--mult`).
- **Random draws**: the host uses numpy `default_rng(seed)` and selects N tags from a pool of `max_tags` in the
  same way as the simulator. With `reseed_per_nodes: true` in the config and the same `--seed` / `--sim-max-tags`,
  round k draws the same data and backoff in both (except tags recorded as `tx_dropped`).
- **Data distribution**: `--data-dist gauss` on the host corresponds to `data_dist: gauss` in the config.
