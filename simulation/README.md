# NFC Packet Interval Modulation (PIM) Simulator

A tool that simulates communication between multiple tags in an NFC environment and outputs logs necessary for evaluating collision rates and throughput. It allows for the comparison of two types of MAC protocols (Slotted ALOHA / PIM).

## Directory Structure

```
simulation/
├── common/                  # Common modules
│   ├── simulator.py         # Simulator Core (Time Management, Round Execution)
│   ├── channel.py           # Communication Channel (Packet Management, Collision Detection)
│   └── logger.py            # Event Log Output (TSV Format)
├── slotted_aloha/           # Slotted ALOHA Protocol
│   ├── config_template.yaml # Configuration File Template
│   ├── reader.py            # Reader (Inventory, Data Collection)
│   ├── tag.py               # Tag (Mask Determination, Slot Selection)
│   └── run.py               # Execution Entry Point
└── pim/                     # Packet Interval Modulation (PIM) Protocol
    ├── config_template.yaml # Configuration File Template
    ├── config_exp_dist_mult.yaml # Config for the data-distribution × redundancy experiment
    ├── reader.py            # Reader (Collision Detection & Data Decryption)
    ├── tag.py               # Tag (Backoff & Pre/Post-Amble Generation)
    └── run.py               # Execution Entry Point
```

## Requirements

- Python 3.9+
- PyYAML
- NumPy

## How to Run

Prepare `config.yaml` as described below and run the simulation from the repository root:
```bash
# Slotted ALOHA
python -m simulation.slotted_aloha.run

# PIM
python -m simulation.pim.run

# PIM with another config file
python -m simulation.pim.run simulation/pim/config_exp_dist_mult.yaml
```

## Configuration Parameters

Copy `config_template.yaml` to `config.yaml` in each protocol directory (e.g. `simulation/pim/config.yaml`) and modify the parameters.

### Common Parameters

| Parameter | Description |
|--|--|
| `datarate` | Data rate [Hz] |
| `tu` | Time unit [s] |
| `max_tags` | Number of tags generated; `nodes` tags are selected from them each round |
| `nodes` | List of nodes to simulate |
| `data_bits` | Number of data bits |
| `id_bits` | Tag ID length [bits] |
| `num_rounds` | Number of rounds |
| `random_seed` | Random seed |
| `reseed_per_nodes` | (Optional) Restart the random generator from `random_seed` for each number of nodes, so that each round draws the same values as the hardware run with the same `--seed` |

### Slotted ALOHA Specific

| Parameter | Description |
|--|--|
| `num_slots` | Number of inventory slots |
| `eof_bits` | EOF signal length [bits] |
| `reader_cmd_bytes` | Reader command length [bytes] |
| `tag_overhead_bytes` | Tag response overhead [bytes] |

### PIM-Specific

| Parameter | Description |
|--|--|
| `backoff_max` | Maximum random backoff [TU]. A list sweeps the values |
| `delimiter_bits` | Delimiter length [bit] |
| `data_tu_mult` | Multiple of TU for data waiting time (redundancy). A list sweeps the values |
| `data_dist` | Distribution of data values: `uniform` (1-255) or `gauss` (N(`data_mean`, `data_std`) rounded and clipped to 1-255). A list sweeps the values |
| `data_mean` / `data_std` | Mean and standard deviation for `data_dist: gauss` |
| `preamble_tu` | (Optional) Pre-amble length [TU]. If omitted, `(delimiter_bits + id_bits)` is used |
| `postamble_tu` | (Optional) Post-amble length [TU]. If omitted, `(delimiter_bits + id_bits)` is used |

When `backoff_max`, `data_tu_mult`, or `data_dist` is a list, every combination is simulated and written to `logs/pim_<YYYYMMDD_HHMMSS>/<combination>/` (e.g. `gauss_mult2`, `backoff_16384`). Each subdirectory has the same format as a single-value run.

## Protocol Overview

### Slotted ALOHA (NFC Type V Compliant)

1. **Inventory Phase**: The reader transmits an inventory command (specifying mask length, mask value, and number of slots = 16). Tags respond in the slot determined by the 4 bits immediately above the mask if the lower bits of their UID match the mask. In the event of a collision, the mask length is extended by 4 bits to resolve the conflict recursively.
2. **Data Collection Phase**: The reader sequentially sends data requests to recognized tags using address specification.

### PIM (Packet Interval Modulation)

1. The reader transmits a start signal.
2. Each tag transmits a preamble (delimiter + ID) after a random backoff.
   The hardware implementation has no delimiter byte; it distinguishes the pre-amble and post-amble by the length of a solid-ON SOF, so the two pulses are 21 TU and 25 TU long. Set `preamble_tu: 21` and `postamble_tu: 25` to simulate the same pulse lengths.
3. Wait for a duration proportional to the data value, then transmit a post-amble (delimiter + ID).
4. The interval between the pre-amble and post-amble represents the data.

## Log Output

A `logs/<protocol>_<YYYYMMDD_HHMMSS>/` directory is generated in the current directory for each execution.

| File | Content |
|--|--|
| `events_<N>.tsv` | Complete event log for `N` nodes (TSV format) |
| `config.yaml` | Copy of runtime configuration |
| `summary.txt` | Summary of collisions and successes by number of nodes |

### Columns in events_<N>.tsv

| Column | Description |
|--|--|
| `time_tu` | Event occurrence time [TU] |
| `round` | Round number |
| `phase` | Phase (`inventory` / `data_collection` / `transmission`) |
| `event` | Event type |
| `tag_id` | Target tag UID (hexadecimal) |
| `detail` | Detailed information |
| `result` | Result (`success` / `collision` / `empty` / `-`) |

### Major Event Types

**Slotted ALOHA:**
`inventory_cmd`, `tag_response`, `collision`, `slot_empty`, `eof`, `unresolvable`, `data_request`, `data_response`

**PIM:**
`start_signal`, `tx_preamble`, `tx_postamble`, `rx_success`, `collision`
