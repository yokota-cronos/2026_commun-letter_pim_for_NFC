# NFC Packet Interval Modulation (PIM) Simulator

A tool that simulates communication between multiple tags in an NFC environment and outputs logs necessary for evaluating collision rates and throughput. It allows for the comparison of two types of MAC protocols (Slotted ALOHA / PIM).

## Directory Structure

```
src/
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
    ├── reader.py            # Reader (Collision Detection & Data Decryption)
    ├── tag.py               # Tag (Backoff & Pre/Post-Amble Generation)
    └── run.py               # Execution Entry Point
```

## Requirements

- Python 3.9+
- PyYAML
- NumPy

## How to Run

Prepare for `config.yaml` as described below and execute simulation as:
```bash
# Slotted ALOHA
python -m src.slotted_aloha.run

# PIM
python -m src.pim.run
```

## Configuration Parameters

Copy `config_template.yaml` to `config.yaml` and modify parameters for each protocol.

### Common Parameters

| Parameter | Description |
|--|--|--|
| `datarate` | Data rate [Hz] |
| `nodes` | List of nodes to simulate |
| `data_bits` | Number of data bits |
| `id_bits` | Tag ID length [bits] |
| `num_rounds` | Number of rounds |

### Slotted ALOHA Specific

| Parameter | Description |
|--|--|--|
| `num_slots` | Number of inventory slots |
| `eof_bits` | EOF signal length [bits] |
| `reader_cmd_bytes` | Reader command length [bytes] |
| `tag_overhead_bytes` | Tag response overhead [bytes] |

### PIM-Specific

| Parameter | Description |
|--|--|--|
| `backoff_max` | Maximum random backoff [TU] |
| `delimiter_bits` | Delimiter length [bit] |
| `data_tu_mult` | Multiple of TU for data waiting time |

## Protocol Overview

### Slotted ALOHA (NFC Type V Compliant)

1. **Inventory Phase**: The reader transmits an inventory command (specifying mask length, mask value, and number of slots = 16). Tags respond in the slot determined by the 4 bits immediately above the mask if the lower bits of their UID match the mask. In the event of a collision, the mask length is extended by 4 bits to resolve the conflict recursively.
2. **Data Collection Phase**: The reader sequentially sends data requests to recognized tags using address specification.

### PIM (Packet Interval Modulation)

1. The reader transmits a start signal.
2. Each tag transmits a preamble (delimiter + ID) after a random backoff.
3. Wait for a duration proportional to the data value, then transmit a post-amble (delimiter + ID).
4. The interval between the pre-amble and post-amble represents the data.

## Log Output

A `logs/<protocol>_<YYYYMMDD_HHMMSS>/` directory is generated for each execution.

| File | Content |
|--|--|
| `events.tsv` | Complete event log (TSV format) |
| `config.yaml` | Copy of runtime configuration |
| `summary.txt` | Summary of collisions and successes by number of nodes |

### Columns in events.tsv

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
