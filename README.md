# Packet Interval Modulation (PIM) for NFC

Simulator and hardware testbed for Packet Interval Modulation (PIM), a MAC scheme in which NFC tags send a
preamble and a postamble and encode their data in the interval between them. The simulator compares PIM with
NFC Type V Slotted ALOHA, and the hardware testbed runs PIM with emulated tags and a real NFC reader. Both write
logs in the same TSV format, so collision rates and throughput can be compared directly.

## Directory Structure

```
simulation/        # PIM / Slotted ALOHA simulator (see simulation/README.md)
├── common/        # Simulator core, channel, logger
├── pim/           # PIM protocol
└── slotted_aloha/ # Slotted ALOHA protocol (NFC Type V)
hardware/          # Hardware testbed (see hardware/README.md)
├── firmware/      # Reader firmware (Nucleo-L476RG + X-NUCLEO-NFC05A1)
└── host/          # Python scripts driving emulated tags with Analog Discovery 3
requirements.txt   # Python dependencies for both
```

- [simulation/README.md](simulation/README.md): how to configure and run the simulator, and the log format.
- [hardware/README.md](hardware/README.md): hardware setup, wiring, and measurement.
- [hardware/firmware/README.md](hardware/firmware/README.md): reader firmware details.

## Requirements

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

The simulator needs Python 3.9+, PyYAML, and NumPy. The hardware host needs Python 3.12+, NumPy, and Matplotlib,
plus non-Python tools listed in [hardware/README.md](hardware/README.md).

## Quick Start

```bash
# Simulation (from the repository root)
cp simulation/pim/config_template.yaml simulation/pim/config.yaml
python -m simulation.pim.run

# Hardware self-test (no hardware needed)
python3 hardware/host/pim.py
```

## License

This software is released under the BSD 3-clause license. See [LICENSE.txt](LICENSE.txt).

Copyright (c) 2026, Satoshi IDO, Shigemi ISHIDA, and Ryo TAKAHASHI
