import random


class PimTag:
    DELIMITER_PREAMBLE = 0xAA
    DELIMITER_POSTAMBLE = 0x55

    def __init__(self, index: int, config: dict):
        self.index = index
        self.id_bits = config["id_bits"]
        self.data_bits = config["data_bits"]
        self.delimiter_bits = config["delimiter_bits"]
        self.backoff_max = config["backoff_max"]
        self.data_tu_mult = config["data_tu_mult"]
        self.bits_per_tu = config["datarate"] * config["tu"]

        self.uid = random.getrandbits(self.id_bits)
        self.data = random.getrandbits(self.data_bits)
        self.backoff = random.randint(0, self.backoff_max - 1)

    @property
    def uid_hex(self) -> str:
        return format(self.uid, f"0{(self.id_bits + 3) // 4}x")

    @property
    def packet_bits(self) -> int:
        return self.delimiter_bits + self.id_bits

    @property
    def packet_tu(self) -> float:
        return self.packet_bits / self.bits_per_tu

    @property
    def data_wait_tu(self) -> float:
        return self.data * self.data_tu_mult

    @property
    def preamble_start(self) -> float:
        return float(self.backoff)

    @property
    def preamble_end(self) -> float:
        return self.preamble_start + self.packet_tu

    @property
    def postamble_start(self) -> float:
        return self.preamble_end + self.data_wait_tu

    @property
    def postamble_end(self) -> float:
        return self.postamble_start + self.packet_tu
