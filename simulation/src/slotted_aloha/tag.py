import random


class SlottedAlohaTag:
    def __init__(self, index: int, config: dict):
        self.index = index
        self.id_bits = config["id_bits"]
        self.data_bits = config["data_bits"]
        self.uid = random.getrandbits(self.id_bits)
        self.data = random.getrandbits(self.data_bits)
        self.recognized = False

    @property
    def uid_hex(self) -> str:
        return format(self.uid, f"0{(self.id_bits + 3) // 4}x")

    def check_mask(self, mask_length: int, mask_value: int) -> bool:
        if mask_length == 0:
            return True
        lower_bits = self.uid & ((1 << mask_length) - 1)
        return lower_bits == mask_value

    def select_slot(self, mask_length: int) -> int:
        slot_bits = (self.uid >> mask_length) & 0xF
        return slot_bits

    def response_bits(self) -> int:
        return self.data_bits + 3 * 8  # data + 3byte overhead
