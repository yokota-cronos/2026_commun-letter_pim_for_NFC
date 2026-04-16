from dataclasses import dataclass, field


@dataclass
class Packet:
    sender_id: int
    start_tu: int
    end_tu: int
    kind: str  # "inventory_cmd", "eof", "tag_response", "data_request", "data_response", "preamble", "postamble", "start_signal"


class Channel:
    def __init__(self):
        self._active_packets: list[Packet] = []
        self._collided_ids: set[int] = set()

    def submit(self, packet: Packet):
        self._active_packets.append(packet)

    def get_active_packets(self, current_tu: int) -> list[Packet]:
        return [p for p in self._active_packets if p.start_tu <= current_tu < p.end_tu]

    def check_collision(self, current_tu: int) -> tuple[list[Packet], bool]:
        active = self.get_active_packets(current_tu)
        tag_packets = [p for p in active if p.sender_id >= 0]  # reader has id -1
        collision = len(tag_packets) > 1
        if collision:
            for p in tag_packets:
                self._collided_ids.add(id(p))
        return active, collision

    def is_collided(self, packet: Packet) -> bool:
        return id(packet) in self._collided_ids

    def cleanup(self, current_tu: int):
        self._active_packets = [p for p in self._active_packets if p.end_tu > current_tu]
        self._collided_ids = {pid for pid in self._collided_ids
                              if any(id(p) == pid for p in self._active_packets)}

    def clear(self):
        self._active_packets.clear()
        self._collided_ids.clear()
