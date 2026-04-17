from ..common.channel import Channel, Packet
from ..common.logger import EventLogger
from .tag import PimTag


class PimReader:
    def __init__(self, config: dict, channel: Channel, logger: EventLogger, round_num: int):
        self.config = config
        self.channel = channel
        self.logger = logger
        self.round_num = round_num
        self.data_bits = config["data_bits"]
        self.data_tu_mult = config["data_tu_mult"]
        self.backoff_max = config["backoff_max"]

    def run_round(self, tags: list[PimTag]) -> dict:
        stats = {"collisions": 0, "successes": 0}

        # Send start signal
        self.logger.log(0, self.round_num, "transmission", "start_signal",
                        "-", "-", "-")

        # Pre-compute all tag transmission intervals
        # Each tag sends: preamble (backoff ~ backoff+packet_bits),
        #                  then waits data_wait_tu,
        #                  then postamble
        tag_intervals = []
        for tag in tags:
            tag_intervals.append({
                "tag": tag,
                "preamble": (tag.preamble_start, tag.preamble_end),
                "postamble": (tag.postamble_start, tag.postamble_end),
            })

        # Log preamble events
        for ti in tag_intervals:
            tag = ti["tag"]
            ps, pe = ti["preamble"]
            self.logger.log(ps, self.round_num, "transmission", "tx_preamble",
                            tag.uid_hex, f"start={ps},end={pe},bits={tag.packet_bits}", "-")

        # Check preamble collisions: two preambles overlap if their intervals intersect
        preamble_collided = set()
        for i, ti in enumerate(tag_intervals):
            for j, tj in enumerate(tag_intervals):
                if i >= j:
                    continue
                s1, e1 = ti["preamble"]
                s2, e2 = tj["preamble"]
                if s1 < e2 and s2 < e1:  # overlap
                    preamble_collided.add(i)
                    preamble_collided.add(j)

        # Log postamble events and check postamble collisions
        postamble_collided = set()
        for i, ti in enumerate(tag_intervals):
            tag = ti["tag"]
            ps, pe = ti["postamble"]
            self.logger.log(ps, self.round_num, "transmission", "tx_postamble",
                            tag.uid_hex,
                            f"start={ps},end={pe},bits={tag.packet_bits},data={tag.data},wait={tag.data_wait_tu}",
                            "-")

        for i, ti in enumerate(tag_intervals):
            for j, tj in enumerate(tag_intervals):
                if i >= j:
                    continue
                s1, e1 = ti["postamble"]
                s2, e2 = tj["postamble"]
                if s1 < e2 and s2 < e1:
                    postamble_collided.add(i)
                    postamble_collided.add(j)

        # Also check cross-collision: preamble of one with postamble of another
        for i, ti in enumerate(tag_intervals):
            for j, tj in enumerate(tag_intervals):
                if i == j:
                    continue
                # ti's preamble vs tj's postamble
                s1, e1 = ti["preamble"]
                s2, e2 = tj["postamble"]
                if s1 < e2 and s2 < e1:
                    preamble_collided.add(i)
                    postamble_collided.add(j)
                # ti's postamble vs tj's preamble
                s1, e1 = ti["postamble"]
                s2, e2 = tj["preamble"]
                if s1 < e2 and s2 < e1:
                    postamble_collided.add(i)
                    preamble_collided.add(j)

        # Determine success/collision per tag
        for i, ti in enumerate(tag_intervals):
            tag = ti["tag"]
            pre_col = i in preamble_collided
            post_col = i in postamble_collided
            collided = pre_col or post_col

            if collided:
                detail_parts = []
                if pre_col:
                    detail_parts.append("preamble_collision")
                if post_col:
                    detail_parts.append("postamble_collision")
                self.logger.log(ti["postamble"][1], self.round_num, "transmission", "collision",
                                tag.uid_hex, ",".join(detail_parts), "collision")
                stats["collisions"] += 1
            else:
                self.logger.log(ti["postamble"][1], self.round_num, "transmission", "rx_success",
                                tag.uid_hex, f"data={tag.data}", "success")
                stats["successes"] += 1

        return stats
