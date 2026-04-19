from ..common.channel import Channel, Packet
from ..common.logger import EventLogger
from .tag import SlottedAlohaTag


class SlottedAlohaReader:
    def __init__(self, config: dict, channel: Channel, logger: EventLogger, round_num: int):
        """
        SlottedAlohaリーダの初期化
        
        Args:
            config: リーダの設定情報（num_slots、datarate等を含む）
            channel: 通信チャネル
            logger: イベントログ出力用のロガー
            round_num: 現在のラウンド番号
        """
        self.config = config
        self.channel = channel
        self.logger = logger
        self.round_num = round_num
        self.num_slots = config["num_slots"]
        self.eof_bits = config["eof_bits"]
        self.reader_cmd_bits = config["reader_cmd_bytes"] * 8
        self.tag_overhead_bits = config["tag_overhead_bytes"] * 8
        self.data_bits = config["data_bits"]
        self.bits_per_tu = config["datarate"] * config["tu"]
        self.tu = 0.0

    def _bits_to_tu(self, bits: float) -> float:
        """
        ビット数をTime Unit（TU）に変換
        
        Args:
            bits: 変換するビット数
            
        Returns:
            対応するTime Unit数
        """
        return bits / self.bits_per_tu

    def run_round(self, tags: list[SlottedAlohaTag]) -> dict:
        """
        1ラウンドの処理を実行
        
        タグの在庫確認とデータ収集を順序立てて実行する
        
        Args:
            tags: スロット内のタグ一覧
            
        Returns:
            衝突数と成功数を含む統計情報
        """
        stats = {"collisions": 0, "successes": 0}

        # Phase 1: Inventory
        recognized_tags = self._inventory(tags, mask_length=0, mask_value=0, stats=stats)

        # Phase 2: Data collection
        self._data_collection(recognized_tags, stats)

        return stats

    def _inventory(self, tags: list[SlottedAlohaTag], mask_length: int, mask_value: int,
                   stats: dict) -> list[SlottedAlohaTag]:
        """
        タグの在庫確認処理（タグの識別と認識）
        
        Slotted Alohaプロトコルに従い、マスク値に一致するタグを段階的に識別する。
        衝突が発生した場合は再帰的に解決する。
        
        Args:
            tags: 在庫確認対象のタグ一覧
            mask_length: 現在のマスク長
            mask_value: 現在のマスク値
            stats: 統計情報を格納する辞書
            
        Returns:
            新たに認識されたタグの一覧
        """
        recognized = []
        id_bits = self.config["id_bits"]

        # mask_lengthがID長に達したらこれ以上区別不可能
        if mask_length >= id_bits:
            unresolved = [t for t in tags if not t.recognized and t.check_mask(mask_length, mask_value)]
            for t in unresolved:
                self.logger.log(self.tu, self.round_num, "inventory", "unresolvable",
                                t.uid_hex, f"mask_len={mask_length}", "collision")
                stats["collisions"] += 1
            return recognized

        # Send inventory command
        mask_hex = format(mask_value, f"0{(mask_length + 3) // 4}x") if mask_length > 0 else "-"
        self.logger.log(self.tu, self.round_num, "inventory", "inventory_cmd",
                        "-", f"mask_len={mask_length},mask={mask_hex},slots={self.num_slots}", "-")

        self.tu += self._bits_to_tu(self.reader_cmd_bits)

        # Determine which tags respond and their slot assignments
        responding_tags = [t for t in tags if not t.recognized and t.check_mask(mask_length, mask_value)]
        slot_map: dict[int, list[SlottedAlohaTag]] = {}
        for tag in responding_tags:
            slot = tag.select_slot(mask_length)
            slot_map.setdefault(slot, []).append(tag)

        collision_slots: list[tuple[int, int]] = []  # (slot_num, mask_value_for_drill)

        for slot_num in range(self.num_slots):
            tags_in_slot = slot_map.get(slot_num, [])

            if len(tags_in_slot) == 0:
                self.logger.log(self.tu, self.round_num, "inventory", "slot_empty",
                                "-", f"slot={slot_num}", "empty")
            elif len(tags_in_slot) == 1:
                tag = tags_in_slot[0]
                response_duration = tag.response_bits()
                self.logger.log(self.tu, self.round_num, "inventory", "tag_response",
                                tag.uid_hex, f"slot={slot_num}", "success")
                tag.recognized = True
                recognized.append(tag)
                stats["successes"] += 1
                self.tu += self._bits_to_tu(response_duration)
            else:
                tag_ids = ",".join(t.uid_hex for t in tags_in_slot)
                self.logger.log(self.tu, self.round_num, "inventory", "collision",
                                tag_ids, f"slot={slot_num},count={len(tags_in_slot)}", "collision")
                stats["collisions"] += 1
                new_mask_value = (mask_value | (slot_num << mask_length))
                collision_slots.append((slot_num, new_mask_value))
                self.tu += self._bits_to_tu(self.eof_bits)  # collision also takes some time

            # EOF to advance to next slot
            self.logger.log(self.tu, self.round_num, "inventory", "eof",
                            "-", f"slot={slot_num}", "-")
            self.tu += self._bits_to_tu(self.eof_bits)

        # Recursively resolve collisions
        for _slot_num, new_mask_val in collision_slots:
            sub_recognized = self._inventory(tags, mask_length + 4, new_mask_val, stats)
            recognized.extend(sub_recognized)

        return recognized

    def _data_collection(self, recognized_tags: list[SlottedAlohaTag], stats: dict):
        """
        データ収集処理
        
        認識済みの各タグに対して、アドレス指定でデータリクエストを送信し、
        タグからのデータレスポンスを受信する。
        
        Args:
            recognized_tags: データを収集するタグ一覧
            stats: 統計情報を格納する辞書
        """
        for tag in recognized_tags:
            # Reader sends addressed data request
            self.logger.log(self.tu, self.round_num, "data_collection", "data_request",
                            tag.uid_hex, f"cmd_bits={self.reader_cmd_bits}", "-")
            self.tu += self._bits_to_tu(self.reader_cmd_bits)

            # Tag responds with data
            response_bits = tag.data_bits + self.tag_overhead_bits
            self.logger.log(self.tu, self.round_num, "data_collection", "data_response",
                            tag.uid_hex, f"bits={response_bits},data={tag.data}", "success")
            self.tu += self._bits_to_tu(response_bits)
