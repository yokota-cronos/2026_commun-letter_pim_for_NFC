import random


class SlottedAlohaTag:
    def __init__(self, index: int, config: dict):
        """
        SlottedAlohaタグの初期化
        
        Args:
            index: タグのインデックス
            config: 設定情報（id_bits、data_bitsを含む）
        """
        self.index = index
        self.id_bits = config["id_bits"]
        self.data_bits = config["data_bits"]
        self.uid = random.getrandbits(self.id_bits)
        self.data = random.getrandbits(self.data_bits)
        self.recognized = False

    @property
    def uid_hex(self) -> str:
        """
        タグのUIDを16進数の文字列として返す
        
        Returns:
            16進数表記のUID文字列
        """
        return format(self.uid, f"0{(self.id_bits + 3) // 4}x")

    def check_mask(self, mask_length: int, mask_value: int) -> bool:
        """
        UID下位ビットがマスク値と一致するかチェック
        
        Args:
            mask_length: チェック対象のビット数
            mask_value: 比較対象のマスク値
            
        Returns:
            UIDの下位ビットがマスク値と一致する場合True
        """
        if mask_length == 0:
            return True
        lower_bits = self.uid & ((1 << mask_length) - 1)
        return lower_bits == mask_value

    def select_slot(self, mask_length: int) -> int:
        """
        UIDからスロット番号を選択
        
        Args:
            mask_length: マスク処理済みのビット数
            
        Returns:
            選択されたスロット番号（0-15）
        """
        slot_bits = (self.uid >> mask_length) & 0xF
        return slot_bits

    def response_bits(self) -> int:
        """
        タグがレスポンスで返すビット数を計算
        
        Returns:
            データビット + 3バイトのオーバーヘッドの合計ビット数
        """
        return self.data_bits + 3 * 8  # data + 3byte overhead
