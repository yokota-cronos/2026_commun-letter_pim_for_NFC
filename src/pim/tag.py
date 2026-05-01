import numpy as np
from numpy.random import default_rng


class PimTag:
    DELIMITER_PREAMBLE = 0xAA
    DELIMITER_POSTAMBLE = 0x55

    def __init__(self, index: int, config: dict, rng: np.random.Generator):
        """
        PIMタグの初期化
        
        Args:
            index: タグのインデックス
            config: 設定情報（id_bits、data_bits、delimiter_bits等を含む）
        """
        self.index = index
        self.id_bits = config["id_bits"]
        self.data_bits = config["data_bits"]
        self.delimiter_bits = config["delimiter_bits"]
        self.backoff_max = config["backoff_max"]
        self.data_tu_mult = config["data_tu_mult"]
        self.bits_per_tu = config["datarate"] * config["tu"]
        self.rng = rng

        self.uid = rng.integers(2**self.id_bits)
        self.data = rng.integers(1, 256)
        self.backoff = rng.integers(self.backoff_max)

    @property
    def uid_hex(self) -> str:
        """
        タグのUIDを16進数の文字列として返す
        
        Returns:
            16進数表記のUID文字列
        """
        return format(self.uid, f"0{(self.id_bits + 3) // 4}x")

    @property
    def packet_bits(self) -> int:
        """
        パケットの総ビット数を計算（デリミタ + ID）
        
        Returns:
            パケットの総ビット数
        """
        return self.delimiter_bits + self.id_bits

    @property
    def packet_tu(self) -> float:
        """
        パケット送信に要するTime Unit（TU）を計算
        
        Returns:
            パケット送信時間（TU単位）
        """
        return self.packet_bits / self.bits_per_tu

    @property
    def data_wait_tu(self) -> float:
        """
        データ部の待機時間（Time Unit）を計算
        
        タグのデータ値に基づいて、リーダの応答を待つ時間を決定する
        
        Returns:
            データ待機時間（TU単位）
        """
        return self.data * self.data_tu_mult

    @property
    def preamble_start(self) -> float:
        """
        プリアンブル（前置部分）の開始時刻を取得
        
        バックオフ時間に基づいて決定される
        
        Returns:
            プリアンブル開始時刻（TU単位）
        """
        return float(self.backoff)

    @property
    def preamble_end(self) -> float:
        """
        プリアンブルの終了時刻を計算
        
        開始時刻にパケット送信時間を加えた値
        
        Returns:
            プリアンブル終了時刻（TU単位）
        """
        return self.preamble_start + self.packet_tu

    @property
    def postamble_start(self) -> float:
        """
        ポストアンブル（後置部分）の開始時刻を計算
        
        プリアンブル終了後、データ待機時間を加えた時刻
        
        Returns:
            ポストアンブル開始時刻（TU単位）
        """
        return self.preamble_end + self.data_wait_tu

    @property
    def postamble_end(self) -> float:
        """
        ポストアンブルの終了時刻を計算
        
        開始時刻にパケット送信時間を加えた値
        
        Returns:
            ポストアンブル終了時刻（TU単位）
        """
        return self.postamble_start + self.packet_tu
