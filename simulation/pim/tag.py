import numpy as np
from numpy.random import default_rng


DATA_MIN, DATA_MAX = 1, 255


def draw_data(rng: np.random.Generator, config: dict) -> int:
    """config の data_dist に従ってデータ値を 1 つ引く（1-255 の整数）。"""
    dist = config.get("data_dist", "uniform")
    if dist == "uniform":
        return int(rng.integers(DATA_MIN, DATA_MAX + 1))
    if dist == "gauss":
        v = rng.normal(loc=config.get("data_mean", 128), scale=config.get("data_std", 30))
        return int(np.clip(np.rint(v), DATA_MIN, DATA_MAX))
    raise ValueError(f"data_dist は uniform / gauss のどちらか（{dist!r}）")


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
        # パルス長 [TU] を直接指定する場合（実機に合わせる場合）。
        # 実機（2026_test_NFC_PIM_w_AD3_ido の host/pim.py）は delimiter バイトを持たず、
        # SOF（サブキャリア連続 ON）の «長さ» で preamble / postamble を区別するので、
        # 2 つのパルス長が違う: preamble = SOF 4 + gap 1 + ID 16 = 21 TU、
        #                       postamble = SOF 8 + gap 1 + ID 16 = 25 TU。
        # 指定が無ければ従来どおり (delimiter_bits + id_bits) から両方同じ長さを計算する
        # （従来の config の結果は変わらない）。
        self._preamble_tu = config.get("preamble_tu")
        self._postamble_tu = config.get("postamble_tu")
        self.rng = rng

        self.uid = rng.integers(2**self.id_bits)

        # データ値の分布（config の data_dist）。乱数の消費順（uid → data → backoff）は
        # どちらでも同じ。実機 host/pim_multitag.py の SimTagRng も同じ手順で引く。
        #   uniform（既定）: 1-255 の一様分布
        #   gauss          : 正規分布 N(data_mean, data_std) を四捨五入して 1-255 に丸める
        #                    （実機は整数しか送れないので丸める）
        self.data = draw_data(rng, config)
        
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
    def preamble_tu(self) -> float:
        """
        プリアンブル 1 発の長さ（TU単位）

        config の preamble_tu があればその値、無ければ packet_tu

        Returns:
            プリアンブル送信時間（TU単位）
        """
        if self._preamble_tu is None:
            return self.packet_tu
        return float(self._preamble_tu)

    @property
    def postamble_tu(self) -> float:
        """
        ポストアンブル 1 発の長さ（TU単位）

        config の postamble_tu があればその値、無ければ packet_tu

        Returns:
            ポストアンブル送信時間（TU単位）
        """
        if self._postamble_tu is None:
            return self.packet_tu
        return float(self._postamble_tu)

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
        
        開始時刻にプリアンブル送信時間を加えた値
        
        Returns:
            プリアンブル終了時刻（TU単位）
        """
        return self.preamble_start + self.preamble_tu

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
        
        開始時刻にポストアンブル送信時間を加えた値
        
        Returns:
            ポストアンブル終了時刻（TU単位）
        """
        return self.postamble_start + self.postamble_tu
