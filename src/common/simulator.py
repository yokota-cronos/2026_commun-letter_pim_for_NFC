from numpy.random import default_rng
import yaml

from .channel import Channel
from .logger import EventLogger


def load_config(config_path: str) -> dict:
    """
    YAML形式の設定ファイルを読み込む
    
    Args:
        config_path: 読み込む設定ファイルのパス
        
    Returns:
        解析された設定情報を含む辞書
    """
    with open(config_path) as f:
        return yaml.safe_load(f)


def bits_to_tu(bits: float, datarate: float, tu: float) -> float:
    """ビット数をTU単位の時間に変換する。

    1ビットの送信時間 = 1/datarate [秒]
    1 TU = tu [秒]
    N bits = N / datarate / tu [TU]
    """
    return bits / (datarate * tu)


class Simulator:
    def __init__(self, protocol: str, config: dict, config_path: str,
                 reader_factory, tag_factory, log_base_dir: str = "logs"):
        """
        シミュレーターの初期化
        
        Args:
            protocol: プロトコル名（"slotted_aloha"または"pim"）
            config: シミュレーション設定情報
              - max_tags: 各ラウンドで生成するタグ総数（タグプールサイズ）
              - nodes: タグプールから通信参加として選択するタグ数のリスト
            config_path: 設定ファイルのパス
            reader_factory: リーダーインスタンスを生成するファクトリ関数
            tag_factory: タグインスタンスを生成するファクトリ関数
            log_base_dir: ログ出力先ディレクトリのベースパス（デフォルト: "logs"）
        """
        self.protocol = protocol
        self.config = config
        self.config_path = config_path
        self.channel = Channel()
        self.logger = EventLogger(protocol, log_base_dir)
        self.logger.copy_config(config_path)
        self.reader_factory = reader_factory
        self.tag_factory = tag_factory
        self.max_tags = config["max_tags"]
        self.tu = 0.0
        self.rng = default_rng(config["random_seed"])

    def run_all(self):
        """
        設定ファイルで指定された全ての選択タグ数に対してシミュレーションを実行
        
        各設定で、max_tags 個のタグを生成した後に nodes 個をランダム選択して
        通信させた結果を収集し、サマリーファイルに出力する。
        """
        nodes_list = self.config["nodes"]
        num_rounds = self.config["num_rounds"]
        summary_lines = []

        for num_nodes in nodes_list:
            self.logger.set_logfile('events_%d.tsv' % (num_nodes))
            print(f"[{self.protocol}] max_tags={self.max_tags}, selected_nodes={num_nodes}, rounds={num_rounds}")
            stats = self.run(num_nodes, num_rounds)
            summary_lines.append(f"nodes={num_nodes}: {stats}")

        self.logger.write_summary(summary_lines)
        print(f"[{self.protocol}] done. logs: {self.logger.log_dir}")

    def run(self, num_nodes: int, num_rounds: int) -> dict:
        """
        指定された選択タグ数とラウンド数でシミュレーションを実行
        
        各ラウンドで以下を実施する:
        1. max_tags 個のタグインスタンスを生成する
        2. その中から num_nodes 個をランダムに抽出する
        3. 抽出されたタグのみでプロトコル処理を実行し、統計を集計する
        
        Args:
            num_nodes: タグプールから通信参加として選択するタグ数
            num_rounds: 実行するラウンド数
            
        Returns:
            衝突数、成功数、実行ラウンド数、ノード数を含む辞書
        """
        if num_nodes > self.max_tags:
            raise ValueError(f"nodes ({num_nodes}) must be <= max_tags ({self.max_tags})")

        total_collisions = 0
        total_successes = 0

        for round_num in range(1, num_rounds + 1):
            self.channel.clear()
            self.tu = 0.0
            reader = self.reader_factory(self.config, self.channel, self.logger, round_num)
            tag_pool = [self.tag_factory(i, self.config, self.rng) for i in range(self.max_tags)]
            selected_tags = self.rng.choice(tag_pool, num_nodes, replace=False)
            round_stats = reader.run_round(selected_tags)
            total_collisions += round_stats.get("collisions", 0)
            total_successes += round_stats.get("successes", 0)

            if round_num % 10 == 0:
                print(f"  round {round_num}/{num_rounds}")

        return {"collisions": total_collisions, "successes": total_successes,
            "rounds": num_rounds, "nodes": num_nodes, "max_tags": self.max_tags}
