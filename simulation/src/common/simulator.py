import yaml

from .channel import Channel
from .logger import EventLogger


def load_config(config_path: str) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


class Simulator:
    def __init__(self, protocol: str, config: dict, config_path: str,
                 reader_factory, tag_factory, log_base_dir: str = "logs"):
        self.protocol = protocol
        self.config = config
        self.config_path = config_path
        self.channel = Channel()
        self.logger = EventLogger(protocol, log_base_dir)
        self.logger.copy_config(config_path)
        self.reader_factory = reader_factory
        self.tag_factory = tag_factory
        self.tu = 0

    def run_all(self):
        nodes_list = self.config["nodes"]
        num_rounds = self.config["num_rounds"]
        summary_lines = []

        for num_nodes in nodes_list:
            print(f"[{self.protocol}] nodes={num_nodes}, rounds={num_rounds}")
            stats = self.run(num_nodes, num_rounds)
            summary_lines.append(f"nodes={num_nodes}: {stats}")

        self.logger.write_summary(summary_lines)
        print(f"[{self.protocol}] done. logs: {self.logger.log_dir}")

    def run(self, num_nodes: int, num_rounds: int) -> dict:
        total_collisions = 0
        total_successes = 0

        for round_num in range(1, num_rounds + 1):
            self.channel.clear()
            self.tu = 0
            reader = self.reader_factory(self.config, self.channel, self.logger, round_num)
            tags = [self.tag_factory(i, self.config) for i in range(num_nodes)]
            round_stats = reader.run_round(tags)
            total_collisions += round_stats.get("collisions", 0)
            total_successes += round_stats.get("successes", 0)

            if round_num % 10 == 0:
                print(f"  round {round_num}/{num_rounds}")

        return {"collisions": total_collisions, "successes": total_successes,
                "rounds": num_rounds, "nodes": num_nodes}
