import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from src.common.simulator import Simulator, load_config
from src.pim.reader import PimReader
from src.pim.tag import PimTag


def reader_factory(config, channel, logger, round_num):
    return PimReader(config, channel, logger, round_num)


def tag_factory(index, config):
    return PimTag(index, config)


def main():
    config_path = os.path.join(os.path.dirname(__file__), "config.yaml")
    config = load_config(config_path)
    sim = Simulator("pim", config, config_path, reader_factory, tag_factory)
    sim.run_all()


if __name__ == "__main__":
    main()
