import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from src.common.simulator import Simulator, load_config
from src.slotted_aloha.reader import SlottedAlohaReader
from src.slotted_aloha.tag import SlottedAlohaTag


def reader_factory(config, channel, logger, round_num):
    return SlottedAlohaReader(config, channel, logger, round_num)


def tag_factory(index, config):
    return SlottedAlohaTag(index, config)


def main():
    config_path = os.path.join(os.path.dirname(__file__), "config.yaml")
    config = load_config(config_path)
    sim = Simulator("slotted_aloha", config, config_path, reader_factory, tag_factory)
    sim.run_all()


if __name__ == "__main__":
    main()
