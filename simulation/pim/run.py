import itertools
import os
import sys
from numpy.random import default_rng

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from simulation.common.logger import timestamp_now
from simulation.common.simulator import Simulator, load_config
from simulation.pim.reader import PimReader
from simulation.pim.tag import PimTag

# リストで渡すと掃引できる config キーと、サブディレクトリ名の付け方
SWEEP_KEYS = {
    "data_dist": lambda v: str(v),               # uniform / gauss
    "data_tu_mult": lambda v: f"mult{int(v)}",   # mult1 / mult2 / mult3
    "backoff_max": lambda v: f"backoff_{int(v)}",
}


def reader_factory(config, channel, logger, round_num):
    return PimReader(config, channel, logger, round_num)


def tag_factory(index, config, rng=None):
    return PimTag(index, config, rng)


def main():
    # 引数で config を指定できる（例 python -m simulation.pim.run simulation/pim/config_exp_dist_mult.yaml）
    config_path = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(__file__), "config.yaml")
    config = load_config(config_path)
    swept = [k for k in SWEEP_KEYS if isinstance(config.get(k), list)]
    if not swept:
        sim = Simulator("pim", config, config_path, reader_factory, tag_factory)
        sim.run_all()
        return

    # data_dist / data_tu_mult / backoff_max のどれかがリストなら、全組み合わせを回す。出力は
    #   logs/pim_YYYYMMDD_HHMMSS/config.yaml        （元の config のコピー）
    #   logs/pim_YYYYMMDD_HHMMSS/<組み合わせ名>/     （値を 1 つに絞った config.yaml と events_N.tsv）
    # 組み合わせ名は値を "_" でつないだもの（例 gauss_mult2、backoff_max だけなら従来どおり
    # backoff_16384）。サブディレクトリは単一値のときのログと同じ形式なので、
    # そのまま同じ方法で集計できる。乱数は組み合わせごとに seed から作り直す。
    root = os.path.join("logs", f"pim_{timestamp_now()}")
    os.makedirs(root, exist_ok=True)
    with open(config_path) as src, open(os.path.join(root, "config.yaml"), "w") as dst:
        dst.write(src.read())
    for values in itertools.product(*(config[k] for k in swept)):
        cfg = dict(config, **dict(zip(swept, values)))
        for k in ("data_tu_mult", "backoff_max"):
            if k in swept:
                cfg[k] = int(cfg[k])
        name = "_".join(SWEEP_KEYS[k](v) for k, v in zip(swept, values))
        print(f"[pim] {name}")
        sim = Simulator("pim", cfg, config_path, reader_factory, tag_factory,
                        log_dir=os.path.join(root, name))
        sim.run_all()


if __name__ == "__main__":
    main()
