from __future__ import annotations
import os
import shutil
from datetime import datetime

import yaml


def timestamp_now() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


class EventLogger:
    HEADER = "time_tu\tround\tphase\tevent\ttag_id\tdetail\tresult"

    def __init__(self, protocol: str, log_base_dir: str = "logs", log_dir: str | None = None):
        # log_dir を渡すとそのディレクトリへ直接書く（パラメータ掃引のサブディレクトリ用）
        if log_dir is None:
            log_dir = os.path.join(log_base_dir, f"{protocol}_{timestamp_now()}")
        self._log_dir = log_dir
        os.makedirs(self._log_dir, exist_ok=True)
        self._events_path = None

    @property
    def log_dir(self) -> str:
        return self._log_dir

    def set_logfile(self, fname):
        self._events_path = os.path.join(self._log_dir, fname)
        with open(self._events_path, "w") as f:
            f.write(self.HEADER + "\n")

    def log(self, time_tu: float, round_num: int, phase: str, event: str,
            tag_id: str, detail: str, result: str):
        line = f"{time_tu:.6f}\t{round_num}\t{phase}\t{event}\t{tag_id}\t{detail}\t{result}"
        with open(self._events_path, "a") as f:
            f.write(line + "\n")

    def copy_config(self, config_path: str):
        dest = os.path.join(self._log_dir, os.path.basename(config_path))
        shutil.copy2(config_path, dest)

    def write_config(self, config: dict, fname: str = "config.yaml"):
        """実際に使った設定（掃引で値を 1 つに絞ったもの）を書き出す。"""
        with open(os.path.join(self._log_dir, fname), "w") as f:
            yaml.safe_dump(config, f, default_flow_style=None, sort_keys=False, allow_unicode=True)

    def write_summary(self, lines: list[str]):
        path = os.path.join(self._log_dir, "summary.txt")
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
