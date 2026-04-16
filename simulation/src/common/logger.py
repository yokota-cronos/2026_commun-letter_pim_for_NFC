import os
import shutil
from datetime import datetime


class EventLogger:
    HEADER = "time_tu\tround\tphase\tevent\ttag_id\tdetail\tresult"

    def __init__(self, protocol: str, log_base_dir: str = "logs"):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_dir = os.path.join(log_base_dir, f"{protocol}_{timestamp}")
        os.makedirs(self._log_dir, exist_ok=True)
        self._events_path = os.path.join(self._log_dir, "events.tsv")
        with open(self._events_path, "w") as f:
            f.write(self.HEADER + "\n")

    @property
    def log_dir(self) -> str:
        return self._log_dir

    def log(self, time_tu: int, round_num: int, phase: str, event: str,
            tag_id: str, detail: str, result: str):
        line = f"{time_tu}\t{round_num}\t{phase}\t{event}\t{tag_id}\t{detail}\t{result}"
        with open(self._events_path, "a") as f:
            f.write(line + "\n")

    def copy_config(self, config_path: str):
        dest = os.path.join(self._log_dir, os.path.basename(config_path))
        shutil.copy2(config_path, dest)

    def write_summary(self, lines: list[str]):
        path = os.path.join(self._log_dir, "summary.txt")
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
