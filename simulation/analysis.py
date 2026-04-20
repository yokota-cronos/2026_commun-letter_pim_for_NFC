from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import japanize_matplotlib
import pandas as pd
import yaml


# ログパスはここで固定指定する
PIM_LOG_DIR = Path("logs/pim_20260419_225828")
SLOTTED_ALOHA_LOG_DIR = Path("logs/slotted_aloha_20260419_225913")
# PIM_LOG_DIR = Path("logs/pim_20260420_011748")
# SLOTTED_ALOHA_LOG_DIR = Path("logs/slotted_aloha_20260420_011724")

# PDF出力先
OUTPUT_DIR = Path("img/analysis")
COLLISION_RATE_PDF = OUTPUT_DIR / "collision_rate_comparison2.pdf"
THROUGHPUT_BPS_PDF = OUTPUT_DIR / "throughput_bps_comparison2.pdf"


def load_config(config_path: Path) -> dict:
    """設定ファイルを読み込み、解析に必要なパラメータ辞書を返す。"""
    with config_path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_events(events_path: Path) -> pd.DataFrame:
    """events.tsvを読み込み、時刻とラウンドを数値列へ整形して返す。"""
    df = pd.read_csv(events_path, sep="\t")
    df["time_tu"] = pd.to_numeric(df["time_tu"], errors="coerce")
    df["round"] = pd.to_numeric(df["round"], errors="coerce").astype(int)
    df["event"] = df["event"].astype(str)
    df["result"] = df["result"].astype(str)
    return df


def assign_block_id(events: pd.DataFrame) -> pd.DataFrame:
    """
    連続したイベント列を、nodes条件ごとの実行ブロックに分割する。

    round番号はnodes条件ごとに1から振り直されるため、
    roundが小さくなるタイミングを新しいブロック開始として扱う。
    """
    block_ids = []
    block_id = 0
    prev_round = None

    for round_num in events["round"]:
        if prev_round is not None and round_num < prev_round:
            block_id += 1
        block_ids.append(block_id)
        prev_round = round_num

    out = events.copy()
    out["block_id"] = block_ids
    return out


def success_event_mask(protocol: str, events: pd.DataFrame) -> pd.Series:
    """プロトコルごとの成功イベント条件を返す。"""
    if protocol == "pim":
        return events["event"] == "rx_success"
    if protocol == "slotted_aloha":
        return (events["event"] == "data_response") & (events["result"] == "success")
    raise ValueError(f"未知のプロトコルです: {protocol}")


def parse_detail_fields(detail: str) -> dict:
    """detail文字列を key=value の辞書へ変換する。"""
    out = {}
    for item in str(detail).split(","):
        part = item.strip()
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        out[key.strip()] = value.strip()
    return out


def extract_slotted_aloha_first_inventory(block: pd.DataFrame) -> pd.DataFrame:
    """
    Slotted Aloha ブロックから、mask_len=0 の最初の inventory だけを切り出す。

    返す範囲は最初の inventory_cmd から、次の inventory_cmd の直前まで。
    inventory_cmd が1回しかない場合は、その inventory 全体を返す。
    """
    inventory_cmds = block[(block["phase"] == "inventory") & (block["event"] == "inventory_cmd")]
    first_cmd = inventory_cmds[inventory_cmds["detail"].astype(str).str.contains("mask_len=0", na=False)]

    if first_cmd.empty:
        return block.iloc[0:0]

    start_time = float(first_cmd.iloc[0]["time_tu"])
    next_cmds = inventory_cmds[inventory_cmds["time_tu"] > start_time]
    if next_cmds.empty:
        end_time = None
    else:
        end_time = float(next_cmds["time_tu"].min())

    inventory = block[(block["phase"] == "inventory") & (block["time_tu"] >= start_time)]
    if end_time is not None:
        inventory = inventory[inventory["time_tu"] < end_time]

    return inventory


def summarize_protocol(protocol: str, log_dir: Path) -> pd.DataFrame:
    """
    1プロトコル分のログから、nodes別の衝突率とスループット(bps)を算出する。

    衝突率: collisions*衝突パケット数 / (collisions*衝突パケット数 + successes)
    スループット(bps): (successes * data_bits) / 総実行時間[sec]
    """
    config = load_config(log_dir / "config.yaml")
    events = load_events(log_dir / "events.tsv")
    events = assign_block_id(events)

    nodes_list = config["nodes"]
    data_bits = float(config["data_bits"])
    tu_sec = float(config["tu"])

    rows = []
    for block_id, num_nodes in enumerate(nodes_list):
        block = events[events["block_id"] == block_id]
        if block.empty:
            continue

        success_mask = success_event_mask(protocol, block)
        successes = int(success_mask.sum())
        if protocol == "slotted_aloha":
            collision_block = extract_slotted_aloha_first_inventory(block)
            collision_successes = int(
                ((collision_block["event"] == "tag_response") & (collision_block["result"] == "success")).sum()
            )
        else:
            collision_block = block
            collision_successes = successes

        collisions = int((collision_block["event"] == "collision").sum())
        # 複数ノードが同時に衝突した場合は、パケットが完全に失われるため、衝突数倍して計算する
        collision_rows = collision_block[collision_block["event"] == "collision"]
        collision_packets = 0
        for detail in collision_rows["detail"]:
            fields = parse_detail_fields(detail)
            try:
                collision_packets += int(float(fields.get("count", "1")))
            except ValueError:
                collision_packets += 1

        attempts = collision_successes + collision_packets
        collision_rate = (collision_packets / attempts) if attempts > 0 else 0.0

        # nodes条件ごとのブロック内で、全roundの最終時刻を合計して総実行時間(TU)を得る
        total_tu = float(block.groupby("round")["time_tu"].max().sum())
        total_sec = total_tu * tu_sec
        throughput_bps = (successes * data_bits / total_sec) if total_sec > 0 else 0.0

        row = {
            "protocol": protocol,
            "nodes": int(num_nodes),
            "collisions": collisions,
            "successes": successes,
            "collision_rate": collision_rate,
            "throughput_bps": throughput_bps,
        }

        rows.append(row)

    return pd.DataFrame(rows).sort_values(["nodes", "protocol"]).reset_index(drop=True)


def plot_collision_rate(df: pd.DataFrame, output_pdf: Path) -> None:
    """nodesに対する衝突率をPIM/Slotted Alohaで比較し、PDFに出力する。"""
    pivot = df.pivot(index="nodes", columns="protocol", values="collision_rate").sort_index()

    fig, ax = plt.subplots(figsize=(8, 5))
    for protocol in pivot.columns:
        ax.plot(pivot.index, pivot[protocol], marker="o", label=protocol)

    ax.set_title("Collision Rate vs Selected Tags")
    ax.set_xlabel("Selected Tags (nodes)")
    ax.set_ylabel("Collision Rate")
    ax.set_ylim(0.0, 1.0)
    ax.grid(alpha=0.3)
    ax.legend()

    output_pdf.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(output_pdf) as pdf:
        pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def plot_throughput_bps(df: pd.DataFrame, output_pdf: Path) -> None:
    """nodesに対するスループット(bps)をPIM/Slotted Alohaで比較し、PDFに出力する。"""
    pivot = df.pivot(index="nodes", columns="protocol", values="throughput_bps").sort_index()

    fig, ax = plt.subplots(figsize=(8, 5))
    for protocol in pivot.columns:
        ax.plot(pivot.index, pivot[protocol], marker="o", label=protocol)

    ax.set_title("Throughput (bps) vs Selected Tags")
    ax.set_xlabel("Selected Tags (nodes)")
    ax.set_ylabel("Throughput (bps)")
    ax.grid(alpha=0.3)
    ax.legend()

    output_pdf.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(output_pdf) as pdf:
        pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    """PIMとSlotted Alohaを比較解析し、各指標のPDFを出力する。"""
    pim_df = summarize_protocol("pim", PIM_LOG_DIR)
    slotted_df = summarize_protocol("slotted_aloha", SLOTTED_ALOHA_LOG_DIR)
    merged = pd.concat([pim_df, slotted_df], ignore_index=True)

    plot_collision_rate(merged, COLLISION_RATE_PDF)
    plot_throughput_bps(merged, THROUGHPUT_BPS_PDF)

    print("比較解析が完了しました。")
    print(f"PIM log dir: {PIM_LOG_DIR}")
    print(f"Slotted Aloha log dir: {SLOTTED_ALOHA_LOG_DIR}")
    print(f"Collision rate PDF: {COLLISION_RATE_PDF}")
    print(f"Throughput PDF: {THROUGHPUT_BPS_PDF}")
    print()
    print("=== 解析結果 ===")
    print(
        merged[["protocol", "nodes", "collision_rate", "throughput_bps"]].to_string(index=False)
    )


if __name__ == "__main__":
    main()