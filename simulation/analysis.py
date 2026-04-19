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
WAVEFORM_THROUGHPUT_BPS_PDF = OUTPUT_DIR / "waveform_throughput_bps_comparison2.pdf"


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


def extract_waveform_time_pim(block: pd.DataFrame) -> float:
    """
    PIM ブロック内の全タグ送信波形時間(成功+失敗)の合計を計算する。

    各タグについて、利用可能な送信イベントから以下を優先して時間を計算する。
    1) preamble_start から postamble_end
    2) preamble 区間のみ
    3) postamble 区間のみ

    これにより、衝突で途中終了した送信も分母へ含める。
    """
    total_wave_tu = 0.0

    tx_pairs = (
        block[
            (block["event"].isin(["rx_success", "collision", "tx_preamble", "tx_postamble"]))
            & (block["tag_id"] != "-")
        ][["round", "tag_id"]]
        .drop_duplicates()
        .itertuples(index=False)
    )

    for round_num, tag_id in tx_pairs:
        tx_events = block[(block["round"] == round_num) & (block["tag_id"] == tag_id)]

        preamble_start = None
        preamble_end = None
        for detail in tx_events[tx_events["event"] == "tx_preamble"]["detail"]:
            fields = parse_detail_fields(detail)
            if "start" in fields:
                value = float(fields["start"])
                preamble_start = value if preamble_start is None else min(preamble_start, value)
            if "end" in fields:
                value = float(fields["end"])
                preamble_end = value if preamble_end is None else max(preamble_end, value)

        postamble_start = None
        postamble_end = None
        for detail in tx_events[tx_events["event"] == "tx_postamble"]["detail"]:
            fields = parse_detail_fields(detail)
            if "start" in fields:
                value = float(fields["start"])
                postamble_start = value if postamble_start is None else min(postamble_start, value)
            if "end" in fields:
                value = float(fields["end"])
                postamble_end = value if postamble_end is None else max(postamble_end, value)

        wave_tu = 0.0
        if preamble_start is not None and postamble_end is not None:
            wave_tu = postamble_end - preamble_start
        elif preamble_start is not None and preamble_end is not None:
            wave_tu = preamble_end - preamble_start
        elif postamble_start is not None and postamble_end is not None:
            wave_tu = postamble_end - postamble_start

        if wave_tu > 0:
            total_wave_tu += wave_tu

    return total_wave_tu


def extract_waveform_time_slotted_aloha(block: pd.DataFrame) -> float:
    """
    Slotted Aloha ブロック内の全タグ送信波形時間(成功+失敗)の合計を計算する。

    各タグ送信について、以下を合算する。
    - inventory: tag_response から同一slotの eof まで
    - inventory(collision): collision から同一slotの eof まで (count倍)
    - data_collection: data_response から次の data_request まで
    """
    total_wave_tu = 0.0

    for _, round_events in block.groupby("round"):
        round_events = round_events.sort_values(["time_tu"], kind="stable")

        inv = round_events[round_events["phase"] == "inventory"]
        inv_success = inv[(inv["event"] == "tag_response") & (inv["result"] == "success")]
        inv_eof = inv[inv["event"] == "eof"]

        for _, row in inv_success.iterrows():
            resp_time = float(row["time_tu"])
            fields = parse_detail_fields(row["detail"])
            slot = fields.get("slot")
            if slot is None:
                continue

            matched_eof = []
            for _, eof_row in inv_eof.iterrows():
                eof_fields = parse_detail_fields(eof_row["detail"])
                if eof_fields.get("slot") == slot and float(eof_row["time_tu"]) >= resp_time:
                    matched_eof.append(float(eof_row["time_tu"]))

            if not matched_eof:
                continue

            wave_tu = min(matched_eof) - resp_time
            if wave_tu > 0:
                total_wave_tu += wave_tu

        inv_collision = inv[inv["event"] == "collision"]
        for _, row in inv_collision.iterrows():
            col_time = float(row["time_tu"])
            fields = parse_detail_fields(row["detail"])
            slot = fields.get("slot")
            if slot is None:
                continue

            try:
                count = int(float(fields.get("count", "1")))
            except ValueError:
                count = 1

            matched_eof = []
            for _, eof_row in inv_eof.iterrows():
                eof_fields = parse_detail_fields(eof_row["detail"])
                if eof_fields.get("slot") == slot and float(eof_row["time_tu"]) >= col_time:
                    matched_eof.append(float(eof_row["time_tu"]))

            if not matched_eof:
                continue

            wave_tu = min(matched_eof) - col_time
            if wave_tu > 0:
                total_wave_tu += wave_tu * count

        data = round_events[round_events["phase"] == "data_collection"]
        data_response = data[data["event"] == "data_response"]
        data_request_times = list(data[data["event"] == "data_request"]["time_tu"].astype(float))

        for _, row in data_response.iterrows():
            resp_time = float(row["time_tu"])
            next_requests = [t for t in data_request_times if t > resp_time]

            if next_requests:
                wave_tu = min(next_requests) - resp_time
            else:
                fields = parse_detail_fields(row["detail"])
                wave_tu = float(fields.get("bits", 0.0))

            if wave_tu > 0:
                total_wave_tu += wave_tu

    return total_wave_tu


def summarize_protocol(protocol: str, log_dir: Path) -> pd.DataFrame:
    """
    1プロトコル分のログから、nodes別の衝突率とスループット(bps)を算出する。

    衝突率: collisions*衝突パケット数 / (collisions*衝突パケット数 + successes)
    スループット(bps): (successes * data_bits) / 総実行時間[sec]
    
    追加指標として、送信波形時間ベースのスループットも算出する。
    ここで送信波形時間は成功・失敗の両方を含む。
    - PIM: preamble_start ～ postamble_end
    - Slotted Aloha: inventory応答時間 + data応答時間
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
        collisions = int((block["event"] == "collision").sum())
        # 複数ノードが同時に衝突した場合は、パケットが完全に失われるため、衝突数倍して計算する
        collision_rows = block[block["event"] == "collision"]
        collision_packets = 0
        for detail in collision_rows["detail"]:
            fields = parse_detail_fields(detail)
            try:
                collision_packets += int(float(fields.get("count", "1")))
            except ValueError:
                collision_packets += 1

        attempts = successes + collision_packets
        collision_rate = (collision_packets / attempts) if attempts > 0 else 0.0

        # ラウンドごとの最終時刻を足して、そのブロックの総実行時間(TU)を得る
        total_tu = float(block.groupby("round")["time_tu"].max().sum())
        total_sec = total_tu * tu_sec
        throughput_bps = (successes * data_bits / total_sec) if total_sec > 0 else 0.0

        if protocol == "pim":
            waveform_tu = extract_waveform_time_pim(block)
        elif protocol == "slotted_aloha":
            waveform_tu = extract_waveform_time_slotted_aloha(block)
        else:
            waveform_tu = 0.0

        waveform_sec = waveform_tu * tu_sec
        waveform_throughput_bps = (successes * data_bits / waveform_sec) if waveform_sec > 0 else 0.0

        row = {
            "protocol": protocol,
            "nodes": int(num_nodes),
            "collisions": collisions,
            "successes": successes,
            "collision_rate": collision_rate,
            "throughput_bps": throughput_bps,
            "waveform_time_sec": waveform_sec,
            "waveform_throughput_bps": waveform_throughput_bps,
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


def plot_waveform_throughput_bps(df: pd.DataFrame, output_pdf: Path) -> None:
    """nodesに対する波形時間ベースのスループット(bps)を比較し、PDFに出力する。"""
    pivot = df.pivot(index="nodes", columns="protocol", values="waveform_throughput_bps").sort_index()

    fig, ax = plt.subplots(figsize=(8, 5))
    for protocol in pivot.columns:
        ax.plot(pivot.index, pivot[protocol], marker="o", label=protocol)

    ax.set_title("Waveform Throughput (bps) vs Selected Tags")
    ax.set_xlabel("Selected Tags (nodes)")
    ax.set_ylabel("Waveform Throughput (bps)")
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
    # plot_throughput_bps(merged, THROUGHPUT_BPS_PDF)
    plot_waveform_throughput_bps(merged, WAVEFORM_THROUGHPUT_BPS_PDF)

    print("比較解析が完了しました。")
    print(f"PIM log dir: {PIM_LOG_DIR}")
    print(f"Slotted Aloha log dir: {SLOTTED_ALOHA_LOG_DIR}")
    print(f"Collision rate PDF: {COLLISION_RATE_PDF}")
    print(f"Throughput PDF: {THROUGHPUT_BPS_PDF}")
    print(f"Waveform throughput PDF: {WAVEFORM_THROUGHPUT_BPS_PDF}")
    print()
    print("=== 解析結果 ===")
    print(
        merged[
            [
                "protocol",
                "nodes",
                "collision_rate",
                "throughput_bps",
                "waveform_time_sec",
                "waveform_throughput_bps",
            ]
        ].to_string(index=False)
    )


if __name__ == "__main__":
    main()