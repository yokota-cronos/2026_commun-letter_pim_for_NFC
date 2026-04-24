from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
import yaml


def load_config(config_path: Path) -> dict:
	with config_path.open("r", encoding="utf-8") as f:
		return yaml.safe_load(f)


def load_events(events_path: Path) -> pd.DataFrame:
	df = pd.read_csv(events_path, sep="\t")
	df["time_tu"] = pd.to_numeric(df["time_tu"], errors="coerce")
	df["round"] = pd.to_numeric(df["round"], errors="coerce").astype(int)
	df["event"] = df["event"].astype(str)
	df["result"] = df["result"].astype(str)
	df["detail"] = df["detail"].astype(str)
	return df


def assign_block_id(events: pd.DataFrame) -> pd.DataFrame:
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


def parse_detail_fields(detail: str) -> dict:
	out = {}
	for item in str(detail).split(","):
		part = item.strip()
		if "=" not in part:
			continue
		key, value = part.split("=", 1)
		out[key.strip()] = value.strip()
	return out


def success_event_mask(protocol: str, events: pd.DataFrame) -> pd.Series:
	if protocol == "pim":
		return events["event"] == "rx_success"
	if protocol == "slotted_aloha":
		return (events["event"] == "data_response") & (events["result"] == "success")
	raise ValueError(f"unknown protocol: {protocol}")


def extract_slotted_aloha_first_inventory(block: pd.DataFrame) -> pd.DataFrame:
	inventory_cmds = block[(block["phase"] == "inventory") & (block["event"] == "inventory_cmd")]
	first_cmd = inventory_cmds[inventory_cmds["detail"].str.contains("mask_len=0", na=False)]

	if first_cmd.empty:
		return block.iloc[0:0]

	start_time = float(first_cmd.iloc[0]["time_tu"])
	next_cmds = inventory_cmds[inventory_cmds["time_tu"] > start_time]
	end_time = None if next_cmds.empty else float(next_cmds["time_tu"].min())

	inventory = block[(block["phase"] == "inventory") & (block["time_tu"] >= start_time)]
	if end_time is not None:
		inventory = inventory[inventory["time_tu"] < end_time]
	return inventory


def collision_count_from_detail(detail: str) -> int:
	fields = parse_detail_fields(detail)
	try:
		return int(float(fields.get("count", "1")))
	except ValueError:
		return 1


def debug_block(
	protocol: str,
	block: pd.DataFrame,
	num_nodes: int,
	block_id: int,
	max_rows: int,
	max_rounds: int,
) -> dict:
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

	collision_rows = collision_block[collision_block["event"] == "collision"].copy()
	collisions = int(collision_rows.shape[0])
	collision_rows["parsed_count"] = collision_rows["detail"].map(collision_count_from_detail)
	collision_packets = int(collision_rows["parsed_count"].sum())

	attempts = collision_successes + collision_packets
	collision_rate = (collision_packets / attempts) if attempts > 0 else 0.0

	print("\n[DEBUG] ------------------------------")
	print(f"[DEBUG] protocol={protocol} nodes={num_nodes} block_id={block_id}")
	print(
		"[DEBUG] "
		f"successes(all block)={successes}, "
		f"collision_successes(used in rate)={collision_successes}, "
		f"collision_rows={collisions}, collision_packets={collision_packets}"
	)
	print(
		"[DEBUG] "
		f"attempts={attempts}, collision_rate={collision_rate:.8f}"
	)

	print(f"[DEBUG] collision row sample (max {max_rows})")
	if collision_rows.empty:
		print("[DEBUG] (no collision rows)")
	else:
		for idx, (_, row) in enumerate(collision_rows.head(max_rows).iterrows(), start=1):
			print(
				"[DEBUG] "
				f"row{idx}: round={int(row['round'])}, time_tu={row['time_tu']:.6f}, "
				f"tag_id={row['tag_id']}, detail='{row['detail']}', "
				f"parsed_count={int(row['parsed_count'])}"
			)

	print(f"[DEBUG] per-round collision packet sample (max {max_rounds})")
	if collision_rows.empty:
		print("[DEBUG] (no round summary)")
	else:
		per_round = (
			collision_rows.groupby("round")
			.agg(collision_rows=("event", "size"), collision_packets=("parsed_count", "sum"))
			.reset_index()
			.sort_values("round")
		)
		for _, row in per_round.head(max_rounds).iterrows():
			print(
				"[DEBUG] "
				f"round={int(row['round'])} collision_rows={int(row['collision_rows'])} "
				f"collision_packets={int(row['collision_packets'])}"
			)

	if protocol == "pim":
		print(
			"[DEBUG] PIM check: "
			f"collision_packets({collision_packets}) == collision_rows({collisions}) is expected"
		)
	else:
		print(
			"[DEBUG] Slotted check: "
			f"collision_packets({collision_packets}) >= collision_rows({collisions}) is expected"
		)

	return {
		"protocol": protocol,
		"nodes": num_nodes,
		"block_id": block_id,
		"successes": successes,
		"collision_successes": collision_successes,
		"collision_rows": collisions,
		"collision_packets": collision_packets,
		"attempts": attempts,
		"collision_rate": collision_rate,
	}


def run(protocol: str, log_dir: Path, max_rows: int, max_rounds: int) -> None:
	config = load_config(log_dir / "config.yaml")
	events = load_events(log_dir / "events.tsv")
	events = assign_block_id(events)

	nodes_list = config.get("nodes", [])
	print(f"[INFO] protocol={protocol} log_dir={log_dir}")
	print(f"[INFO] nodes in config: {nodes_list}")
	print(f"[INFO] total events={len(events)}")

	rows = []
	for block_id, num_nodes in enumerate(nodes_list):
		block = events[events["block_id"] == block_id]
		if block.empty:
			print(f"[WARN] block_id={block_id} (nodes={num_nodes}) has no rows")
			continue
		rows.append(debug_block(protocol, block, int(num_nodes), block_id, max_rows, max_rounds))

	if not rows:
		print("[WARN] no blocks were processed")
		return

	summary_df = pd.DataFrame(rows)
	print("\n=== COLLISION CHECK SUMMARY ===")
	print(
		summary_df[
			[
				"protocol",
				"nodes",
				"collision_rows",
				"collision_packets",
				"collision_successes",
				"attempts",
				"collision_rate",
			]
		].to_string(index=False)
	)


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Collision verification by debug prints")
	parser.add_argument(
		"--protocol",
		required=True,
		choices=["pim", "slotted_aloha"],
		help="Target protocol",
	)
	parser.add_argument(
		"--log-dir",
		required=True,
		type=Path,
		help="Path to log directory containing config.yaml and events.tsv",
	)
	parser.add_argument(
		"--max-rows",
		type=int,
		default=8,
		help="Max collision rows to print per block",
	)
	parser.add_argument(
		"--max-rounds",
		type=int,
		default=8,
		help="Max round summaries to print per block",
	)
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	run(
		protocol=args.protocol,
		log_dir=args.log_dir,
		max_rows=args.max_rows,
		max_rounds=args.max_rounds,
	)


if __name__ == "__main__":
	main()
