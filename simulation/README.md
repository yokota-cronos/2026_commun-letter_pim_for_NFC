# NFC MAC Protocol Simulator

NFC環境における複数タグの通信をシミュレーションし、衝突率やスループットの評価に必要なログを出力するツール。2種類のMACプロトコル（Slotted ALOHA / PIM）を比較できる。

## ディレクトリ構成

```
src/
├── common/              # 共通モジュール
│   ├── simulator.py     # シミュレーター本体（時間管理・ラウンド実行）
│   ├── channel.py       # 通信チャネル（パケット管理・衝突検出）
│   └── logger.py        # イベントログ出力（TSV形式）
├── slotted_aloha/       # Slotted ALOHAプロトコル
│   ├── config.yaml      # 設定ファイル
│   ├── reader.py        # リーダー（インベントリ・データ収集）
│   ├── tag.py           # タグ（マスク判定・スロット選択）
│   └── run.py           # 実行エントリポイント
└── pim/                 # PIM (Packet Interval Modulation) プロトコル
    ├── config.yaml      # 設定ファイル
    ├── reader.py        # リーダー（衝突検出・データ復号）
    ├── tag.py           # タグ（バックオフ・プリ/ポストアンブル生成）
    └── run.py           # 実行エントリポイント
```

## 必要環境

- Python 3.9+
- PyYAML

## 実行方法

```bash
# Slotted ALOHA
python -m src.slotted_aloha.run

# PIM
python -m src.pim.run
```

## 設定パラメータ

各プロトコルの `config.yaml` で設定する。

### 共通パラメータ

| パラメータ | 説明 | デフォルト |
|--|--|--|
| `datarate` | データレート [Hz] | 424000 |
| `nodes` | シミュレーション対象のノード数リスト | [5, 10] |
| `data_bits` | データビット数 | 8 |
| `id_bits` | タグID長 [bit] | 8 |
| `num_rounds` | ラウンド数 | 100 |

### Slotted ALOHA固有

| パラメータ | 説明 | デフォルト |
|--|--|--|
| `num_slots` | インベントリスロット数 | 16 |
| `eof_bits` | EOF信号長 [bit] | 8 |
| `reader_cmd_bytes` | リーダーコマンド長 [byte] | 13 |
| `tag_overhead_bytes` | タグ応答オーバーヘッド [byte] | 3 |

### PIM固有

| パラメータ | 説明 | デフォルト |
|--|--|--|
| `backoff_max` | ランダムバックオフ最大値 [TU] | 4096 |
| `delimiter_bits` | デリミタ長 [bit] | 8 |
| `data_tu_mult` | データ待機単位時間のTU倍数 | 1 |

## プロトコル概要

### Slotted ALOHA (NFC Type V準拠)

1. **インベントリフェーズ**: リーダーがインベントリコマンド（マスク長・マスク値・スロット数=16を指定）を送信。タグはUIDの下位ビットがマスクに一致すれば、マスク直上4ビットで決まるスロットで応答する。衝突発生時はマスク長を4bit拡張して再帰的に解決する。
2. **データ収集フェーズ**: 認識済みタグに対してアドレス指定でデータ要求を順次送信。

### PIM (Packet Interval Modulation)

1. リーダーが開始信号を送信。
2. 各タグはランダムバックオフ後にプリアンブル（デリミタ+ID）を送信。
3. 送信データ値に比例した時間だけ待機し、ポストアンブル（デリミタ+ID）を送信。
4. プリアンブルとポストアンブルの間隔がデータを表現する。

## ログ出力

実行ごとに `logs/<protocol>_<YYYYMMDD_HHMMSS>/` ディレクトリが生成される。

| ファイル | 内容 |
|--|--|
| `events.tsv` | 全イベントログ（TSV形式） |
| `config.yaml` | 実行時設定のコピー |
| `summary.txt` | ノード数ごとの衝突数・成功数の集計 |

### events.tsv のカラム

| カラム | 説明 |
|--|--|
| `time_tu` | イベント発生時刻 [TU] |
| `round` | ラウンド番号 |
| `phase` | フェーズ（`inventory` / `data_collection` / `transmission`） |
| `event` | イベント種別 |
| `tag_id` | 対象タグのUID（16進数） |
| `detail` | 詳細情報 |
| `result` | 結果（`success` / `collision` / `empty` / `-`） |

### 主なイベント種別

**Slotted ALOHA:**
`inventory_cmd`, `tag_response`, `collision`, `slot_empty`, `eof`, `unresolvable`, `data_request`, `data_response`

**PIM:**
`start_signal`, `tx_preamble`, `tx_postamble`, `rx_success`, `collision`
