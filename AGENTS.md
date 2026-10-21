# AGENTS.md

M5Stack 用の Unitree サーボ基板 (`kim-xps12/m5stack_board_unitree_brushless_digital_servo`) を動かして確かめるための、AtomS3R のファームウェア、計測と PC 用のスクリプト、実験の記録です。

## 参照先

| ファイル | 読むとき |
| --- | --- |
| `atoms3r/docs/ttl_board_redesign.md` | 送信の自動切り替えの定数やタイミング、試作基板の試験結果 |
| `atoms3r/docs/s288_link_explained.md` | 通信の問題と対策の説明 |
| `atoms3r/README.md` | AtomS3R の配線、計測のコマンド |
| `python/README.md` | PC 用のスクリプト |
| `external/digital_servo/specs/protocol.md` | パケットの仕様 (公式) |

## コマンド

```bash
git submodule update --init           # 公式リポジトリの取得 (初回)
cd atoms3r && pio run -e bench -t upload   # 計測用ファームウェア (首振りデモは -e sin_swing)
python3 atoms3r/tools/bench.py check  # 配線の確認。計測の手順は atoms3r/README.md
```

## 制約

- `external/digital_servo` (Unitree の公式) は変更しない。直したいものは自分のファイルに置く (例：`python/servo_protocol.py`)。
- 公式のコードをこのリポジトリにコピーしない。Unitree のライセンスは非商用向けで、再配布の条件がはっきりしない。
- 実験の記録とログは、当時の通信速度のまま残す。書き換えない。
- AtomS3R の端子に 5 V の信号を直接入れない (ESP32-S3 は 5 V に耐えない)。

## やりとり

- 回答と報告は日本語で書く。用語は説明してから使い、平易に書く。
- README は日本語の段落と英語の段落の 2 言語で書く。日本語の句読点は「，」「．」。

## Git

- remote は GitHub の `kim-xps12/m5stack_board_unitree_servo_bench`。push はユーザに頼まれたときだけ行う。
- 作業はブランチで行い、PR で `main` に入れる。PR のマージはユーザがブラウザで行う。
- コミットはユーザに頼まれたときだけ作る。1 コミット = 1 つの論理的な変更。
- コミットメッセージは英語で書く。件名は命令形、本文に理由を書く。
