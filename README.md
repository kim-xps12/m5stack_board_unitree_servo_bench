# Unitree Servo Board Bench for M5Stack

## About
[M5Stack 用の Unitree サーボ基板](https://github.com/kim-xps12/m5stack_board_unitree_brushless_digital_servo) を動かして確かめるための，ファームウェア，スクリプト，実験の記録です．AtomS3R を測定器としても使い，オシロスコープなしで，送信の切り替えのタイミングとサーボとの通信を測ります．基板を作り直す前に DYNAMIXEL 用の TTL 基板で行った実験も含みます．

Firmware, scripts and test records for bringing up the [Unitree servo board for M5Stack](https://github.com/kim-xps12/m5stack_board_unitree_brushless_digital_servo). The AtomS3R doubles as the measuring instrument, so the transmit-enable timing and the servo link are measured without an oscilloscope. The experiments on the DYNAMIXEL TTL board that led to the redesign are included.

## Contents

| フォルダ / Folder | 中身 / Contents |
| --- | --- |
| `atoms3r/` | AtomS3R のファームウェア (`bench`：計測用，`sin_swing`：首振りデモ)，計測スクリプト，実験，設計メモ (`docs/`)，ログ (`logs/`)．AtomS3R firmware, bench scripts, experiments, design notes and logs |
| `python/` | Unitree の USB アダプタを使って PC から動かすスクリプトと，パケット処理 (`servo_protocol.py`)．Host scripts through Unitree's USB adapter, and the packet module |
| `external/digital_servo/` | Unitree の公式リポジトリ (git submodule，変更しない)．Unitree's repository as a git submodule, kept unmodified |

## Setup

```bash
git clone --recursive https://github.com/kim-xps12/m5stack_board_unitree_servo_bench.git
git submodule update --init   # --recursive を付けずに取得したとき / if cloned without --recursive
```

ファームウェアは公式リポジトリのパケット処理 (`stm32/firmware/App/protocol.c`) をそのままビルドに使います．使い方は [atoms3r/README.md](atoms3r/README.md) と [python/README.md](python/README.md) を見てください．

The firmware builds Unitree's packet code (`stm32/firmware/App/protocol.c`) unmodified. See [atoms3r/README.md](atoms3r/README.md) and [python/README.md](python/README.md) for usage.

## Notes

実験の記録とログは，当時の通信速度 (6 Mbps，3.95 Mbps など) のまま残しています．基板は 4 Mbps だけを扱います．

The records and logs keep the baud rates used at the time (6 Mbps, 3.95 Mbps, ...). The board supports 4 Mbps only.

## License

このリポジトリのコードと文書は MIT ライセンスです ([LICENSE](LICENSE))．`external/digital_servo` は Unitree のリポジトリで，MIT ライセンスには含まれません．そのコードは Unitree の条件 (非商用の研究・教育・DIY 向け) に従います．それを組み込んでビルドしたファームウェアも同じです．

The code and documents in this repository are under the MIT License ([LICENSE](LICENSE)). `external/digital_servo` is Unitree's repository and is not covered: its code is under Unitree's terms (non-commercial research, education and DIY), and so is firmware built with it.
