# experiments

2026-10-11 に DYNAMIXEL 版の TTL 基板とサーボで行った通信実験のスクリプトとログです。
Claude の一時フォルダから 2026-10-12 に移しました。どれも `../bench.py` を使います。

| ファイル | 内容 |
| --- | --- |
| `wide_scan.py` / `.log` | ボーレート × ID の広い掃引 (停止コマンド、mode 0)。何か受信したら記録 |
| `fine_scan.py` / `.log` | 応答が出たボーレート付近の細かい掃引 |
| `reply_rate.py` / `.log` | サーボの応答波形から、サーボ側の送信ビット時間を推定 |
| `preamble.py` / `.log` | 停止コマンドの前にダミーの 0xFF を付けた場合と付けない場合の比較 |
| `replay.py`, `latency_replay.py` | 記録したログの再解析 |
| `verify.py`, `supp.py` | 再設計前の検証 (RMT で送信波形を合成し、最初のエッジの遅れを補正)。結果は `*_result.json`、`*_run.txt`、`../../logs/` |
