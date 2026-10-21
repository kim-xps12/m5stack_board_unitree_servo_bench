"""Wide baud x ID sweep with stop commands (mode 0); reports any received bytes at all."""
import os, sys, time
sys.path.insert(0, '/Users/b-sky-lab/Projects/digital_servo/atoms3r/tools')
import bench

log = open(os.path.join(os.path.dirname(__file__), 'wide_scan.log'), 'w')
b = bench.Bench(bench.find_port(), log)
hits = []
for d16 in range(304, 338, 2):          # ~4.21 .. 3.79 Mbps
    baud = int(16 * bench.UART_SCLK / (d16 + 0.5))
    for sid in range(15):
        r, fbk, _ = bench.xchg(b, baud, 3, sid, period=2000, dump=1)
        rx = r['n'] - r['noresp']
        if rx:
            hits.append((r['baud'], sid, r))
            print('baud %d id %d: %s  %s' % (r['baud'], sid, bench.fmt_res(r), fbk or ''))
    print('baud %d done' % baud, flush=True)
print('hits:', len(hits))
