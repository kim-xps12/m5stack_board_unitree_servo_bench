"""Fine baud sweep for one ID with stop commands (mode 0)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import bench

sid, n = 0, 300
log = open(os.path.join(os.path.dirname(__file__), 'fine_scan.log'), 'w')
b = bench.Bench(bench.find_port(), log)
fbk_last = None
for d16 in range(318, 336):              # ~4.02 .. 3.80 Mbps, 1/16 divider steps
    baud = int(16 * bench.UART_SCLK / (d16 + 0.5))
    r, fbk, _ = bench.xchg(b, baud, n, sid, period=1000)
    fbk_last = fbk or fbk_last
    print('div %d/16  baud %8d (%+.2f%%)  %s' % (d16, r['baud'], 100.0 * (r['baud'] / 4e6 - 1), bench.fmt_res(r)), flush=True)
print(fbk_last)
