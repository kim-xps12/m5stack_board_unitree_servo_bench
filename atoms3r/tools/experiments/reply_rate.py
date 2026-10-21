"""Capture servo replies on the RX pin and estimate the servo's own TX bit time."""
import os, sys, statistics
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import bench

baud = int(sys.argv[1]) if len(sys.argv) > 1 else 3890577
log = open(os.path.join(os.path.dirname(__file__), 'reply_rate.log'), 'w')
b = bench.Bench(bench.find_port(), log)
r, fbk, lines = bench.xchg(b, baud, 20, 0, pull='u', period=3000, capevery=1, clkdiv=1, filt=4, idle=32000)
print('baud', r['baud'], bench.fmt_res(r))
ests, quiet = [], []
for idx, div, runs in bench.parse_caps(lines):
    w = bench.Wave(runs)
    segs = w.segs()
    # our command: first low .. last edge of the TX burst; reply: after a high run > 4 bits
    T0 = 1e9 / r['baud']
    for i, (s, e, lvl) in enumerate(segs):
        if lvl == 1 and e - s > 4 * T0 and i > 0:
            reply = [x for x in segs[i + 1:]]
            break
    else:
        print('#%d no reply' % idx); continue
    if not reply:
        print('#%d no reply' % idx); continue
    t0 = reply[0][0]
    edges = [x[0] - t0 for x in reply[1:]] + [reply[-1][1] - t0]
    T = 250.0
    for _ in range(5):
        ns = [round(t / T) for t in edges]
        T = sum(t * n for t, n in zip(edges, ns)) / sum(n * n for n in ns)
    resid = max(abs(t - n * T) for t, n in zip(edges, ns))
    span = ns[-1]
    ests.append(T); quiet.append(s and (e - s))
    print('#%d reply edges %d over %d bits: bit %.2f ns -> %.0f bps, max resid %.0f ns, quiet before reply %.2f us'
          % (idx, len(edges), span, T, 1e9 / T, resid, (e - s) / 1000))
if ests:
    m = statistics.median(ests)
    print('\nservo TX bit time median %.2f ns -> %.0f bps (%+.2f%% vs 4 Mbps)' % (m, 1e9 / m, 100 * (1e9 / m / 4e6 - 1)))
