"""Reply timing (DE release -> reply start) from a recorded latency log."""
import os, sys, statistics
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import bench
lines = [l.rstrip('\n') for l in open(sys.argv[1], errors='replace')]
baud = int([l for l in lines if l.startswith('> XCHG')][0].split()[2])
bit = 1e9 / baud
gaps, rel = [], []
for idx, div, runs in bench.parse_caps(lines):
    w = bench.Wave(runs)
    segs = w.segs()
    for s, e, lvl in segs:
        if lvl != 1 or e - s < 4 * bit:
            continue
        fr = bench.uart_decode(w, bit, s)
        d = bytes(f[0] for f in fr)
        if d[:2] == b'\xFC\xEE':
            gaps.append(fr[0][3] - s)
            break
print('%s baud %d: DE release -> reply start median %.2f us (n=%d)' % (sys.argv[1], baud, statistics.median(gaps) / 1000, len(gaps)))
