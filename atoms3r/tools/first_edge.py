#!/usr/bin/env python3
"""First-edge tolerance of the servo, measured through the board with the bench firmware.

Servo connected and powered, G5 jumper removed. Packets are synthesised with the RMT generator
(GEN) instead of the UART, so the first low run can be stretched (D > 0) or shortened (D < 0) by
12.5 ns ticks, i.e. the board's first-edge enable delay S can be cancelled or exaggerated.
Replies are decoded from the RX-pin capture (pull-down: RX low while DE is on).

    delta     D sweep at exactly 4 Mbps -> the D range the servo accepts
    window    baud sweep at a fixed D -> the error-free window with S cancelled
    preamble  one leading byte (contiguous / with an idle gap) at D = 0
    all       baseline (UART vs GEN), delta, window at the centre of the delta range, preamble

Every step uses stop commands and random mode-1 commands with zero gains (the servo goes limp),
and reports the DE release -> reply start time from all decoded replies.
"""

import argparse
import json
import os
import random
import time

import bench
from bench import (Bench, Wave, build_control_packet, find_port, med, parse_caps, parse_feedback_packet,
                   uart_bits, uart_decode, FBK_LEN, TICK_NS)

HERE = os.path.dirname(os.path.abspath(__file__))
REPLY_BIT_NS = 1e9 / 3.996e6   # measured servo transmit rate


def synth(data, bit_ticks, delta=0, pre=b'', gap_bits=0):
    """UART waveform as GEN durations (low first), edges rounded to 12.5 ns ticks."""
    levels = uart_bits(pre) + [1] * gap_bits + uart_bits(data)
    edges = [0.0] + [k * bit_ticks for k in range(1, len(levels)) if levels[k] != levels[k - 1]]
    edges.append(len(levels) * bit_ticks)
    ticks = [round(e) for e in edges]
    durs = [b - a for a, b in zip(ticks, ticks[1:])]
    durs[0] += delta
    if not all(d > 8 for d in durs):     # the generator drops runs of 8 ticks or less
        raise ValueError('run too short for the generator: %s' % durs[:4])
    return durs


def ticks_for(offset_pct):
    return 20.0 / (1 + offset_pct / 100.0)


class Runner:
    def __init__(self, b, sid):
        self.b = b
        self.sid = sid
        self.stop = build_control_packet(sid, 0, 0, 0, 0, 0, 0, 0)
        self.rng = random.Random(1)
        self.gaps = {'stop': [], 'random': []}

    def gen(self, durs, reps):
        lines = self.b.cmd('GEN R 1 4 32000 %d d %d %s' % (reps, len(durs), ' '.join(map(str, durs))), timeout=60)
        ok, gaps = 0, []
        for _, _, runs in parse_caps(lines):
            wave = Wave(runs)
            for s, e, lvl in wave.segs():
                if lvl != 1 or e - s < 4 * REPLY_BIT_NS:
                    continue
                frames = uart_decode(wave, REPLY_BIT_NS, s)
                data = bytes(f[0] for f in frames)
                if data[:2] == b'\xFC\xEE' and parse_feedback_packet(data[:FBK_LEN]) is not None:
                    ok += 1
                    gaps.append(frames[0][3] - s)
                    break
        return ok, gaps

    def rate(self, pattern, bit_ticks, delta=0, n=40, pre=b'', gap_bits=0, per_pkt=10):
        if pattern == 'stop':
            ok, gaps = self.gen(synth(self.stop, bit_ticks, delta, pre, gap_bits), n)
        else:
            ok, gaps = 0, []
            for _ in range(n // per_pkt):
                pkt = build_control_packet(self.sid, 1, 1, 0.0, self.rng.uniform(-50, 50),
                                           self.rng.uniform(-100, 100), 0.0, 0.0)
                o, g = self.gen(synth(pkt, bit_ticks, delta, pre, gap_bits), per_pkt)
                ok += o
                gaps += g
            n = n // per_pkt * per_pkt
        self.gaps[pattern] += gaps
        return ok / n

    def both(self, label, *a, **kw):
        row = {p: self.rate(p, *a, **kw) for p in ('stop', 'random')}
        print('  %-16s stop %3.0f%%  random %3.0f%%' % (label, 100 * row['stop'], 100 * row['random']))
        return row

    def park(self):
        self.gen(synth(self.stop, 20.0), 5)
        self.b.cmd('PULL R u')


def step_baseline(r, args, res):
    print('[baseline] UART vs GEN at D = 0')
    for baud in (4000000, 3890577):
        x, _, _ = bench.xchg(r.b, baud, args.n, args.id)
        print('  UART %d: ok %d/%d' % (x['baud'], x['ok'], x['n']))
        res['uart_%d' % baud] = x['ok'] / x['n']
    for off in (0.0, -2.74):
        res['gen_%+.2f' % off] = r.both('GEN %+.2f%%' % off, ticks_for(off), 0, args.n)


def step_delta(r, args, res):
    print('[delta] first low run stretched by D ticks (12.5 ns) at 4 Mbps')
    rows = {}
    for d in range(args.dmin, args.dmax + 1):
        rows[d] = r.both('D %+3d (%+6.1f ns)' % (d, d * TICK_NS), 20.0, d, args.n)
    good = [d for d, row in rows.items() if min(row.values()) == 1.0]
    res['delta'] = rows
    if good:
        res['delta_centre'] = round((min(good) + max(good)) / 2)
        print('  100%% for both patterns at D = %d..%d (%.1f..%.1f ns); centre D = %d'
              % (min(good), max(good), min(good) * TICK_NS, max(good) * TICK_NS, res['delta_centre']))


def step_window(r, args, res):
    d = args.d if args.d is not None else res.get('delta_centre', 0)
    print('[window] baud sweep at D = %d (%.1f ns)' % (d, d * TICK_NS))
    rows = {}
    for i in range(-14, 15):
        off = i * 0.3
        rows['%+.1f' % off] = r.both('%+5.1f%%' % off, ticks_for(off), d, args.n)
    res['window'] = {'D': d, 'rows': rows}


def step_preamble(r, args, res):
    print('[preamble] one byte before the packet at D = 0, 4 Mbps')
    rows = {}
    for label, pre, gap in (('none', b'', 0), ('00', b'\x00', 0), ('FF', b'\xFF', 0),
                            ('00 + 1 us idle', b'\x00', 4), ('00 + 3 us idle', b'\x00', 12)):
        rows[label] = r.both(label, 20.0, 0, args.n, pre, gap)
    res['preamble'] = rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('step', choices=['delta', 'window', 'preamble', 'all'])
    ap.add_argument('--port')
    ap.add_argument('--id', type=int, default=0)
    ap.add_argument('-n', type=int, default=40, help='exchanges per point and pattern')
    ap.add_argument('--dmin', type=int, default=0, help='delta: first D (negative shortens the first low)')
    ap.add_argument('--dmax', type=int, default=16)
    ap.add_argument('--d', type=int, default=None, help='window: D to use (default: centre of the delta step)')
    args = ap.parse_args()

    os.makedirs(os.path.join(HERE, '..', 'logs'), exist_ok=True)
    stem = os.path.join(HERE, '..', 'logs', time.strftime('%Y%m%d-%H%M%S') + '_first_edge_' + args.step)
    res = {}
    with open(stem + '.log', 'w') as log:
        r = Runner(Bench(args.port or find_port(), log), args.id)
        try:
            steps = ['baseline', 'delta', 'window', 'preamble'] if args.step == 'all' else [args.step]
            for s in steps:
                globals()['step_' + s](r, args, res)
                print()
        finally:
            r.park()
    print('[timing] DE release -> reply start (us)')
    res['reply_gap_us'] = {}
    for pat, g in r.gaps.items():
        if g:
            res['reply_gap_us'][pat] = [min(g) / 1000, med(g) / 1000, max(g) / 1000, len(g)]
            print('  %-6s min %.2f  median %.2f  max %.2f  (n %d)' % (pat, min(g) / 1000, med(g) / 1000,
                                                                     max(g) / 1000, len(g)))
    with open(stem + '.json', 'w') as f:
        json.dump(res, f, indent=1)
    print('\nlog:', os.path.relpath(stem + '.log'))


if __name__ == '__main__':
    main()
