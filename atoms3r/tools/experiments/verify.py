#!/usr/bin/env python3
"""Pre-redesign verification with the servo connected (G5 jumper removed).

Packets are synthesised with the RMT generator (GEN) instead of the UART, so the first low run
can be stretched by D ticks to cancel the board's first-edge enable delay S.  Replies are decoded
from the RX-pin capture (pull-down: RX low while DE is on, high once DE is released).

  E0  GEN vs UART cross-check at D = 0 (4 Mbps and -2.74 %)
  E1  D sweep at exactly 4 Mbps, stop and random payloads -> servo's tolerance to S
  E2  baud sweep at the best D, stop and random payloads -> window with S cancelled
  E3  DE release -> reply start, stop vs random (mode 0 vs mode 1), from E1/E2 captures
  E4  one leading junk byte (contiguous / 3 us gap) at D = 0 -> does the servo accept it?
"""
import json
import os
import random
import sys
import time

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, TOOLS)
import bench  # noqa: E402
from bench import (Bench, Wave, find_port, parse_caps, uart_bits, uart_decode, med, TICK_NS,  # noqa: E402
                   FBK_LEN, build_control_packet, parse_feedback_packet)

SID = 0
REPLY_BIT_NS = 1e9 / 3.996e6
HERE = os.path.dirname(os.path.abspath(__file__))


def synth(data, bit_ticks, delta=0, pre=b'', gap_bits=0):
    """UART waveform as GEN durations (low first), edges rounded to 12.5 ns ticks."""
    levels = uart_bits(pre) + [1] * gap_bits + uart_bits(data)
    edges = [0.0]
    for k in range(1, len(levels)):
        if levels[k] != levels[k - 1]:
            edges.append(k * bit_ticks)
    edges.append(len(levels) * bit_ticks)
    ticks = [round(e) for e in edges]
    durs = [b - a for a, b in zip(ticks, ticks[1:])]
    durs[0] += delta
    assert levels[0] == 0 and all(d > 8 for d in durs), durs
    return durs


def rand_packet(rng):
    return build_control_packet(SID, 1, 1, 0.0, rng.uniform(-50, 50), rng.uniform(-100, 100), 0.0, 0.0)


STOP = build_control_packet(SID, 0, 0, 0, 0, 0, 0, 0)


class Runner:
    def __init__(self, b):
        self.b = b
        self.gaps = {'stop': [], 'random': []}
        self.rng = random.Random(1)

    def gen(self, durs, reps):
        """Send one waveform reps times; return (#valid replies, DE-release->reply gaps, #DE drops)."""
        cmd = 'GEN R 1 4 32000 %d d %d %s' % (reps, len(durs), ' '.join(map(str, durs)))
        lines = self.b.cmd(cmd, timeout=60)
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

    def run(self, pattern, bit_ticks, delta=0, n=40, pre=b'', gap_bits=0, per_pkt=10):
        ok, gaps = 0, []
        if pattern == 'stop':
            o, g = self.gen(synth(STOP, bit_ticks, delta, pre, gap_bits), n)
            ok, gaps = o, g
        else:
            for _ in range(n // per_pkt):
                o, g = self.gen(synth(rand_packet(self.rng), bit_ticks, delta, pre, gap_bits), per_pkt)
                ok += o
                gaps += g
        self.gaps[pattern] += gaps
        return ok, n, gaps

    def stop_servo(self):
        self.gen(synth(STOP, 20.0), 5)
        self.b.cmd('PULL R u')


def ticks_for(offset_pct):
    return 20.0 / (1 + offset_pct / 100.0)


def main():
    os.makedirs(os.path.join(TOOLS, '..', 'logs'), exist_ok=True)
    path = os.path.join(TOOLS, '..', 'logs', time.strftime('%Y%m%d-%H%M%S') + '_verify.log')
    res = {}
    with open(path, 'w') as log:
        b = Bench(sys.argv[1] if len(sys.argv) > 1 else find_port(), log)
        r = Runner(b)
        try:
            # E0: UART baseline (pattern 0 stop), then GEN at D = 0 for the same rates.
            print('[E0] baseline, D = 0')
            res['E0'] = {}
            x, _, _ = bench.xchg(b, 4000000, 50, SID, period=1000)
            y, _, _ = bench.xchg(b, 3890577, 50, SID, period=1000)
            print('  UART 4000000: ok %d/50   UART 3890577: ok %d/50' % (x['ok'], y['ok']))
            res['E0']['uart'] = {'4000000': x['ok'], '3890577': y['ok']}
            for off in (0.0, -2.74):
                ok, n, _ = r.run('stop', ticks_for(off), 0, 50)
                print('  GEN  %+.2f%%: ok %d/%d' % (off, ok, n))
                res['E0']['gen%+.2f' % off] = ok

            # E1: D sweep at exactly 4 Mbps.
            print('\n[E1] first-low stretch D at 4 Mbps (D in ticks of 12.5 ns)')
            res['E1'] = {}
            for d in range(0, 17):
                row = {}
                for pat in ('stop', 'random'):
                    ok, n, _ = r.run(pat, 20.0, d, 40)
                    row[pat] = ok / n
                res['E1'][d] = row
                print('  D %2d (%5.1f ns): stop %3.0f%%  random %3.0f%%' % (
                    d, d * TICK_NS, 100 * row['stop'], 100 * row['random']))
            good = [d for d, row in res['E1'].items() if min(row.values()) == 1.0]
            if good:
                best = round((min(good) + max(good)) / 2)
            else:
                best = max(res['E1'], key=lambda d: sum(res['E1'][d].values()))
            res['best_delta'] = best
            print('  D giving 100%% for both: %s -> using D = %d (%.1f ns)' % (good, best, best * TICK_NS))

            # E2: baud sweep at the best D.
            print('\n[E2] baud window at D = %d' % best)
            res['E2'] = {}
            for i in range(-14, 15):
                off = i * 0.3
                row = {}
                for pat in ('stop', 'random'):
                    ok, n, _ = r.run(pat, ticks_for(off), best, 40)
                    row[pat] = ok / n
                res['E2']['%+.1f' % off] = row
                print('  %+5.1f%%: stop %3.0f%%  random %3.0f%%' % (off, 100 * row['stop'], 100 * row['random']))

            # E4: leading junk byte at D = 0, 4 Mbps.
            print('\n[E4] one leading byte at D = 0, 4 Mbps')
            res['E4'] = {}
            for label, pre, gap in (('none', b'', 0), ('00', b'\x00', 0), ('FF', b'\xFF', 0),
                                    ('00+3us', b'\x00', 12)):
                row = {}
                for pat in ('stop', 'random'):
                    ok, n, _ = r.run(pat, 20.0, 0, 40, pre, gap)
                    row[pat] = ok / n
                res['E4'][label] = row
                print('  %-7s: stop %3.0f%%  random %3.0f%%' % (label, 100 * row['stop'], 100 * row['random']))
        finally:
            r.stop_servo()

        # E3: response timing from every decoded reply above.
        print('\n[E3] DE release -> reply start (us)')
        res['E3'] = {}
        for pat, g in r.gaps.items():
            if g:
                res['E3'][pat] = {'n': len(g), 'min': min(g) / 1000, 'med': med(g) / 1000, 'max': max(g) / 1000}
                print('  %-6s n %4d  min %.2f  median %.2f  max %.2f' % (pat, len(g), min(g) / 1000,
                                                                          med(g) / 1000, max(g) / 1000))
    out = os.path.join(HERE, 'verify_result.json')
    with open(out, 'w') as f:
        json.dump(res, f, indent=1)
    print('\nlog:', path, '\nresult:', out)


if __name__ == '__main__':
    main()
