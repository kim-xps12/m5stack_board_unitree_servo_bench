#!/usr/bin/env python3
"""TXEN timing of the Unitree servo board, read on TP1 with the bench firmware.

Bench step -- servo and J1 power unplugged or not (TXEN is 3.3 V logic either way), jumper from TP1
(TXEN, U1 output) to AtomS3R G5. TXEN is high while the board drives the bus.

    hold      single low pulses of W on the TX pin: TXEN high time - W = hold after the last rising
              edge (design: 3.27-7.30 us, typical 5.2 us)
    gap       low 250 ns, high G, low 250 ns: the gap G at which TXEN drops in between
    packet    real UART packets: TXEN must stay high through the whole packet (one high run), then
              hold after the last rising edge

The TX-fall -> TXEN-rise delay (a few ns) is not visible here, so hold reads a few ns short.
Raw logs are written to atoms3r/logs/.
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bench  # noqa: E402
from bench import TICK_NS, med, parse_caps, uart_bits  # noqa: E402


def high_runs(runs):
    """Lengths (ns) of the high runs in one capture, up to a frame split."""
    out = []
    for r in runs:
        if r is None:
            break
        if r[0] == 1:
            out.append(r[1])
    return out


def step_hold(b, args):
    print('Low pulse W on TX, then idle. Hold = TXEN high time - W.')
    print('  W (ns)   hold median / min / max (us)   captures')
    for w in (20, 80, 720):                     # 1 bit at 4 Mbps, 1 us, 9 us
        caps = parse_caps(b.cmd('GEN A 1 2 8000 %d n 2 %d 1600' % (args.reps, w), timeout=30))
        holds = []
        for _, _, runs in caps:
            hs = high_runs(runs)
            if len(hs) == 1:
                holds.append(hs[0] - w * TICK_NS)
        if holds:
            print('  %6.0f   %5.2f / %5.2f / %5.2f              %d/%d' % (
                w * TICK_NS, med(holds) / 1e3, min(holds) / 1e3, max(holds) / 1e3, len(holds), len(caps)))
        else:
            print('  %6.0f   no single high run on G5: check the jumper from TP1 to G5' % (w * TICK_NS))


def step_gap(b, args):
    print('Low 250 ns, high G, low 250 ns. TXEN high runs: 1 = held through the gap, 2 = released.')
    first_release = None
    for g in range(160, 801, 16):               # 2.0-10.0 us in 0.2 us steps
        caps = parse_caps(b.cmd('GEN A 1 2 8000 %d n 4 20 %d 20 1600' % (args.reps, g), timeout=30))
        counts = [len(high_runs(runs)) for _, _, runs in caps]
        released = sum(1 for c in counts if c == 2)
        if released and first_release is None:
            first_release = g
        if g % 80 == 0 or released not in (0, len(counts)):
            print('  G %5.2f us: released %d/%d' % (g * TICK_NS / 1e3, released, len(counts)))
        if released == len(counts) and g >= (first_release or 0) + 80:
            break
    if first_release is None:
        print('  TXEN held through every gap up to 10 us')
    else:
        print('  TXEN first released at G = %.2f us' % (first_release * TICK_NS / 1e3))


def step_packet(b, args):
    packets = {
        'stop': bench.build_control_packet(0, 0, 0, 0, 0, 0, 0, 0),
        'stress': bytes.fromhex('FEEEFF7F3F1F0F07030100FFFF80C0E0F0F8FCFE'),
    }
    for baud in [int(x) for x in args.bauds.split(',')]:
        for name, data in packets.items():
            lines = b.cmd('UTX A 1 2 8000 %d n %d %s' % (args.reps, baud, data.hex().upper()), timeout=30)
            actual = int([l for l in lines if l.startswith('BAUD')][0].split()[1])
            bits = uart_bits(data)
            last_rise = max(i for i in range(1, len(bits)) if bits[i] == 1 and bits[i - 1] == 0)
            active = last_rise * 1e9 / actual       # first falling edge -> last rising edge
            caps = parse_caps(lines)
            runs = [high_runs(r) for _, _, r in caps]
            single = [hs[0] for hs in runs if len(hs) == 1]
            print('%s @ %d (actual %d): TXEN one high run in %d/%d packets' % (name, baud, actual, len(single), len(runs)))
            if single:
                holds = [h - active for h in single]
                print('  hold after the last rising edge: median %.2f us (min %.2f, max %.2f)' % (
                    med(holds) / 1e3, min(holds) / 1e3, max(holds) / 1e3))
            broken = [hs for hs in runs if len(hs) != 1]
            if broken:
                print('  TXEN dropped mid-packet; high runs (us): %s' % [[round(h / 1e3, 2) for h in hs] for hs in broken[:3]])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('step', choices=['hold', 'gap', 'packet', 'all'])
    ap.add_argument('--port')
    ap.add_argument('--reps', type=int, default=16)
    ap.add_argument('--bauds', default='4000000', help='packet: comma-separated baud rates')
    args = ap.parse_args()
    os.makedirs(os.path.join(HERE, '..', 'logs'), exist_ok=True)
    path = os.path.join(HERE, '..', 'logs', time.strftime('%Y%m%d-%H%M%S') + '_txen_' + args.step + '.log')
    with open(path, 'w') as log:
        b = bench.Bench(args.port or bench.find_port(), log)
        b.cmd('PULL A n')
        for step in (['hold', 'gap', 'packet'] if args.step == 'all' else [args.step]):
            print('\n[%s]' % step)
            globals()['step_' + step](b, args)
    print('\nlog:', os.path.relpath(path))


if __name__ == '__main__':
    main()
