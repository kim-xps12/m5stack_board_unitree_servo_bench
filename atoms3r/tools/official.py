#!/usr/bin/env python3
"""Control experiment: the servo/margin steps of bench.py through Unitree's USB single-wire adapter.

Sends the same commands as bench.py (stop commands, or random targets with zero gains and
torque) through the official adapter, so the servo's tolerance can be compared with the
AtomS3R + DYNAMIXEL TTL board.

    python3 tools/official.py servo  --pattern random
    python3 tools/official.py margin --pattern random

With --pattern random the servo goes limp (mode 1, zero torque) during the test; a stop
command (mode 0) is sent at the end.
"""

import argparse
import glob
import os
import random
import sys
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'python'))
from servo_protocol import build_control_packet, parse_feedback_packet  # noqa: E402

UART_SCLK = 80_000_000  # same divider grid as bench.py margin


def find_port():
    ports = sorted(glob.glob('/dev/cu.usbmodem36594BB3197*'))
    if not ports:
        sys.exit('Official adapter (AT32 Virtual Com Port) not found')
    return ports[0]


def command(sid, pattern):
    if pattern == 'random':
        return build_control_packet(sid, 1, 1, 0.0, random.uniform(-50, 50), random.uniform(-100, 100), 0.0, 0.0)
    return build_control_packet(sid, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0)


def run(port, baud, n, sid, pattern):
    s = serial.Serial(port, baud, timeout=0.02)
    time.sleep(0.05)
    s.reset_input_buffer()
    ok = none = bad = 0
    for _ in range(n):
        s.write(command(sid, pattern))
        fb = s.read(26)
        if not fb:
            none += 1
        elif parse_feedback_packet(fb) is None:
            bad += 1
            s.reset_input_buffer()
        else:
            ok += 1
    s.write(build_control_packet(sid, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0))
    s.read(26)
    s.close()
    return ok, none, bad


def fmt(baud, nominal, n, ok, none, bad):
    return 'baud %8d (%+.2f%%)  ok %d/%d (%.2f%%)  noresp %d  bad %d' % (
        baud, 100.0 * (baud / nominal - 1), ok, n, 100.0 * ok / n, none, bad)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('step', choices=['servo', 'margin'])
    ap.add_argument('--port')
    ap.add_argument('--baud', type=int, default=4_000_000)
    ap.add_argument('--id', type=int, default=0)
    ap.add_argument('-n', type=int, default=None)
    ap.add_argument('--pattern', choices=['stop', 'random'], default='random')
    args = ap.parse_args()
    port = args.port or find_port()

    if args.step == 'servo':
        n = args.n or 3000
        print(fmt(args.baud, args.baud, n, *run(port, args.baud, n, args.id, args.pattern)))
        return

    n = args.n or 300
    base16 = round(16 * UART_SCLK / args.baud)
    good = []
    for side in (1, -1):
        fails, k = 0, (0 if side == 1 else -1)
        while abs(k) <= 48 and fails < 2:
            baud = int(16 * UART_SCLK / (base16 + k))
            ok, none, bad = run(port, baud, n, args.id, args.pattern)
            print('  ' + fmt(baud, args.baud, n, ok, none, bad))
            if ok == n:
                good.append(baud)
            fails = fails + 1 if ok < 0.05 * n else 0
            k += side
    if good:
        print('\nerror-free window: %d .. %d bps (%+.2f%% .. %+.2f%% of %d)' % (
            min(good), max(good), 100.0 * (min(good) / args.baud - 1), 100.0 * (max(good) / args.baud - 1),
            args.baud))


if __name__ == '__main__':
    main()
