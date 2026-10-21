#!/usr/bin/env python3
"""Drive the AtomS3R bench firmware (atoms3r/src/main.cpp) and analyse what it captures.

Bench steps -- servo unplugged, J1 power unplugged, jumper from board J4 pin 3 (A) to AtomS3R G5:
    check     wiring sanity: A line visible on G5, RO tri-states while DE is high
    sweep     MAX3485 enable delay vs. idle time before a falling edge (RMT-generated pattern)
    packet    real UART packets at 4 / 6 Mbps on the A line, decoded by an ideal receiver

Servo steps -- jumper removed, servo connected and powered:
    scan      find the servo ID
    servo     error statistics at the nominal baud rate
    margin    sweep our baud rate around nominal to find where errors begin
    latency   DE release and servo response timing, read from the RX pin (G1) with a pull-down

Raw logs are written to atoms3r/logs/.
"""

import argparse
import bisect
import glob
import os
import statistics
import sys
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'python'))
from servo_demo import build_control_packet, parse_feedback_packet  # noqa: E402

TICK_NS = 12.5          # RMT counter at 80 MHz with clk_div 1
UART_SCLK = 80_000_000  # UART source clock (APB)
CMD_LEN = 20
FBK_LEN = 26


# ----------------------------------------------------------------------------------------------
# Serial link

class Bench:
    def __init__(self, port, log):
        self.log = log
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.05
        # Keep DTR/RTS low so opening the USB-Serial-JTAG port does not reset the chip.
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.buf = b''
        for _ in range(20):
            self.ser.reset_input_buffer()
            self.buf = b''
            try:
                lines = self.cmd('PING', timeout=1.0)
            except TimeoutError:
                continue
            if lines and lines[-1].startswith('OK bench'):
                return
        raise RuntimeError('bench firmware did not answer PING on ' + port)

    def cmd(self, line, timeout=30.0):
        self.log.write('> ' + line + '\n')
        self.ser.write((line + '\n').encode())
        out = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.buf += self.ser.read(self.ser.in_waiting or 1)
            while b'\n' in self.buf:
                raw, self.buf = self.buf.split(b'\n', 1)
                s = raw.decode(errors='replace').rstrip('\r')
                self.log.write(s + '\n')
                if s == '#END':
                    self.log.flush()
                    return out
                if s.startswith('#READY'):
                    continue
                out.append(s)
        self.log.flush()
        raise TimeoutError(line)


def find_port():
    ports = sorted(glob.glob('/dev/cu.usbmodem*'))
    if not ports:
        sys.exit('No /dev/cu.usbmodem* port found. Is the AtomS3R plugged in?')
    return ports[0]


# ----------------------------------------------------------------------------------------------
# Waveform helpers

def parse_caps(lines):
    """CAP lines -> list of (idx, clkdiv, runs). runs: list of (level, ns); None marks a frame split."""
    caps = []
    for line in lines:
        if not line.startswith('CAP '):
            continue
        parts = line.split()
        idx, div = int(parts[2]), int(parts[3])
        runs = []
        for tok in parts[4:]:
            if tok == '|':
                runs.append(None)
            else:
                runs.append((1 if tok[0] == 'H' else 0, int(tok[1:]) * TICK_NS * div))
        caps.append((idx, div, runs))
    return caps


class Wave:
    """Piecewise-constant waveform starting at t=0; idle high after the last recorded run."""

    def __init__(self, runs):
        self.split = None in runs
        self.starts, self.levels = [], []
        t = 0.0
        for r in runs:
            if r is None:
                break
            self.starts.append(t)
            self.levels.append(r[0])
            t += r[1]
        self.end = t
        self.edges = self.starts[1:] + [self.end]

    def level(self, t):
        if t >= self.end:
            return 1
        i = bisect.bisect_right(self.starts, t) - 1
        return self.levels[i] if i >= 0 else 1

    def segs(self):
        bounds = self.starts + [self.end]
        return [(bounds[i], bounds[i + 1], self.levels[i]) for i in range(len(self.levels))]

    def nearest_edge(self, t):
        i = bisect.bisect_left(self.edges, t)
        d = [abs(self.edges[j] - t) for j in (i - 1, i) if 0 <= j < len(self.edges)]
        return min(d) if d else float('inf')


def uart_decode(wave, bit_ns, t_from=0.0):
    """Ideal receiver: sync on each falling edge, sample mid-bit.
    Returns list of (byte, frame_ok, margin_ns, t_start). margin = closest sample-to-edge distance."""
    falls = [s for s, e, lvl in wave.segs() if lvl == 0 and s >= t_from]
    out, t, i = [], t_from, 0
    while True:
        while i < len(falls) and falls[i] < t:
            i += 1
        if i >= len(falls):
            return out
        t0 = falls[i]
        samples = [t0 + (k + 0.5) * bit_ns for k in range(10)]
        bits = [wave.level(ts) for ts in samples]
        byte = sum(bits[1 + k] << k for k in range(8))
        margin = min(wave.nearest_edge(ts) for ts in samples)
        out.append((byte, bits[0] == 0 and bits[9] == 1, margin, t0))
        t = samples[9]


def uart_bits(data):
    bits = []
    for b in data:
        bits += [0] + [(b >> k) & 1 for k in range(8)] + [1]
    return bits


def run_lengths(bits):
    runs = []
    for b in bits:
        if runs and runs[-1][0] == b:
            runs[-1][1] += 1
        else:
            runs.append([b, 1])
    return runs


def med(xs):
    return statistics.median(xs) if xs else float('nan')


# ----------------------------------------------------------------------------------------------
# Bench steps (no servo)

def step_check(b, args):
    print(b.cmd('LEVELS')[0], '  (RX should be 1: RO drives the idle-high bus)')

    print('\n[A line on G5] 20 us low pulse from the TX pin (G2)')
    caps = parse_caps(b.cmd('GEN A 1 2 8000 3 d 2 1600 800'))
    b.cmd('PULL A n')
    for idx, _, runs in caps:
        lows = [e - s for s, e, lvl in Wave(runs).segs() if lvl == 0]
        print('  rep %d: low runs (ns) %s' % (idx, [round(x) for x in lows] or 'none'))
    if not any(Wave(r).segs() for _, _, r in caps):
        print('  -> no edges on G5: check the jumper from J4 pin 3 to G5')

    print('\n[RX pin (G1) with pull-down] RO should float (RX low) while DE is high')
    caps = parse_caps(b.cmd('GEN R 1 4 32000 3 d 2 1600 800'))
    b.cmd('PULL R u')
    for idx, _, runs in caps:
        lows = [e - s for s, e, lvl in Wave(runs).segs() if lvl == 0]
        print('  rep %d: low runs (ns) %s' % (idx, [round(x) for x in lows] or 'none'))
    if not any(Wave(r).segs() for _, _, r in caps):
        print('  -> RX never went low: RO is not tri-stating, or something pulls the RX pin up')


def step_sweep(b, args):
    # Ticks: 0.15-4 us, then 6-40 us. The RMT generator drops items whose high half is
    # 8 ticks or shorter, so gaps start at 12; each pattern is verified on the TX pin first.
    gaps = [12] + list(range(16, 321, 8)) + [480, 800, 1600, 3200]
    widths = [int(w) for w in args.widths.split(',')]
    print('Low pulse W from the TX pin, preceded by a gap G (high). Shortfall = W - (low width on A).')
    print('Shortfall after a long idle = enable delay S (Q1 turn-on + MAX3485 enable - DI->A delay).')
    for w in widths:
        durs = []
        for g in gaps:
            durs += [w, g]
        durs.append(w)
        pattern = '%d %s' % (len(durs), ' '.join(map(str, durs)))
        loop = parse_caps(b.cmd('GEN T 1 2 8000 1 n ' + pattern, timeout=30))
        emitted = [round(r[1] / TICK_NS) for r in loop[0][2] if r is not None] if loop else []
        if emitted != durs:
            print('\nW = %d: generator output differs from the pattern, skipped' % w)
            continue
        cmd = 'GEN A 1 2 8000 %d n %s' % (args.reps, pattern)
        caps = parse_caps(b.cmd(cmd, timeout=30))
        per_gap = {g: [] for g in ['idle'] + gaps}
        bad = 0
        for _, _, runs in caps:
            wave = Wave(runs)
            lows = [e - s for s, e, lvl in wave.segs() if lvl == 0]
            if wave.split or len(lows) != len(gaps) + 1:
                bad += 1
                continue
            for i, m in enumerate(lows):
                per_gap['idle' if i == 0 else gaps[i - 1]].append(w * TICK_NS - m)
        print('\nW = %d ticks (%.1f ns): %d/%d captures usable' % (w, w * TICK_NS, len(caps) - bad, len(caps)))
        if bad == len(caps):
            print('  pulses lost or split on A; raw data is in the log')
            continue
        s_full = med(per_gap['idle'])
        print('  S after long idle: %.1f ns' % s_full)
        print('  gap(ns)  shortfall median/min/max (ns)')
        first_off = None
        for g in gaps:
            xs = per_gap[g]
            if not xs:
                continue
            if first_off is None and med(xs) > s_full / 2:
                first_off = g
            if g <= 40 or g % 40 == 0 or g > 320 or (first_off == g):
                print('  %7.0f  %6.1f %6.1f %6.1f' % (g * TICK_NS, med(xs), min(xs), max(xs)))
        if first_off is None:
            print('  DE stayed enabled through every gap up to 40 us')
        else:
            print('  DE is released (shortfall > S/2) once the gap reaches ~%.0f ns' % (first_off * TICK_NS))


def analyse_packet(caps, data, bit_ns):
    sent = list(data)
    exp_runs = run_lengths(uart_bits(data))
    exp_edges, t = [], 0.0
    for lvl, n in exp_runs[:-1]:
        t += n * bit_ns
        exp_edges.append(t)
    good, margins, s_est, reenable = 0, [], [], []
    for _, _, runs in caps:
        wave = Wave(runs)
        segs = [s for s in wave.segs()]
        while segs and segs[0][2] == 1:      # drop idle-high runs if recorded
            segs.pop(0)
        while segs and segs[-1][2] == 1:
            segs.pop()
        if not segs:
            continue
        t0 = segs[0][0]
        frames = uart_decode(wave, bit_ns, t0)
        got = [f[0] for f in frames]
        if got[:len(sent)] == sent and all(f[1] for f in frames[:len(sent)]):
            good += 1
        margins += [f[2] for f in frames[:len(sent)]]
        meas_edges = [s - t0 for s, e, lvl in segs[1:]] + [segs[-1][1] - t0]
        if len(meas_edges) == len(exp_edges):
            offs = [m - x for m, x in zip(meas_edges, exp_edges)]
            # exp_runs[i + 1] is the level after edge i
            rise = [o for i, o in enumerate(offs) if exp_runs[i + 1][0] == 1]
            s = -med(rise)
            s_est.append(s)
            for i, o in enumerate(offs):
                if exp_runs[i + 1][0] == 0 and o > -s / 2:
                    reenable.append(exp_runs[i][1])   # length of the high run before it, in bits
    return good, margins, s_est, reenable


def step_packet(b, args):
    patterns = {
        'stop': build_control_packet(0, 0, 0, 0, 0, 0, 0, 0),
        'move': build_control_packet(0, 1, 1, 0.3, 0.5, 1.234, 10.0, 0.3),
        'stress': bytes.fromhex('FEEEFF7F3F1F0F07030100FFFF80C0E0F0F8FCFE'),
    }
    for baud in [int(x) for x in args.bauds.split(',')]:
        for name, data in patterns.items():
            lines = b.cmd('UTX A 1 2 8000 %d n %d %s' % (args.reps, baud, data.hex().upper()))
            actual = int([l for l in lines if l.startswith('BAUD')][0].split()[1])
            bit_ns = 1e9 / actual
            good, margins, s_est, reenable = analyse_packet(parse_caps(lines), data, bit_ns)
            print('\n%s @ %d (actual %d): decoded OK %d/%d' % (name, baud, actual, good, args.reps))
            if margins:
                print('  ideal-receiver sampling margin: min %.0f ns (perfect = %.0f ns)' % (min(margins), bit_ns / 2))
            if s_est:
                print('  first-edge delay S: %.0f ns (median of %d)' % (med(s_est), len(s_est)))
            else:
                print('  edge count differs from expected (pulses lost?): see log')
            if reenable:
                print('  DE re-enabled mid-packet %d times; preceding high runs (bits): %s'
                      % (len(reenable), sorted(set(reenable))))


# ----------------------------------------------------------------------------------------------
# Servo steps

PATTERNS = {'stop': 0, 'random': 1}


def xchg(b, baud, n, sid, pull='u', period=1000, capevery=0, clkdiv=1, filt=4, idle=32000, dump=0, pattern=0):
    cmd = 'XCHG %d %d %d %s %d %d %d %d %d %d %d' % (baud, n, sid, pull, period, capevery, clkdiv, filt, idle, dump,
                                                     pattern)
    lines = b.cmd(cmd, timeout=10 + n * period / 1e6 * 3 + n * 0.002)
    res = {}
    for l in lines:
        if l.startswith('XRES'):
            res = dict(kv.split('=') for kv in l.split()[1:])
            res = {k: int(v) for k, v in res.items()}
    fbk = [l for l in lines if l.startswith('FBK')]
    return res, (fbk[0] if fbk else None), lines


def fmt_res(r):
    n = r['n']
    return 'ok %d/%d (%.2f%%)  noresp %d  nohdr %d  short %d  crc %d' % (
        r['ok'], n, 100.0 * r['ok'] / n, r['noresp'], r['nohdr'], r['short'], r['crc'])


def step_scan(b, args):
    found = []
    for sid in range(15):
        r, fbk, _ = xchg(b, args.baud, 5, sid, period=2000)
        if r.get('ok'):
            found.append(sid)
            print('id %2d: %s\n       %s' % (sid, fmt_res(r), fbk))
    print('responding IDs:', found or 'none')


def step_servo(b, args):
    r, fbk, _ = xchg(b, args.baud, args.n, args.id, period=args.period, pattern=PATTERNS[args.pattern])
    print('baud %d (%s commands): %s' % (r['baud'], args.pattern, fmt_res(r)))
    print('UART-driver round trip (coarse, us): min %d avg %d max %d' % (r['lat_min'], r['lat_avg'], r['lat_max']))
    print(fbk or 'no valid feedback')


def step_margin(b, args):
    base16 = round(16 * UART_SCLK / args.baud)    # divider in 1/16 steps; 320 at 4 Mbps
    rows = {}
    for side in (1, -1):
        fails = 0
        k = 0 if side == 1 else -1
        while abs(k) <= 48 and fails < 2:
            d16 = base16 + k
            req = int(16 * UART_SCLK / (d16 + 0.5))
            r, _, _ = xchg(b, req, args.n, args.id, period=args.period, pattern=PATTERNS[args.pattern])
            rows[d16] = r
            rate = r['ok'] / r['n']
            print('  div %d/16  baud %8d  (%+.2f%%)  %s' % (d16, r['baud'], 100.0 * (r['baud'] / args.baud - 1), fmt_res(r)))
            fails = fails + 1 if rate < 0.05 else 0
            k += side
    good = [d for d, r in rows.items() if r['ok'] == r['n']]
    if good:
        lo = min(rows[d]['baud'] for d in good)
        hi = max(rows[d]['baud'] for d in good)
        print('\nerror-free window: %d .. %d bps (%+.2f%% .. %+.2f%% of %d)' % (
            lo, hi, 100.0 * (lo / args.baud - 1), 100.0 * (hi / args.baud - 1), args.baud))


def step_latency(b, args):
    r, _, lines = xchg(b, args.baud, args.n, args.id, pull='d', period=3000, capevery=1,
                       clkdiv=args.clkdiv, filt=4, idle=32000)
    b.cmd('PULL R u')
    bit_ns = 1e9 / r['baud']
    print('RX pin pulled down: low while RO floats (DE high), high once DE is released.')
    print('The pull-down takes a few us to bring the floating pin low, so only the end of the')
    print('low (DE release) is sharp; latency is measured from there to the reply start bit.')
    gaps, replies, margins, ncap = [], 0, [], 0
    for idx, div, runs in parse_caps(lines):
        ncap += 1
        wave = Wave(runs)
        # The quiet bus before the reply is the first high run followed by a valid FC EE packet.
        for s, e, lvl in wave.segs():
            if lvl != 1 or e - s < 4 * bit_ns:
                continue
            frames = uart_decode(wave, bit_ns, s)
            data = bytes(f[0] for f in frames)
            if data[:2] == b'\xFC\xEE' and parse_feedback_packet(data[:FBK_LEN]) is not None:
                gaps.append(frames[0][3] - s)
                margins += [f[2] for f in frames[:FBK_LEN]]
                replies += 1
                break
        else:
            print('  #%d: no valid reply after a quiet bus; raw in log' % idx)
        if wave.split:
            print('  #%d: capture split by idle (> %.0f us); rerun with --clkdiv 4' % (idx, 32000 * TICK_NS * div / 1000))
    print('\ncaptures %d, replies decoded from the waveform with valid CRC %d' % (ncap, replies))
    if gaps:
        print('DE release -> reply start: median %.2f us, min %.2f, max %.2f'
              % (med(gaps) / 1000, min(gaps) / 1000, max(gaps) / 1000))
        print('(DE itself releases 0.6-1.7 us after our last rising edge; see the bench sweep)')
    if margins:
        print('reply sampling margin (ideal receiver): min %.0f ns (perfect = %.0f ns)' % (min(margins), bit_ns / 2))
    print('(UART stats during this run are not meaningful: the pull-down makes the RX pin read as a break while we transmit)')


# ----------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('step', choices=['check', 'sweep', 'packet', 'scan', 'servo', 'margin', 'latency', 'raw'])
    ap.add_argument('rawcmd', nargs='?', help='firmware command line for "raw"')
    ap.add_argument('--port')
    ap.add_argument('--baud', type=int, default=4_000_000)
    ap.add_argument('--bauds', default='4000000,6000000', help='packet: comma-separated baud rates')
    ap.add_argument('--widths', default='13,20,40,180', help='sweep: low pulse widths in 12.5 ns ticks')
    ap.add_argument('--reps', type=int, default=8)
    ap.add_argument('--id', type=int, default=0)
    ap.add_argument('-n', type=int, default=None)
    ap.add_argument('--period', type=int, default=1000, help='servo/margin: exchange period in us')
    ap.add_argument('--clkdiv', type=int, default=1, help='latency: capture clock divider')
    ap.add_argument('--pattern', choices=sorted(PATTERNS), default='stop',
                    help='servo/margin: stop commands, or random targets with zero gains and torque '
                         '(the servo goes limp during the test)')
    args = ap.parse_args()
    if args.n is None:
        args.n = {'servo': 3000, 'margin': 300, 'latency': 20}.get(args.step, 100)

    os.makedirs(os.path.join(HERE, '..', 'logs'), exist_ok=True)
    path = os.path.join(HERE, '..', 'logs', time.strftime('%Y%m%d-%H%M%S') + '_' + args.step + '.log')
    with open(path, 'w') as log:
        b = Bench(args.port or find_port(), log)
        if args.step == 'raw':
            print('\n'.join(b.cmd(args.rawcmd, timeout=120)))
        else:
            globals()['step_' + args.step](b, args)
    print('\nlog:', os.path.relpath(path))


if __name__ == '__main__':
    main()
