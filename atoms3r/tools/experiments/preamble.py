"""Stop command with / without a dummy 0xFF preamble (mode 0 only; the servo does not move)."""
import os, sys
sys.path.insert(0, '/Users/b-sky-lab/Projects/digital_servo/atoms3r/tools')
import bench
from servo_demo import build_control_packet, parse_feedback_packet

reps = 50
stop = build_control_packet(0, 0, 0, 0, 0, 0, 0, 0)
log = open(os.path.join(os.path.dirname(__file__), 'preamble.log'), 'w')
b = bench.Bench(bench.find_port(), log)
for baud in [int(x) for x in sys.argv[1].split(',')]:
    for name, pre in [('none', b''), ('FF', b'\xff'), ('FFFF', b'\xff\xff')]:
        lines = b.cmd('UTX R 4 4 32000 %d u %d %s' % (reps, baud, (pre + stop).hex().upper()), timeout=60)
        ok = 0
        for l in lines:
            if l.startswith('RXB'):
                data = bytes.fromhex(l.split()[2]) if len(l.split()) > 2 else b''
                i = data.find(b'\xfc\xee')
                if i >= 0 and parse_feedback_packet(data[i:i + bench.FBK_LEN]) is not None:
                    ok += 1
        actual = [l for l in lines if l.startswith('BAUD')][0].split()[1]
        print('baud %s  preamble %-5s ok %2d/%d' % (actual, name, ok, reps), flush=True)
