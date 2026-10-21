import json, os, sys, time
import verify as v
from bench import Bench
path = os.path.join(v.TOOLS, '..', 'logs', time.strftime('%Y%m%d-%H%M%S') + '_verify_supp.log')
res = {}
with open(path, 'w') as log:
    r = v.Runner(Bench(sys.argv[1], log))
    try:
        print('[S1] long runs at 4 Mbps, D = 0')
        for label, pre, gap in (('FF', b'\xFF', 0), ('00+3us', b'\x00', 12), ('00+2us', b'\x00', 8), ('00+1us', b'\x00', 4)):
            row = {}
            for pat, n in (('stop', 200), ('random', 400)):
                ok, n, _ = r.run(pat, 20.0, 0, n, pre, gap)
                row[pat] = '%d/%d' % (ok, n)
            res[label] = row
            print('  %-7s stop %s  random %s' % (label, row['stop'], row['random']))
        print('[S2] FF preamble, baud window (random)')
        res['FFwin'] = {}
        for i in range(-10, 11):
            off = i * 0.3
            ok, n, _ = r.run('random', v.ticks_for(off), 0, 40, b'\xFF', 0)
            res['FFwin']['%+.1f' % off] = ok / n
            print('  %+5.1f%%: %3.0f%%' % (off, 100 * ok / n))
    finally:
        r.stop_servo()
json.dump(res, open('supp_result.json', 'w'), indent=1)
print('log:', path)
