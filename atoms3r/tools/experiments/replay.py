"""Re-run bench.py analyses on a recorded log (no hardware)."""
import sys, types
sys.path.insert(0, '/Users/b-sky-lab/Projects/digital_servo/atoms3r/tools')
import bench

class Replay:
    def __init__(self, path):
        self.q = []
        cur = None
        for raw in open(path, errors='replace'):
            s = raw.rstrip('\n')
            if s.startswith('> '):
                cur = [s[2:], []]
                self.q.append(cur)
            elif cur and s not in ('#END',) and not s.startswith('#READY'):
                cur[1].append(s)
    def cmd(self, line, timeout=0):
        while self.q:
            c, out = self.q.pop(0)
            if c == line:
                return out
        raise RuntimeError('not in log: ' + line)

step, path = sys.argv[1], sys.argv[2]
args = types.SimpleNamespace(widths='13,20,40,180', reps=8, bauds=sys.argv[3] if len(sys.argv) > 3 else '4000000')
getattr(bench, 'step_' + step)(Replay(path), args)
