#!/usr/bin/env python3
"""Compare the frozen implementation results to the independent oracle."""
import csv
from collections import defaultdict
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
def read(name):
    with (ROOT/'tests/reference'/name).open() as stream:
        return list(csv.reader(line for line in stream if not line.startswith('#')))
oracle, actual = read('oracle.csv'), read('results.csv')
assert len(oracle) == len(actual)
stats = defaultdict(lambda: [0,0,0])
for o, a in zip(oracle, actual):
    assert o[:4] == a[:4]
    assert o[5] == a[5]
    key = (int(o[0]), o[1])
    stats[key][0] += 1
    if o[4] in ('domain','invalid'):
        assert o[4] == a[4]
        continue
    error = abs(int(o[4]) - int(a[4]))
    assert error <= int(o[6]), (o,a,error)
    stats[key][1] = max(stats[key][1], error)
    stats[key][2] += error != 0
print('| Format | Operation | Cases | Largest raw error | Nonexact cases |')
print('| --- | --- | ---: | ---: | ---: |')
for (f, op), (count, error, different) in sorted(stats.items()):
    print(f'| Q{64-f}.{f} | {op} | {count} | {error} | {different} |')
