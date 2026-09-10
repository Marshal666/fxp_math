#!/usr/bin/env python3
"""Independent geometry threshold/accuracy oracle; regeneration needs mpmath 1.3.0.

Inputs and threshold decisions use Python integers. Expected angles and unit
axes use 110 decimal digits. No C++ implementation or binary floats are used.
"""
import csv
import random
from pathlib import Path
import mpmath as mp

mp.mp.dps = 110
ROOT = Path(__file__).resolve().parents[1]
rng = random.Random(0x544F4C4552414E43)
rows = []


def nearest(value):
    lo = int(mp.floor(value))
    fraction = value - lo
    return lo + (fraction > mp.mpf('0.5') or (fraction == mp.mpf('0.5') and lo % 2))


def normal(f, x, y, z):
    scale = 1 << f

    def angle(a):
        if not a:
            return mp.mpf(0)  # Preserve the source's sign(0) convention.
        if (a*a + z*z) * 100000000 < scale*scale:
            return mp.sign(a) * mp.pi / 2
        return mp.atan2(a, z)

    rows.append((f, 'normal', x, y, z, 0,
                 nearest(-angle(y) * scale), nearest(angle(x) * scale), 0, 0))


def quaternion(f, x, y, z, w):
    scale = 1 << f
    square = x*x + y*y + z*z
    if square * 100000000 <= scale*scale:
        result = (0, scale, 0, 0)
    else:
        length = mp.sqrt(square)
        result = (nearest(2 * mp.atan2(length, w) * scale),
                  *(nearest(a * scale / length) for a in (x, y, z)))
    rows.append((f, 'quaternion', x, y, z, w, *result))


for f in (16, 32):
    scale = 1 << f
    # Straddle the original 1e-8 cutoff and the old rounded-square transitions.
    steps = {0, 1, 2, 4, 5, 6, 7, 8, 64, 128, 181, 182, 256, 320, 512, 1024}
    for center in (scale // 10000, int(mp.sqrt(mp.mpf(scale*scale) / 200000000)),
                   int(mp.sqrt(mp.mpf(scale) / 2))):
        steps.update(range(max(0, center - 3), center + 4))
    steps.update(1 << n for n in range(0, 63, 3))
    for step in sorted(steps):
        for sign_y in (-1, 1):
            for sign_z in (-1, 1):
                normal(f, scale, sign_y * step, sign_z * step)
    for _ in range(128):
        normal(f, *(rng.randrange(-scale, scale + 1) for _ in range(3)))
    for z in (-scale, 0, scale):
        normal(f, 0, 0, z)

    # Unit quaternions quantized once, then interpreted from their actual stored
    # components. Include small rotations, pi, and rotations approaching 2*pi.
    angles = [mp.mpf(n) / scale for n in range(1, 2049, 17)]
    angles += [mp.mpf(s) for s in ('0', '0.00019', '0.0002', '0.00021', '0.001',
                                  '0.01', '0.1', '0.5', '1', '2', '3', '6')]
    angles += [mp.pi, 2 * mp.pi - mp.mpf('0.001')]
    for angle in angles:
        for axis in ((0, 0, 1), (1, 1, 1), (2, -3, 6)):
            length = mp.sqrt(sum(a*a for a in axis))
            xyz = [nearest(mp.sin(angle / 2) * a / length * scale) for a in axis]
            quaternion(f, *xyz, nearest(mp.cos(angle / 2) * scale))

path = ROOT / 'tests/reference/geometry_tolerance_oracle.csv'
with path.open('w', newline='') as out:
    out.write('# Independent integer/mpmath geometry oracle v1; '
              'format,operation,x,y,z,w,expected0,expected1,expected2,expected3\n')
    csv.writer(out, lineterminator='\n').writerows(rows)
print(f'Wrote {len(rows)} independent geometry tolerance cases to {path}')
