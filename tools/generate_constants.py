#!/usr/bin/env python3
"""Print kernel constants from 110-digit arithmetic (mpmath 1.3.0)."""
import mpmath as mp
mp.mp.dps = 110
for name, value in [('pi',mp.pi), ('half_pi',mp.pi/2), ('quarter_pi',mp.pi/4),
                    ('ln2',mp.log(2)), ('ln10',mp.log(10)), ('tan_pi_8',mp.tan(mp.pi/8))]:
    print(f'{name}: {int(mp.nint(value * 2**60))}')
wide = int(mp.nint(mp.pi/2 * 2**128))
print(f'half_pi_q128: {{{wide >> 128}, {{{(wide >> 64) % 2**64}, {wide % 2**64}}}}}')
