#!/usr/bin/env python3
"""Independent expected results. Requires mpmath 1.3.0 only for regeneration.

Python integers/Fraction define exact operations; 110 decimal digits define
the transcendental oracle. This generator never calls the C++ implementation.
"""
import csv
import random
from fractions import Fraction
from pathlib import Path
import mpmath as mp

mp.mp.dps = 110
LOW, HIGH = -(1 << 63), (1 << 63) - 1
ROOT = Path(__file__).resolve().parents[1]
rng = random.Random(0x46585017)
rows = []

def sat(n):
    return max(LOW, min(HIGH, n))

def nearest(n, d=1):
    if d < 0:
        n, d = -n, -d
    q, r = divmod(abs(n), d)
    q += r * 2 > d or (r * 2 == d and q % 2)
    return -q if n < 0 else q

def emit(f, op, a, b, result, aux=0, tolerance=0):
    rows.append((f, op, a, b, result, aux, tolerance))

def decode(bits, f, mantissa, exponent, bias):
    frac = bits & ((1 << mantissa) - 1)
    exp = (bits >> mantissa) & ((1 << exponent) - 1)
    sign = -1 if bits >> (mantissa + exponent) else 1
    if exp == (1 << exponent) - 1:
        return 'domain' if frac else (LOW if sign < 0 else HIGH)
    shift = (exp or 1) - bias - mantissa + f
    sig = frac + ((1 << mantissa) if exp else 0)
    return sat(nearest(sign * sig * (1 << max(0, shift)), 1 << max(0, -shift)))

def encode(raw, f, mantissa, exponent, bias):
    if not raw:
        return 0
    top = abs(raw).bit_length() - 1
    shift = top - mantissa
    sig = nearest(abs(raw), 1 << shift) if shift > 0 else abs(raw) << -shift
    if sig == 1 << (mantissa + 1):
        top += 1
        sig >>= 1
    return ((raw < 0) << (mantissa + exponent)) | ((top - f + bias) << mantissa) | (sig & ((1 << mantissa) - 1))

def exact(f, op, a, b=0):
    s, aux = 1 << f, 0
    if op == 'add': result = sat(a + b)
    elif op == 'sub': result = sat(a - b)
    elif op == 'mul': result = sat(nearest(a * b, s))
    elif op == 'div': result = sat(nearest(a * s, b)) if b else 'domain'
    elif op in ('fmod', 'remainder', 'remquo'):
        if not b: result = 'domain'
        else:
            q = nearest(a, b) if op != 'fmod' else (abs(a) // abs(b)) * (-1 if (a < 0) != (b < 0) else 1)
            result = a - q * b
            if op == 'remquo': aux = (abs(q) & 127) * (-1 if q < 0 else 1)
    elif op == 'copysign': result = sat(-abs(a) if b < 0 else abs(a))
    elif op == 'abs': result = sat(abs(a))
    elif op == 'integer': result = nearest(a, s)
    elif op == 'convert': result = sat(a << 16) if f == 16 else nearest(a, 1 << 16)
    elif op == 'floor': result = sat(a // s * s)
    elif op == 'ceil': result = sat(-((-a) // s) * s)
    elif op == 'trunc': result = (abs(a) // s * s) * (-1 if a < 0 else 1)
    elif op == 'nearbyint': result = sat(nearest(a, s) * s)
    elif op == 'round': result = sat(((abs(a) + s // 2) // s * s) * (-1 if a < 0 else 1))
    elif op == 'roundtrip': result = a
    elif op == 'ldexp': result = sat(a << b) if b >= 0 else nearest(a, 1 << -b)
    elif op == 'powi':
        if b < 0 and not a: result = 'domain'
        else:
            base = sat(nearest(s * s, a)) if b < 0 else a
            n, result = abs(b), s
            while n:
                if n & 1: result = sat(nearest(result * base, s))
                n >>= 1
                if n: base = sat(nearest(base * base, s))
    else: raise ValueError(op)
    emit(f, op, a, b, result, aux)

def math_case(f, op, a, b=0):
    s = 1 << f
    x, y = mp.mpf(a) / s, mp.mpf(b) / s
    if ((op in ('log', 'log2', 'log10') and a <= 0) or
        (op == 'log1p' and a <= -s) or (op == 'sqrt' and a < 0) or
        (op in ('asin', 'acos') and abs(a) > s) or (op == 'atan2' and a == b == 0) or
        (op == 'pow' and ((a < 0 and b % s) or (a == 0 and b < 0)))):
        emit(f, op, a, b, 'domain')
        return
    if op == 'pow' and b % s == 0:
        exact(f, 'powi', a, b // s)
        row = rows.pop()
        emit(f, op, a, b, row[4])
        return
    if op in ('exp', 'expm1') and x > 64:
        emit(f, op, a, b, HIGH)
        return
    if op in ('exp', 'expm1') and x < -64:
        emit(f, op, a, b, 0 if op == 'exp' else -s)
        return
    if op == 'exp2' and abs(x) > 64:
        emit(f, op, a, b, HIGH if x > 0 else 0)
        return
    if op == 'pow' and a and abs(y * mp.log(x)) > 64:
        emit(f, op, a, b, HIGH if y * mp.log(x) > 0 else 0)
        return
    if op == 'log2': value = mp.log(x, 2)
    elif op == 'log10': value = mp.log10(x)
    elif op == 'exp2': value = mp.power(2, x)
    elif op == 'cbrt': value = mp.sign(x) * mp.root(abs(x), 3)
    elif op == 'hypot': value = mp.sqrt(x*x + y*y)
    elif op == 'atan2': value = mp.atan2(x, y)
    elif op == 'pow': value = mp.power(x, y)
    else: value = getattr(mp, op)(x)
    expected = sat(int(mp.nint(value * s)))
    tolerance = 0 if op in ('sqrt', 'cbrt', 'hypot') else 2
    if op in ('exp', 'exp2', 'expm1'):
        tolerance += int(mp.ceil(abs(expected) * mp.mpf('1e-16')))
    if op == 'pow':
        tolerance += int(mp.ceil(abs(expected) * mp.mpf('1e-16') * (1 + abs(y))))
    if op == 'tan':
        phase = abs(x) * mp.power(2, -128)
        if abs(value) <= 256: phase += mp.power(2, -54)
        tolerance += min(HIGH, int(mp.ceil(s * (1 + value*value) * phase)))
    emit(f, op, a, b, expected, tolerance=min(HIGH, tolerance))

for f in (16, 32):
    s = 1 << f
    edges = [LOW, LOW+1, LOW+s, -2*s, -s, -s+1, -3, -2, -1, 0, 1, 2, 3, s-1, s, s+1, 2*s, HIGH-s, HIGH-1, HIGH]
    pairs = [(a, b) for a in edges for b in edges]
    pairs += [(rng.randint(LOW, HIGH), rng.randint(LOW, HIGH)) for _ in range(350)]
    pairs += [(rng.randint(-100*s, 100*s), rng.randint(-100*s, 100*s)) for _ in range(350)]
    for a, b in pairs:
        for op in ('add', 'sub', 'mul', 'div', 'fmod', 'remainder', 'remquo', 'copysign'):
            exact(f, op, a, b)
    for a in edges + [rng.randint(LOW, HIGH) for _ in range(250)]:
        for op in ('convert', 'integer', 'abs', 'floor', 'ceil', 'trunc', 'round', 'nearbyint', 'roundtrip'):
            exact(f, op, a)
        for width, m, e, bias in ((32,23,8,127), (64,52,11,1023)):
            emit(f, 'to'+str(width), a, 0, encode(a, f, m, e, bias))
    for a in edges:
        for b in (-128, -65, -64, -33, -32, -17, -16, -1, 0, 1, 16, 32, 63, 64, 128):
            exact(f, 'ldexp', a, b)
        for b in (-63, -7, -2, -1, 0, 1, 2, 3, 7, 63, LOW):
            exact(f, 'powi', a, b)
    for width, m, e, bias in ((32,23,8,127), (64,52,11,1023)):
        bits = [0, 1, (1<<m)-1, 1<<m, (bias << m), (1<<width)-1, 1<<(width-1)]
        bits += [(((1<<e)-1)<<m)|payload for payload in (0,1,1<<(m-1))]
        for exponent in range(max(0, bias-f-2), bias+65):
            for fraction in (0, 1, (1<<m)-1, 1<<(m-1)):
                bits.extend(((exponent<<m)|fraction, (1<<(width-1))|(exponent<<m)|fraction))
        bits += [rng.getrandbits(width) for _ in range(800)]
        for bit in bits:
            emit(f, 'from'+str(width), bit, 0, decode(bit, f, m, e, bias))
    decimals = ['0','-0','1.2','4.00002345','.5','1.','1e20','-1e20','1e-50','99999999999999999999999999999999999']
    decimals += [str(rng.randint(-10**40,10**40))+'e'+str(rng.randint(-70,0)) for _ in range(250)]
    for raw in [0,1,2,3,s-1,s, HIGH//2]:
        midpoint = Fraction(2*raw+1, 2*s)
        d = midpoint.denominator.bit_length()-1
        numerator = midpoint.numerator * 5**d
        text = str(numerator).zfill(d+1)
        text = text[:-d]+'.'+text[-d:] if d else text
        decimals.extend((text, text+'0000000000000000000001', '-'+text))
    for text in decimals:
        value = Fraction(text)
        emit(f, 'parse', text, 0, sat(nearest(value.numerator*s, value.denominator)))
    for text in ('nan','inf','1e','1.2.3','0x1','--2'):
        emit(f, 'parse', text, 0, 'invalid')
    broad = edges + [rng.randint(LOW, HIGH) for _ in range(180)]
    moderate = [rng.randint(-32*s, 32*s) for _ in range(200)]
    small = [rng.randint(-s, s) for _ in range(180)] + [-s,-s+1,-1,0,1,s-1,s]
    trig = broad + moderate + small
    for k in [1,2,3,4,10,1000,1000000]:
        center = int(mp.nint(k*mp.pi/2*s))
        trig.extend(sat(center+j) for j in range(-2,3))
    # Continued-fraction convergents find extreme near-pole angles that uniform
    # random sampling almost certainly misses. Include the old Q80 reduction's
    # adversarial cases as well as convergents of a 128-bit pi approximation.
    for precision in (80,128):
        ratio = Fraction(int(mp.nint(mp.pi/2 * 2**precision)), 2**(precision-f))
        n,d = ratio.numerator,ratio.denominator
        h0,h1,k0,k1 = 0,1,1,0
        while d:
            q,n,d = n//d,d,n%d
            h,k = q*h1+h0,q*k1+k0
            if h > HIGH: break
            trig.extend((h-1,h,h+1,-h))
            h0,h1,k0,k1 = h1,h,k1,k
    for op in ('sin','cos','tan','atan'):
        for a in trig: math_case(f,op,a)
    for op in ('asin','acos'):
        for a in small + [s+1,-s-1]: math_case(f,op,a)
    for op in ('exp','exp2','expm1'):
        for a in broad + moderate + [-f*s,(-f-1)*s,47*s,31*s]: math_case(f,op,a)
    for op in ('log','log2','log10','log1p','sqrt','cbrt'):
        for a in broad + moderate + small: math_case(f,op,a)
    for a,b in pairs[:400] + pairs[-200:]:
        math_case(f,'atan2',a,b)
        math_case(f,'hypot',a,b)
    for a,b in [(s+1,HIGH),(s-1,HIGH),(s+1,LOW),(s-1,LOW),(0,0),(-2*s,3*s),(-2*s,s//2)]:
        math_case(f,'pow',a,b)
    for _ in range(250):
        math_case(f,'pow',rng.randint(1,10*s), rng.randint(-5*s,5*s))

path = ROOT/'tests/reference/oracle.csv'
with path.open('w', newline='') as stream:
    stream.write('# fxp_math independent oracle v1; format,operation,a,b,result,auxiliary,tolerance\n')
    csv.writer(stream, lineterminator='\n').writerows(rows)
print(f'Wrote {len(rows)} independent oracle cases to {path}')
