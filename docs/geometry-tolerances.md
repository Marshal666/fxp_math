# Geometry tolerance assessment

The original tests established determinism and checked ordinary geometry cases.
They did **not** establish that clamping every small source tolerance to one
`real` raw unit was an adequate numerical policy. The focused tests added on
2026-09-10 exposed errors in both `real` and `fine`.

## Why increasing the constants did not solve it

One `real` raw unit is `epsilon = 2^-16 = 0.0000152587890625`. Clamping `1e-8`
to epsilon already enlarged its numeric value about 1,526 times; for `1e-6`,
about 15 times. The effect depends on the comparison:

- For `length_squared > cutoff`, a larger cutoff rejects more small rotations.
- For `distance < tolerance`, a larger tolerance admits more points, including
  points beyond a segment's endpoints.

Neither supplies an error budget for preceding arithmetic. In particular,
rounding each product back to `real` can erase a squared length or a dot product
before the comparison happens. Angle extraction through `acos(w)` also loses
small rotations when the stored quaternion component `w` rounds to 1. The
sensitivity of inverse cosine near its endpoints is discussed in Kahan's
[Mangled Angles, section 12](https://people.eecs.berkeley.edu/~wkahan/Mindless.pdf).

These are reproducible examples using exactly representable `real` inputs:

| Case | Initial port | Corrected behavior |
| --- | --- | --- |
| `get_angles((1,t,t))`, `t = 64/65536` | Phi was about -90 degrees | Phi is -45 degrees, rounded to `real`. |
| Decompose a Z rotation of `512/65536` radians | Returned angle 0 | Returns `512/65536` radians and the unit Z axis. |
| Degenerate triangle `(a,c,a)`, with `a=(1,1)`, `c=a+(128,128)/65536`, point `a+(64,64)/65536` | Rejected a point exactly on the segment | Accepts it; off-line points are still excluded. |

Increasing the squared cutoff leaves the first two failures in place: their
rounded intermediate squares were already zero or at the old cutoff. Widening
the distance tolerance enough to cover a broken segment calculation would also
change the intended geometric acceptance region.

## Changes

The fixes address the three helpers that used `1e-8` or `1e-6`:

- `get_angles` compares the unrounded sum of squared raw components against
  `1e-8` and uses `atan2(component,z)`. The source's zero-component and tiny
  projection fallback conventions remain intact.
- Both `quat::decomp_angle_axis` overloads use the same wide threshold check,
  wide axis normalization, and `2*atan2(length(xyz),w)` angle extraction.
- `is_point_inside_triangle` determines orientation from exact signed products
  of raw coordinate differences. Collinear triangles use interval membership;
  points beyond the interval retain the source's strict `distance < 1e-6`
  endpoint slack, evaluated as a wide squared-distance comparison against
  `1e-12`. Coincident vertices retain that distance check too.

No binary floating-point arithmetic is introduced. The comparisons preserve the
nominal decimal source thresholds without first converting them to the public
scalar type. For example, the `1e-8` cutoff in `real` compares an integer sum of
raw squares to `2^32/100000000`: a sum of 42 is below it and 43 is above it.
It does not discard representable component information by rounding each square.

## Independent tests and their limits

[test_geometry_tolerances.cpp](../tests/test_geometry_tolerances.cpp) adds eleven
CTest cases across both formats, including:

- **1,448 independently generated cases** for projection angles and quaternion
  decomposition. Inputs straddle the original cutoff and the old product-rounding
  transitions, include sign/quadrant changes, small rotations, pi, rotations near
  2*pi, and multiple axes. Projection cases also exercise large magnitudes.
- Small-rotation round trips, agreement between both decomposition overloads,
  normalized output axes, exact collinear incidence, and points just inside and
  outside the endpoint tolerance.
- **68,400 comparisons** against bounded integer triangle geometry, translated
  to zero and close to both signed storage limits. Further cases span the entire
  signed coordinate range and distinguish a boundary point from its one-raw-unit
  neighbor.

The oracle generator uses Python integers for threshold decisions and mpmath
at **110 decimal digits** for expected angles and axes. It never calls the C++
library. C++ tests read the checked-in
[oracle CSV](../tests/reference/geometry_tolerance_oracle.csv), so ordinary builds
need neither Python nor mpmath. Regeneration uses:

```sh
python3 tools/generate_geometry_tolerance_oracle.py
ctest --test-dir build -R GeometryTolerance --output-on-failure
```

On the sampled inputs, projection angles and axis components must be within
**one raw unit** of the independently rounded result; quaternion angles may
differ by **two raw units** because vector length and the half-angle are rounded
before doubling. Unit-axis length is checked within two raw units. Expectations
refer to the actual stored coordinates/components, not an unquantized input.
These are tested bounds on this corpus, not a universal guarantee for all
geometry operations. Other composite helpers, including standalone segment
distance and matrix inversion, still have the documented intermediate-rounding
and saturation limits. The remaining `1e-4` and `0.001` source constants were
not increased by this change.

Seven of the first nine new tests failed on the initial implementation, including
the independent oracle. After the fixes they pass; two further tests cover the
exhaustive translated triangles. Frozen geometry reference version 2 appends
144 exact-output cases, preserving all 5,386 earlier rows unchanged. Numerical
results of the corrected helpers can nevertheless change for existing callers;
simulation peers must use the same library version.
