# Fixed-point geometry

These structures and functions are based on Blitzkrieg 2's existing math and
geometry code (`Geom.h`, `GeomMisc.h`, and `Geom.cpp`). They are intended as
deterministic fixed-point replacements using `real` or `fine`, with modernized
names and the behavior changes documented below.

The geometry API is header-only C++17. Include `<fxp/fxp.hpp>` for everything,
or `<fxp/vector.hpp>`, `<fxp/geometry.hpp>`, or `<fxp/geometry_misc.hpp>` for the
individual layers. It has no engine, Boost, or floating-point dependency.

```cpp
#include <fxp/fxp.hpp>
using namespace fxp;
using namespace fxp::literals;

vec3<> position{10, 20, 30};       // real is the default
vec3<fine> precise{1, 2, 3};
auto combined = position + precise; // vec3<fine>

vec3<> velocity{3, 4, 0};
real speed = length(velocity);    // 5
normalize(&velocity);            // returns false for a zero vector

quat<> rotation(pi_v<real> / 2, axis3_z_v<real>);
mat4<> transform(position, rotation);
vec3<> world;
transform.rotate_h_vector(&world, vec3<>(1, 0, 0));

oriented_rect<> rectangle;
rectangle.init_rect(vec2<>(0, 0), vec2<>(1, 0), 2_r, 1_r);
vec2<> hit;
bool intersects = intersect_ray_rect(&hit, vec2<>(-5, 0), vec2<>(1, 0), rectangle);
```

Use `vec2<fine>`, `mat4<fine>`, `quat<fine>`, and so on for Q32.32.
The short vector aliases are `real2`, `real3`, `real4`, `fine2`, `fine3`, and
`fine4`. C++17 requires the angle brackets in `vec3<>`; it cannot deduce an
alias template's arguments. Floating-point coordinates are rejected. Use
integer arguments, fixed-point literals, or the existing IEEE byte conversions.

## Types and migration

| Original type | Replacement |
| --- | --- |
| `CVec2`, `CVec3`, `CVec4` | `vec2<T>`, `vec3<T>`, `vec4<T>` |
| `SVec3Hash` | `vec3_hash<T>`; a deterministic unsigned 64-bit hash |
| `CVecPolar` | `polar<T>` with `latitude`, `longitude` |
| `SVector` | `int_vec2`, with explicit 32-bit integer coordinates |
| `CLine2`, `CSegment`, `CCircle` | `line2<T>`, `segment2<T>`, `circle<T>` |
| `SPlane` | `plane<T>` |
| `SHMatrix`, `SFBTransform` | `mat4<T>`, `transform_pair<T>` |
| `CQuat` | `quat<T>` |
| `CTPoint`, `CTRect` | `point2<T>` (an alias of `vec2<T>`), `aabb2<T>` |
| `STriangle` | `triangle_indices` with three unsigned 16-bit indices |
| `CRay`, `SSphere`, `SMassSphere`, `SBound` | `ray3<T>`, `sphere<T>`, `mass_sphere<T>`, `bound3<T>` |
| `SRect`, `ESide` | `oriented_rect<T>`, `side` |

Methods and helpers use lowercase names with underscores: `FromEulerAngles`
becomes `from_euler_angles`, `RotateHVector` becomes `rotate_h_vector`,
`GetClosestPoint` becomes `get_closest_point`, etc. `Union` becomes `unite`.
The bounds helpers, plane construction/tests, tangent-circle construction,
tangent points, triangle predicates, determinant overloads, transforms, and
quaternion composition/decomposition/interpolation are included.

Vectors have assignable `x`, `y`, `z`, `w` members and checked `operator[]`.
Alternative names use reference accessors: `u()`, `v()`, `r()`, `g()`, `b()`,
`q()`, and `a()` where applicable. Matrices use real array storage and
zero-based `m(row, column)`, `m[row][column]`, or `m.at(row, column)` indexing.
Row and axis helpers remain available. Rectangle coordinates are `minx`, `miny`,
`maxx`, `maxy`; oriented rectangle vertices are `corners[0..3]`, also exposed
through `corner0()` to `corner3()`. Its orientation field is `direction`.
Circle radii are named `radius`.

There are no overlapping union aliases, packed objects, or reinterpreted
vector/matrix references. Defaults initialize to zero; a default quaternion
is the identity rotation. `identity(&matrix)` creates an identity matrix.
The `zero2_v<T>`/`zero3_v<T>`/`zero4_v<T>`, `axis3_x_v<T>` (and corresponding
other dimensions/axes), `zero_polar_v<T>`, and `identity_quat_v<T>` constants
replace the old global constants.

## Arithmetic and conventions

### Differences from common library conventions

Math libraries vary, but these Blitzkrieg 2 conventions deserve attention when
migrating code. The port preserves them:

- Quaternion `a *= b` evaluates `b * a`, reversing the usual compound-assignment
  order. Quaternion division evaluates `conjugate(b) * a` and assumes a unit
  divisor; it is not general division by an arbitrary quaternion.
- Vector `*` means dot product. In 2D, `^` means complex multiplication; in 3D,
  it means cross product. The original vector `fabs` meant length, and rectangle
  `fabs(a,b)` meant a projected gap; the port gives those distinct names.
- Direction codes start at +Y and use a piecewise rational mapping. Equal code
  increments need not represent equal angular increments.
- The source's RH view helper negates both Y and Z axes relative to its LH
  helper. Projections use [0,1] depth, and the direct screen transform includes
  a half-pixel offset. These are engine conventions, not universal defaults.
- Boundary inclusion depends on the shape/query: circle tangency is excluded,
  rectangle/circle tangency is included, and box containment treats X differently
  from Y/Z. Rectangle length/width inputs are half extents.

The source bugs corrected by the port are listed under
[Defined fixes and precision](#defined-fixes-and-precision).

### Operations and transforms

Vector `*` vector is a dot product; vector `*` scalar scales each component.
In **two dimensions**, `a ^ b` and `complex_product(a,b)` multiply complex
numbers; `cross(a,b)` returns the scalar determinant. In **three dimensions**,
`a ^ b` and `cross(a,b)` both return the cross-product vector.

`length` and `length_squared` replace the geometry `fabs`/`fabs2` overloads.
There are scalar-component, vector, and quaternion overloads. Use
`length_xy`, `length_xy_squared`, `length_xyz`, and `length_xyz_squared` for
partial-vector lengths. `normalized(v)` returns a normalized copy; a zero
vector stays zero. `normalize(&v)` returns false without changing a zero vector.
Length sums squares in wide integer arithmetic. Normalization rescales into
Q4.60 internally so both a one-raw-unit vector and a vector at the storage
limits can be normalized without underflow or a saturated intermediate length.

Mixed vector arithmetic, dot/cross products, and matrix/quaternion products
promote to `fine`. Vectors, matrices, and quaternions can be converted in both
directions. Member operations that mutate an object use that object's scalar
format; explicit conversion selects the format for other geometry operations.

Matrix storage is row-major and transformations multiply column vectors.
Translation is in column 3. `rotate_vector` uses only the upper 3x3 block;
`rotate_h_vector(vec3)` applies translation. A four-component result includes
homogeneous `w`; perspective division is explicit. Quaternion matrix
decomposition writes the upper 3x3 block; `mat4(rotation)`/`make_matrix` also
set the remaining affine entries.

The source quaternion conventions are preserved deliberately:

- `a * b` is the Hamilton product.
- **`a *= b` means `a = b * a`.** Compound multiplication prepends a rotation.
- `a / b` and `a /= b` prepend the conjugate of `b`; these rotation-division
  operations require a unit divisor. `inverse` computes the general inverse.
- Euler arguments are yaw about Z, pitch about Y, and roll about X. Composition
  uses the source's coefficients (`qz * qy * qx` for column-vector rotation).
- `slerp` selects the shorter quaternion arc. Its inputs are unit rotations;
  `log` is the rotation-quaternion logarithm, and `exp` exponentiates the vector
  part of a pure-vector quaternion. These preserve the source's rotation use.

`create_view_matrix_lh`/`rh` accept a position plus quaternion, a position plus
matrix, or look-at position/target/up vectors. The source RH convention negates
both the Y and Z view axes. Perspective/orthographic projection helpers preserve
the source's [0,1] depth mapping and LH/RH forward-axis signs.
`create_direct_transform_matrix` includes the source's half-pixel offset.

## Shapes, direction codes, and rasterization

Both symmetric and asymmetric oriented rectangles are supported. Length and
width arguments are **half extents**; the asymmetric overload takes forward
and backward extents separately. Rectangle/rectangle and rectangle/segment
tests exclude touching-only boundaries. Oriented rectangle point containment
excludes edges, while triangle containment includes edges and handles degenerate
triangles. Circle/circle intersection excludes tangency; rectangle/circle
intersection includes it. `aabb2::is_intersect_edges` includes touching edges,
while `is_intersect` does not. Existing bound containment retains its X-inclusive,
Y/Z-exclusive box checks. Negative shape extents/radii are outside the supported
geometric domain.

`intersect_ray_rect` finds the nearest nonnegative intersection with a rectangle
boundary, including exit intersections for interior origins and collinear edge
overlaps. Its output pointer may be null; on failure a supplied output is zero.
Other output-pointer APIs require valid pointers. `projected_distance(a,b)`
preserves the old rectangle `fabs(a,b)` operation: it is the gap projected along
the center-to-center direction, **not** the general minimum Euclidean distance.

`direction_by_vector`, `vector_by_direction`, `direction_difference`,
`direction_difference_sign`, `is_in_the_angle`, `is_in_the_min_angle`,
`z_direction`, `z_angle`, `visible_angle`, and rectangle `get_side` retain the
source's unsigned 16-bit direction representation. These are piecewise rational
direction codes, not uniformly sampled radians:

| Direction | Code |
| --- | ---: |
| +Y | 0 |
| -X | 16384 |
| -Y | 32768 |
| +X | 49152 |

Wrapping is explicitly modulo 65536. Near-zero direction inputs use the source
threshold of 0.0001, rounded to the selected format. `triangle_area2` combines
the two identical signed doubled-area helpers in the source.

`bresenham_circle`, `bresenham_filled_circle`, and `bresenham_ellipse` accept
callbacks and preserve the source's visitation order and repeated points for
positive radii. The circle callbacks receive 32-bit integer coordinates; ellipse
callbacks receive integer X and fixed-point Y. A zero radius emits the center
once. Negative radii and integer-coordinate overflow are rejected. `int_vec2`
provides the original grid turn/snap helpers, truncating integer division,
fixed-vector conversion/normalization, and saturating integer arithmetic;
`grid_distance` is the original maximum-axis distance.

## Packing and old floating-point shortcuts

`pack_dword`, `unpack_high_word`, `unpack_low_word`, and `unpack_byte0` through
`unpack_byte3` use fixed-width unsigned integers. `vec3_to_dword` and
`dword_to_vec3` store signed components in the low three bytes, using
`fixed_to_byte`/`byte_to_fixed`: clamp input to [-1,1], multiply by 127, truncate,
and encode negative integers modulo 256. Decoding treats the byte as signed
without depending on plain `char` signedness; byte 128 decodes to -128/127,
as in the signed-char source behavior.

The `FP_*` macros are replaced with typed functions: `abs_bits` (unsigned raw
magnitude), `sign_bit` (bit 63), `norm_to_byte` (clamped unsigned [0,1] encoding
using `min(255, floor(256*x))`), `reciprocal`, and `exp_negative`. The old
`FP_EXP` formula approximated **exp(-x)** despite its comment; `exp_negative`
uses the deterministic scalar exponential for that same sign convention.
`get_binary32_bits` replaces `GetFloatBits` using integer IEEE encoding. The
existing binary32/binary64 bit and byte APIs remain available.

## Defined fixes and precision

The port fixes the source's successful affine inverse returning false,
non-unit quaternion inverse dividing by length instead of squared length,
incorrect rectangle half-length in the four-corner initializer, incomplete
rectangle/segment and contained-triangle intersection checks, and rectangle
ray hits behind the origin. Matrix multiplication, transpose, and inversion
support in-place use; failed inversions leave the destination unchanged.
Line queries recalculate from their public coefficients rather than trusting
an outdated normalization cache. Vector division divides components directly,
avoiding a reciprocal that can round to zero first.

Quaternion inverse trigonometric arguments are clamped to [-1,1] to handle
quantized unit rotations.
Zero axes/line normals and degenerate look-at bases are rejected where an
operation would divide by zero. Normalize and inverse operations with boolean
failure results retain that form.

Except for wide length/normalization, direction encoding, triangle containment,
and the source threshold comparisons described below, composite geometry follows
the scalar library's rounding and saturation at each arithmetic operation.
Intermediate products and determinants can saturate or
round to zero even when a final mathematical result would fit. In particular,
use appropriately scaled coordinates for dot products, intersection predicates,
and matrix inverses; these are not arbitrary-range exact geometric predicates.
All such results are deterministic. No bit identity with the original floating
implementation, nor a universal geometric accuracy bound, is claimed.

### Source tolerances versus fixed-point resolution

The smallest positive `real` value is `real::epsilon()`, exactly
`2^-16 = 0.0000152587890625`. For `fine`, one raw unit is `2^-32`, approximately
`2.3283064365e-10`. These are absolute steps, including near zero.

The source's **`1e-8` and `1e-6` tolerances are below one `real` raw unit and
both round to zero** on direct conversion. The initial port clamped them to one
raw unit. Focused accuracy tests found that this did not address intermediate
rounding errors; see the [tolerance assessment](geometry-tolerances.md).

The affected helpers now preserve the nominal decimal source thresholds with
wide integer comparisons. A product of raw coordinates retains twice the
fractional bits of the public type, so the threshold need not be stored in a
`real` variable first.

| Source value | Used for | Current treatment in both formats |
| --- | --- | --- |
| `1e-8` | Squared-length checks in quaternion angle/axis decomposition and normal `get_angles` | Compare the unrounded raw sum of squares against `2^(2F) / 100000000`, where `F` is 16 or 32. |
| `1e-6` | Distance slack in degenerate-triangle containment | Exact collinearity and interval checks; outside the interval, compare squared endpoint distance against `1e-12` using wide integers. |
| `1e-4` | `FP_QUAT_EPSILON`: axis validation, quaternion exp/log/slerp; also near-zero direction checks | Round to 7 `real` raw units or 429,497 `fine` raw units. |
| `0.001` | Squared-distance check for point containment in an oriented rectangle classified as degenerate | Round to 66 `real` raw units or 4,294,967 `fine` raw units. |

Thus the `1e-8` squared-length threshold retains its nominal `0.0001` length
scale. The old clamp had raised that scale to `0.00390625` in `real`.
`get_angles` uses `atan2` for the projection angle, and quaternion decomposition
uses `2*atan2(length(xyz), w)` plus wide normalization, preserving small rotations
when `w` rounds to 1. Source fallback conventions still apply below the threshold.
These decisions do not define a universal error bound or a gameplay contact margin.

Original quaternion inversion also used `FP_EPSILON2`, whose definition was
absent from the supplied files, so its original magnitude cannot be confirmed.
The port requires the computed squared norm to be greater than `T::epsilon()`.

## Tests and references

The GoogleTest suite exercises both formats, mixed vector promotion, exact
identities, rotation/matrix round trips, boundary conventions, degeneracies,
aliasing, packing, and integer rasterization. Every non-template method of the
geometry classes is explicitly instantiated for both scalar types.

[geometry.csv](../tests/reference/geometry.csv) contains **5,530 frozen cases**,
including 144 threshold cases added after the tolerance assessment.
Normal tests compare every raw output exactly. The writer always compiles with
`FXP_PORTABLE_ONLY=1` and `FXP_DISABLE_SSE2=1`; native integer acceleration and
other compilers are tested against the portable baseline. Input generation
specifies evaluation order as well as the random seed.

The frozen file records implementation behavior; independent expected identities
and geometric invariants are checked by the unit tests. The original scalar
oracle/reference files are unchanged by this extension. The tolerance suite adds
1,448 independent high-precision angle/axis cases and 68,400 integer triangle
incidence checks; see [coverage and limits](geometry-tolerances.md).
To deliberately revise
the geometry baseline after reviewing an algorithm change:

```sh
cmake --build build --target fxp_geometry_reference_writer --config Release
./build/fxp_geometry_reference_writer tests/reference/geometry.csv
ctest --test-dir build --output-on-failure
```

For Visual Studio use `build/Release/fxp_geometry_reference_writer.exe` and
`ctest -C Release`. Review changed raw outputs before distributing a new
baseline to applications that need simulation compatibility.
