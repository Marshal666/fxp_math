# fxp_math

A C++17, header-only fixed point library for deterministic game simulation.
The runtime library has no dependencies and performs no floating point arithmetic.

```cpp
#include <fxp/fxp.hpp>
using fxp::real;
using fxp::fine;
using namespace fxp::literals;

real speed = 1.2_r;
fine offset = 4.00002345_fine;
real count = 123;
auto combined = speed + offset; // fine
real rounded = combined;       // allowed; rounds to 16 fractional bits
// real invalid = 1.23;         // error: deleted floating point constructor

auto distance = fxp::hypot(3_r, 4_r); // exactly 5
auto wave = fxp::sin(speed);          // angles are in radians
```

## Build and use

Requires CMake 3.20+ and a C++17 compiler. Supported implementations target GCC,
Clang/Apple Clang, and MSVC 2019+ (including its x64 wide arithmetic intrinsics).
GCC and Clang have been tested locally; the supplied CI matrix also covers MSVC
and Apple Clang. See [validation](docs/validation.md) for what has actually run.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

Tests use `find_package(GTest)` first. When no package is found, CMake downloads
GoogleTest 1.15.2 and verifies its SHA-256. To require an installed package, set
`-DFXP_FETCH_GOOGLETEST=OFF`. For offline builds with unpacked GoogleTest source,
set `-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest-1.15.2`.

In another CMake project:

```cmake
add_subdirectory(path/to/fxp_math)
target_link_libraries(your_game PRIVATE fxp::fxp)
```

Or install with `cmake --install build --prefix /your/prefix`, then use:

```cmake
find_package(fxp CONFIG REQUIRED)
target_link_libraries(your_game PRIVATE fxp::fxp)
```

Tests and examples default to off when this project is a subdirectory. You can
also copy `include/fxp` into your include path and use the headers directly.

## Numeric contract

| Type | Alias | Range | Smallest step |
| --- | --- | --- | --- |
| `fxp::q48_16` | `fxp::real` | −2^47 through 2^47 − 2^−16 | 0.0000152587890625 |
| `fxp::q32_32` | `fxp::fine` | −2^31 through 2^31 − 2^−32 | 0.00000000023283064365386962890625 |

The integer-bit count includes the sign bit. Each object contains one
`std::int64_t`. `T::from_raw(raw)` and `value.raw_value()` provide exact access.
Default construction produces zero. `T::min()` is the most negative value;
`T::max()` is the most positive; `T::epsilon()` is one raw unit.
`std::numeric_limits<T>::lowest()` is the minimum, while its `min()` is epsilon.

Arithmetic and conversions saturate on overflow. Operations that discard
precision round to nearest, with exact halfway cases going to the even raw
integer. There is one zero, no infinity, and no NaN. Division or remainder by
zero and invalid mathematical domains throw `std::domain_error`. No floating
point environment, rounding-mode setting, or platform math library is involved.

Both fixed formats convert implicitly to each other. Mixed arithmetic and
comparisons first convert both operands to the format with more fractional
bits. This is deliberately also true when conversion saturates: a sufficiently
large `real` compares equal to `fine::max()`. Compound assignment computes in
the promoted type, then converts back to the destination type. Integer operands
convert to the fixed operand's format before the operation.

The integer constructors are implicit. Floating point constructors are deleted;
assignment and mixed operators with floating point values are also rejected.
Integer output conversions are explicit and round to nearest, ties to even:

```cpp
auto whole = (2.5_r).to_integer<std::int32_t>(); // 2
auto small = (1000_r).to_integer<std::uint8_t>(); // saturates to 255
auto truncated = fxp::trunc(2.9_r);             // 2_r
```

`static_cast<bool>(value)` tests whether the raw value is nonzero.
`to_integer<bool>()` instead rounds to an integer and tests that rounded value.

## Literals and text

`_r` and `_fine` are constexpr decimal literals. Character packs are parsed
directly; the compiler never constructs a floating point value. Decimal
exponents and digit separators in the significand are supported:

```cpp
constexpr real step = 1e-3_r;
constexpr fine large = 1'234.5_fine;
auto value = real::from_string("-12.0000152587890625");
std::string text = value.to_string();
real restored(text);
```

`fxp::from_string<T>()`, `fxp::to_string()`, explicit conversion to `std::string`,
and stream insertion/extraction are provided. `to_string()` produces the exact
terminating decimal, with no unnecessary fractional trailing zeros. Parsing it
back recovers the same raw value. Parsing supports a leading sign, optional
decimal point, and exponent. Whitespace, hexadecimal syntax, NaNs, infinities,
and malformed strings are rejected with `std::invalid_argument`; range overflow
saturates. Input length is capped at 1,000,000 characters.

A leading minus on a C++ literal is a separate unary operation. At the negative
range endpoint, use `T::min()` or a signed string, because the corresponding
positive literal has already saturated before unary negation runs.

## Math API

All functions are in `fxp`; unary functions take a fixed type. Binary functions
accept mixed formats and integer arguments when at least one operand is fixed.

| Category | Functions/operators |
| --- | --- |
| Arithmetic | `+ - * / %`, unary `+ -`, compound assignments, increment/decrement, all comparisons |
| Basic | `abs fabs fmod remainder copysign remquo fdim fmin fmax` |
| Rounding/scaling | `floor ceil trunc round nearbyint rint modf ldexp scalbn frexp nextafter` |
| Classification | `signbit isfinite isnan isinf` |
| Trigonometry | `sin cos tan sincos` |
| Inverse trig | `asin acos atan atan2` |
| Exponential/log | `exp exp2 expm1 log log10 log2 log1p` |
| Power/root | `pow sqrt cbrt hypot` |
| Constants | `pi_v<T> e_v<T>`; default `T` is `real` |

`sincos(x)` returns a pair containing sine and cosine and shares angle reduction.
`hypot` supports two promoted operands, or three operands of the same fixed
type. Roots and hypot keep wide intermediates, avoiding premature saturation.

`round` follows the conventional named function's halfway-away-from-zero rule;
`nearbyint` and `rint` use ties to even. Neither depends on the host rounding
mode. `fmod` uses a quotient truncated toward zero. `remainder` and `remquo` use
a nearest-even quotient; `remquo` writes the signed low **seven** quotient bits.
`frexp` returns a rounded fixed mantissa in [0.5, 1) in magnitude, with a signed
exponent; fixed mantissa precision can make reconstruction lossy for large inputs.

`atan2(0, 0)` throws. Logarithms require positive arguments; `log1p` requires
`x > -1`; `sqrt` requires `x >= 0`; `asin` and `acos` require `-1 <= x <= 1`.
`pow(0, 0)` is one. A negative base requires an integer exponent. Zero to a
negative power throws. Integer powers use exponentiation by squaring, rounding
and saturating each multiply; a negative exponent starts with the rounded
reciprocal. Noninteger powers use integer log/exp kernels. Pointer outputs must
be nonnull or the function throws `std::invalid_argument`.

Arithmetic, decimal/IEEE conversions, `sqrt`, `cbrt`, and `hypot` have exact
rounding definitions. Transcendentals use deterministic approximations. Their
output bits are reproducible, but they are **not guaranteed correctly rounded
for every input**. See [algorithm and accuracy notes](docs/numerics.md) and the
[measured error table](docs/accuracy-results.md). In particular, tangent near a
pole magnifies small angle errors; use an application-appropriate domain.

## IEEE 754 interchange without floating point

The interchange functions accept integer bit patterns or arrays of exactly
four/eight `std::uint8_t` bytes. Byte APIs require explicit byte order and never
depend on host endianness:

```cpp
std::array<std::uint8_t, 4> network_bytes{0x3f, 0xc0, 0x00, 0x00};
real value = fxp::from_ieee754_binary32_bytes<real>(
    network_bytes, fxp::byte_order::big_endian); // exactly 1.5
auto binary64 = fxp::to_ieee754_binary64_bytes(
    value, fxp::byte_order::little_endian);
auto bits = fxp::to_ieee754_binary32_bits(value); // std::uint32_t
fine decoded = fxp::from_ieee754_binary64_bits<fine>(
    UINT64_C(0x3ff8000000000000));
```

The corresponding `from_ieee754_binary32_bits` and `to_ieee754_binary64_bits`
functions complete the API. Exponents, significands, sign bits, and rounding
are handled entirely with integers. Signed zero becomes fixed zero; export
produces positive zero. Both quiet and signaling NaNs throw without evaluating
them as floating point. Infinities and out-of-range finite values saturate.
Subnormals are handled and round to zero in these fixed formats. Export rounds
to nearest even; exporting a large `fine` value to binary32 can lose precision.

## Performance and portability

Scalar storage and the reference wide algorithms use `std::int64_t` and
`std::uint64_t`. The default fast path uses an internal compiler 128-bit integer
on GCC/Clang and `_umul128`/`_udiv128` on MSVC x64. These are isolated in
`detail/wide.hpp`; no extension types appear in the public API. Setting
`-DFXP_PORTABLE_ONLY=ON` selects the standard C++ two-word implementations.

`fxp::batch::add` and `subtract` have integer-only SSE2 implementations with the
same saturation semantics. `multiply` and `divide` provide scalar batch loops.

```cpp
fxp::batch::add(a.data(), b.data(), out.data(), out.size());
fxp::batch::add(a.data(), b.data(), out.data(), out.size(),
                fxp::batch::backend::scalar);
```

SSE2 is selected at compile time when the target supports it; there is no runtime
CPU dispatcher. `batch::has_sse2` reports availability. Disable it with
`-DFXP_DISABLE_SSE2=ON`. No 16-byte alignment is required. Exact in-place output
aliasing with either input is supported; partially overlapping ranges are not.
For zero count, null pointers are allowed. Division can throw after writing a
prefix of the output. All feature macros must be consistent across translation
units. The portable-wide and SSE2 options are independent.

The compiler can auto-vectorize scalar loops, so SSE2 is not necessarily faster
on every workload. Build the included benchmark with
`-DFXP_BUILD_BENCHMARKS=ON`, then run `build/fxp_benchmark` (or the executable in
`build/Release` on Visual Studio). See [local measurements](docs/validation.md).

## Determinism tests

`tests/reference/results.csv` contains 51,004 frozen output cases checked with
zero bit tolerance on every platform. `oracle.csv` contains independently
computed results and accuracy tolerances. Tests also cover compile-time literals,
floating point compile rejection, saturation boundaries, mixed promotion,
endian conversion, domain errors, and SSE2/scalar agreement.

The reference files are inputs to normal tests and are never regenerated by
CMake or CTest. A platform passes only when **100% of its tests pass**. Re-run
the same version's suite when changing compiler, architecture, or feature flags.
See [reference maintenance](docs/numerics.md#reference-maintenance) for deliberate
regeneration and compatibility rules.
