# Fixed point versus floating point benchmarks

Measured array throughput in **nanoseconds per output value**; lower is faster. Each table reports the median of 11 samples.

| Setting | Value |
| --- | --- |
| Run started (UTC) | 2026-09-08 10:57:56 |
| CPU | Intel Core Ultra 7 165H |
| CPU affinity | Linux logical CPU 1, pinned with taskset -c 1 |
| Platform | Linux x86_64 |
| Compiler | GNU 13.3.0 |
| Build configuration | Release |
| Compiler flags |  -O3 -DNDEBUG |
| real / fine / float / double size | 8 / 8 / 4 / 8 bytes |
| Input count per pass | 4096 |
| Minimum calibration time per sample | 20 ms |
| Samples per type and operation | 11 |
| Fixed wide arithmetic | native where supported |
| Explicit SSE2 batch implementation | enabled |
| Floating point rounding mode | nearest |
| glibc headers | 2.39 |
| libstdc++ headers | 13 |

## Method

Inputs are deterministic pseudorandom multiples of 1/1024, exactly representable by all four types. Setup, conversion, allocation, and checksum calculation are outside the timed regions. Each sample repeats full array passes, with repetition counts calibrated separately for each implementation. Type order rotates between samples. Output barriers and a non-inlined kernel prevent elimination of repeated work; output checksums must remain stable.

Ordinary operators and math functions are called per element. Floating point uses the std overload for the measured type, including float overloads. The explicit batch rows use the library's automatic backend for real/fine; float/double use ordinary array loops. The compiler may auto-vectorize any loop. Comparisons write one byte per element for all four types. Other outputs have the measured type's size, so float uses half the output/input bandwidth of the 64-bit types.

The inputs avoid division by zero, invalid math domains, overflow, floating point underflow, subnormal floating point, and tangent poles. Fixed-point results still undergo normal quantization. These numbers describe ordinary finite inputs, not worst-case timings, saturated arithmetic, very large trig arguments, or dependency-chain latency. The copy row provides a loop/memory baseline.

Fixed point includes its saturation and deterministic rounding rules. Floating point has different range, precision, exception, and cross-platform math behavior. This is a speed comparison, not a claim of equivalent numerical results. Transcendental accuracy is documented separately in numerics.md.

Ratios real/float and fine/double compare the median costs: above 1 means fixed point took longer. Scheduling, turbo, cache state, and libm implementation affect these measurements; sub-nanosecond values reflect vectorized/amortized throughput.

## Basic operations

| Operation | real ns | fine ns | float ns | double ns | real/float | fine/double |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| copy (baseline) | 0.153 | 0.152 | 0.049 | 0.150 | 3.133x | 1.016x |
| + | 0.750 | 0.670 | 0.082 | 0.183 | 9.147x | 3.671x |
| - | 0.562 | 0.793 | 0.081 | 0.183 | 6.926x | 4.321x |
| * | 2.120 | 1.806 | 0.083 | 0.276 | 25.587x | 6.541x |
| / | 2.805 | 8.285 | 0.165 | 0.437 | 16.990x | 18.977x |
| < (byte output) | 0.325 | 0.327 | 0.108 | 0.179 | 3.017x | 1.823x |
| batch add (automatic) | 0.608 | 0.493 | 0.075 | 0.183 | 8.155x | 2.689x |
| batch subtract (automatic) | 0.484 | 0.484 | 0.074 | 0.183 | 6.510x | 2.648x |

## Math functions

| Operation | real ns | fine ns | float ns | double ns | real/float | fine/double |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| abs | 0.787 | 0.775 | 0.058 | 0.160 | 13.500x | 4.850x |
| fmod | 2.235 | 2.187 | 6.690 | 6.688 | 0.334x | 0.327x |
| remainder | 2.259 | 2.265 | 13.610 | 13.562 | 0.166x | 0.167x |
| copysign | 0.602 | 0.600 | 0.099 | 0.217 | 6.097x | 2.769x |
| floor | 0.767 | 0.775 | 0.907 | 0.907 | 0.845x | 0.855x |
| ceil | 0.867 | 0.889 | 0.880 | 0.886 | 0.986x | 1.004x |
| trunc | 0.801 | 0.691 | 0.636 | 0.640 | 1.259x | 1.080x |
| round | 0.895 | 0.749 | 2.244 | 2.172 | 0.399x | 0.345x |
| nearbyint | 1.781 | 1.565 | 0.574 | 0.578 | 3.101x | 2.706x |
| sin | 164.622 | 163.449 | 3.555 | 6.543 | 46.302x | 24.979x |
| cos | 165.437 | 164.414 | 3.844 | 8.358 | 43.033x | 19.670x |
| tan | 136.842 | 136.553 | 6.886 | 5.783 | 19.874x | 23.614x |
| asin | 423.503 | 424.048 | 4.769 | 6.917 | 88.798x | 61.303x |
| acos | 415.239 | 412.341 | 5.929 | 7.284 | 70.040x | 56.610x |
| atan | 195.033 | 195.276 | 5.683 | 5.213 | 34.320x | 37.459x |
| atan2 | 207.518 | 207.356 | 16.942 | 17.915 | 12.249x | 11.575x |
| exp | 128.306 | 126.517 | 1.723 | 3.050 | 74.464x | 41.475x |
| exp2 | 144.095 | 137.310 | 1.635 | 2.079 | 88.132x | 66.043x |
| expm1 | 136.341 | 135.674 | 6.912 | 6.773 | 19.726x | 20.031x |
| log | 165.392 | 162.969 | 1.906 | 2.903 | 86.795x | 56.140x |
| log10 | 170.364 | 167.225 | 4.026 | 6.419 | 42.320x | 26.050x |
| log2 | 164.078 | 163.078 | 1.916 | 2.993 | 85.632x | 54.493x |
| log1p | 163.336 | 161.475 | 6.923 | 7.665 | 23.593x | 21.067x |
| pow (noninteger exponent) | 294.746 | 295.615 | 4.083 | 8.828 | 72.191x | 33.485x |
| sqrt | 100.306 | 148.149 | 0.674 | 1.315 | 148.925x | 112.668x |
| cbrt | 81.754 | 173.708 | 10.165 | 10.517 | 8.042x | 16.517x |
| hypot | 112.917 | 160.541 | 1.547 | 5.976 | 72.984x | 26.863x |

## Sample variability

The five largest within-case max/min ratios are shown below. These are observed sample ranges, not confidence intervals. All samples remain in the CSV; none are discarded. Pinning prevents CPU migration but does not remove scheduling or clock-frequency variation.

| Operation | Type | Minimum ns | Median ns | Maximum ns | Max/min |
| --- | --- | ---: | ---: | ---: | ---: |
| round | real | 0.747 | 0.895 | 1.167 | 1.563x |
| + | fine | 0.610 | 0.670 | 0.844 | 1.383x |
| / | real | 2.531 | 2.805 | 3.303 | 1.305x |
| copy (baseline) | double | 0.148 | 0.150 | 0.192 | 1.299x |
| trunc | fine | 0.681 | 0.691 | 0.873 | 1.283x |

## Input domains

| Operation | Input domain |
| --- | --- |
| copy (baseline) | a,b: [-8,8); b nonzero |
| + | a,b: [-8,8); b nonzero |
| - | a,b: [-8,8); b nonzero |
| * | a,b: [-8,8); b nonzero |
| / | a,b: [-8,8); b nonzero |
| < (byte output) | a,b: [-8,8); b nonzero |
| batch add (automatic) | a,b: [-8,8); b nonzero |
| batch subtract (automatic) | a,b: [-8,8); b nonzero |
| abs | a,b: [-8,8); b nonzero |
| fmod | a,b: [-8,8); b nonzero |
| remainder | a,b: [-8,8); b nonzero |
| copysign | a,b: [-8,8); b nonzero |
| floor | a,b: [-8,8); b nonzero |
| ceil | a,b: [-8,8); b nonzero |
| trunc | a,b: [-8,8); b nonzero |
| round | a,b: [-8,8); b nonzero |
| nearbyint | a,b: [-8,8); b nonzero |
| sin | a,b: [-8,8); b nonzero |
| cos | a,b: [-8,8); b nonzero |
| tan | a: [-1.25,1.25] |
| asin | a: [-1,1] |
| acos | a: [-1,1] |
| atan | a,b: [-8,8); b nonzero |
| atan2 | a,b: [-8,8); b nonzero |
| exp | a: [-4,4] |
| exp2 | a: [-4,4] |
| expm1 | a: [-4,4] |
| log | a,b: [0.0625,8) |
| log10 | a,b: [0.0625,8) |
| log2 | a,b: [0.0625,8) |
| log1p | a: [-0.75,4] |
| pow (noninteger exponent) | a: [0.5,2); b: (-2,2), noninteger |
| sqrt | a,b: [0.0625,8) |
| cbrt | a,b: [-8,8); b nonzero |
| hypot | a,b: [-8,8); b nonzero |

## Reproduction

Build the fxp_benchmark target with FXP_BUILD_BENCHMARKS=ON and a Release configuration. Run --help for options. No fast-math or architecture-specific optimization flags are added by the benchmark target; the compiler flags above record the configured flags. Pinning is performed externally (for example with taskset on Linux); --cpu and --affinity are labels supplied by the caller.

```text
"./build-benchmarks/fxp_benchmark" "--size" "4096" "--samples" "11" "--min-ms" "20" "--cpu" "Intel Core Ultra 7 165H" "--affinity" "Linux logical CPU 1, pinned with taskset -c 1" "--markdown" "docs/benchmark-results.md" "--csv" "docs/benchmark-samples.csv"
```

Raw sample timings and output checksums: docs/benchmark-samples.csv.
