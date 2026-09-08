# Validation record

Verified on Linux x86-64 with CMake 3.28.3 and GoogleTest 1.15.2. Each full suite
contains **27 CTest tests**, including **51,004 bit-exact reference cases** and
the same 51,004 inputs checked against independent expected values. Exact
operations allow zero error; transcendental oracle tolerances are documented in
[numerics.md](numerics.md).

| Configuration | Result |
| --- | --- |
| GCC 13.3.0, Release, native wide arithmetic, SSE2 enabled | 27/27 passed |
| Clang 18.1.3, Release, native wide arithmetic, SSE2 enabled | 27/27 passed |
| GCC 13.3.0, Release, portable wide arithmetic, SSE2 disabled | 27/27 passed |
| GCC 13.3.0, Debug, portable wide arithmetic, SSE2 enabled, AddressSanitizer + UndefinedBehaviorSanitizer | 27/27 passed |
| Separate consumer using `add_subdirectory` | Built and ran |
| Separate consumer using an installed `find_package(fxp)` package | Built and ran |
| Every public header included independently | Compiled with GCC and Clang |

LeakSanitizer was disabled for the local sanitizer run with
`ASAN_OPTIONS=detect_leaks=0`: this execution environment uses tracing, which
LeakSanitizer does not support. Address and undefined behavior checks remained
enabled; `UBSAN_OPTIONS=halt_on_error=1` was used during tests. The ordinary CI
sanitizer job retains its default leak checking.

MSVC and Apple Clang have **not** been run locally. The supplied
[CI workflow](../.github/workflows/ci.yml) configures Windows/MSVC, Linux/GCC,
Linux/Clang, and macOS/Apple Clang, including portable fallbacks. No remote CI
run is claimed by this record. SIMD selection is compile-time, and the default
CI jobs use the architecture provided by each runner.

## Benchmark snapshot

Illustrative measurements from the included benchmark on an Intel Core Ultra 7
165H with GCC 13.3.0 Release (`-O3`, without `-march=native`). Values are elapsed
nanoseconds per output element; arrays contain 4,096 ordinary game-scale values.
These are local microbenchmarks and are sensitive to CPU scheduling, frequency,
cache residency, and concurrent work. They are not latency guarantees or a
measurement of full-range inputs. Compiler auto-vectorization is permitted for
the scalar loops.

| Operation | Q48.16 ns/value | Q32.32 ns/value |
| --- | ---: | ---: |
| Batch add, scalar | 0.533 | 0.524 |
| Batch add, automatic/SSE2 | 0.537 | 0.525 |
| Batch subtract, scalar | 0.581 | 0.632 |
| Batch subtract, automatic/SSE2 | 0.574 | 0.531 |
| Multiply | 2.310 | 2.092 |
| Divide | 2.953 | 4.565 |
| Sine | 166.975 | 153.085 |
| Square root | 108.899 | 155.198 |
| Exponential | 147.727 | 144.417 |
| Natural logarithm | 188.181 | 182.583 |

The scalar and SSE2 batch outputs produce identical checksums. This run shows
that hand-written SSE2 is not automatically faster than the compiler's scalar
loop optimization; retain the selectable backends and measure on the target
hardware. Very large trig angles require more reduction work, near-pole tangent
uses an additional wide division, and portable division can be slower than the
native wide path.

## Reproducing checks

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DFXP_BUILD_BENCHMARKS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/fxp_benchmark

cmake -S . -B build-portable -DCMAKE_BUILD_TYPE=Release \
  -DFXP_PORTABLE_ONLY=ON -DFXP_DISABLE_SSE2=ON
cmake --build build-portable --parallel
ctest --test-dir build-portable --output-on-failure

cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug \
  -DFXP_PORTABLE_ONLY=ON -DFXP_ENABLE_SANITIZERS=ON
cmake --build build-sanitize --parallel
ctest --test-dir build-sanitize --output-on-failure
```

Use `-DCMAKE_CXX_COMPILER=clang++` in a separate build directory for Clang.
Multi-configuration generators require `--config Release` when building and
`-C Release` for CTest. Offline GoogleTest setup is described in the README.
