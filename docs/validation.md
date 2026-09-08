# Validation record

Verified on Linux x86-64 with CMake 3.28.3 and GoogleTest 1.15.2. Each full suite
contains **29 CTest tests**, including **51,004 bit-exact reference cases** and
the same 51,004 inputs checked against independent expected values. Exact
operations allow zero error; transcendental oracle tolerances are documented in
[numerics.md](numerics.md).

| Configuration | Result |
| --- | --- |
| GCC 13.3.0, Debug, Ninja, native wide arithmetic, SSE2 enabled | 29/29 passed |
| Clang 18.1.3, Release, native wide arithmetic, SSE2 enabled | 29/29 passed |
| GCC 13.3.0, Release, portable wide arithmetic, SSE2 disabled | 29/29 passed |
| GCC 13.3.0, Debug, portable wide arithmetic, SSE2 enabled, AddressSanitizer + UndefinedBehaviorSanitizer | 29/29 passed |
| Separate consumer using `add_subdirectory` | Built and ran |
| Separate consumer using an installed `find_package(fxp)` package | Built and ran |
| Every public header included independently | Compiled with GCC and Clang |
| GCC 13.3.0, Ninja Multi-Config Debug, build path containing spaces | 4/4 compile-rejection tests passed |

The reference writer is now forced to use portable scalar C++, with native wide
arithmetic and the SSE2 implementation disabled. Its regenerated results match
all 51,004 original reference rows exactly. The two additional batch tests compare
scalar/SSE2 addition and subtraction directly against the frozen reference file.

The reported VS Code failures were reproduced after the main build changed from
Unix Makefiles to Ninja while old nested compile-rejection caches remained.
Rejection tests now reuse the main build and no longer create or consume those
nested caches. Their positive control also verifies that unrelated floating point
code can compile in a translation unit which includes this library. The rejection
harness was also checked to fail when the selected target unexpectedly compiles
or when the target does not exist, avoiding false passes for those situations.

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

## Comparative benchmarks

The [benchmark results](benchmark-results.md) compare `real`, `fine`, `float`, and
`double` across 35 operations, using identical representable inputs. The report
contains median timings, relative costs, compiler/CPU metadata, input domains,
and methodology. [Raw timings and checksums](benchmark-samples.csv) preserve all
individual samples for inspection.

The recorded run used GCC 13.3.0 Release on an Intel Core Ultra 7 165H, pinned
to logical CPU 1: 4,096 elements, 11 samples, and a 20 ms calibration target.
All 1,540 sample rows and the report's medians, ratios, and variability ranges
were checked against the raw timings. GCC and Clang 18.1.3 also completed short
runs covering every operation and type with 257 elements, exercising batch
tails. Both benchmark builds compiled without warnings. The ordinary 29-test
suite passed after the benchmark/CMake changes; numerical library code was
unchanged.

The comparison uses Release builds and separate repetitions for each type's
calibrated timing interval. It reports array throughput, including compiler
auto-vectorization, and distinguishes numerical semantics and storage sizes.
Both the ordinary operator and explicit batch add/subtract paths must produce
identical checksums. The updated suite uses different inputs and a different
sampling method from the original fixed-only benchmark snapshot.

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
