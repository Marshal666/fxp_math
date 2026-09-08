# Comparison benchmark

`fxp_benchmark` measures `real`, `fine`, `float`, and `double` for 35 operations.
It calls `fxp` functions for fixed point and the corresponding `std` overloads
for floating point. It has no third-party benchmark dependency.

```sh
cmake -S . -B build-benchmarks -DCMAKE_BUILD_TYPE=Release \
  -DFXP_BUILD_BENCHMARKS=ON -DFXP_BUILD_TESTS=OFF -DFXP_BUILD_EXAMPLES=OFF
cmake --build build-benchmarks --config Release --parallel
./build-benchmarks/fxp_benchmark --markdown docs/benchmark-results.md \
  --csv docs/benchmark-samples.csv
```

On Visual Studio, the executable is normally under `build-benchmarks/Release`.
Do not use Debug numbers as a performance comparison. Floating point exists
only in this benchmark executable; the library and scalar reference writer keep
their integer-only arithmetic.

| Option | Default | Meaning |
| --- | --- | --- |
| `--size` | 4096 | Elements in each input/output array |
| `--samples` | 7 | Odd number of measured samples per type/operation |
| `--min-ms` | 10 | Calibration target per sample; repetitions double until reached |
| `--markdown` | none | Write a Markdown report in addition to standard output |
| `--csv` | none | Write each sample's time, repetition count, and output checksum |
| `--cpu` | not recorded | CPU description to include in the report |
| `--affinity` | not pinned | Description of externally configured CPU affinity |

The last two options only label the report; they do not change system settings.
For example, on Linux with logical CPU 1 available:

```sh
taskset -c 1 ./build-benchmarks/fxp_benchmark \
  --cpu 'Your CPU model' --affinity 'CPU 1, taskset -c 1' \
  --markdown docs/benchmark-results.md --csv docs/benchmark-samples.csv
```

Inputs use a fixed seed and multiples of 1/1024 in operation-specific domains.
All four types start with exactly the same values. In particular, the generic
power benchmark uses noninteger exponents for every type, so it exercises the
fixed log/exp path. No input generation, conversions, allocations, or checksum
calculations are timed. Output barriers and non-inlined array kernels preserve
repeated work. Checksums are verified after each sample, and the explicit batch
paths must agree with the ordinary addition/subtraction operators.

The compiler may vectorize loops, and a float array occupies half the space of
the other arrays. The report measures array throughput, not serial dependency
latency or guaranteed per-call cost. CPU turbo, cache residency, the system math
library, and other processes can affect results. Compare Release runs with the
same sample settings and input counts; consult the raw samples for variability.

The generated report includes CMake's compiler identity, platform, active build
configuration, configured global/configuration compiler flags, backend settings,
and library-header versions where available. The benchmark adds no fast-math
or architecture-specific optimization flags. As with other CMake targets,
flags supplied by the caller still apply.
