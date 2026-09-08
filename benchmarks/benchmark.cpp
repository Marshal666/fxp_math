#include <fxp/fxp.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cfenv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "benchmark_config.hpp"

#if defined(_MSC_VER)
#include <intrin.h>
#define FXP_BENCH_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define FXP_BENCH_NOINLINE __attribute__((noinline))
#else
#define FXP_BENCH_NOINLINE
#endif

namespace {
using Clock = std::chrono::steady_clock;
volatile std::uint64_t sink = 0;
// Keep completed output passes observable without volatile arithmetic inside
// the measured loop. A noinline kernel prevents folding repeated passes.
inline void observe(const void* memory) {
#if defined(_MSC_VER)
    (void)memory;
    _ReadWriteBarrier();
#elif defined(__GNUC__) || defined(__clang__)
    asm volatile("" : : "g"(memory) : "memory");
#else
    (void)memory;
    std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
}
struct Options {
    std::size_t size = 4096, samples = 7, minimum_ms = 10;
    std::string markdown, csv, cpu = "not recorded", affinity = "not pinned", command, recorded;
};
enum class Domain { signed_values, positive, unit, tangent, exponential, log1p, power };
const char* domain_name(Domain domain) {
    switch (domain) {
        case Domain::signed_values: return "a,b: [-8,8); b nonzero";
        case Domain::positive: return "a,b: [0.0625,8)";
        case Domain::unit: return "a: [-1,1]";
        case Domain::tangent: return "a: [-1.25,1.25]";
        case Domain::exponential: return "a: [-4,4]";
        case Domain::log1p: return "a: [-0.75,4]";
        case Domain::power: return "a: [0.5,2); b: (-2,2), noninteger";
    }
    return "unknown";
}
std::uint64_t random_word(std::uint64_t& state) {
    state ^= state << 13; state ^= state >> 7; state ^= state << 17;
    return state;
}
std::int64_t numerator(Domain domain, std::uint64_t word, bool second) {
    switch (domain) {
        case Domain::signed_values: {
            auto n = static_cast<std::int64_t>(word % 16384) - 8192;
            return second && n == 0 ? 1 : n;
        }
        case Domain::positive: return static_cast<std::int64_t>(word % 8128) + 64;
        case Domain::unit: return static_cast<std::int64_t>(word % 2049) - 1024;
        case Domain::tangent: return static_cast<std::int64_t>(word % 2561) - 1280;
        case Domain::exponential: return static_cast<std::int64_t>(word % 8193) - 4096;
        case Domain::log1p: return static_cast<std::int64_t>(word % 4865) - 768;
        case Domain::power: return second ? 2 * static_cast<std::int64_t>(word % 2048) - 2047
            : static_cast<std::int64_t>(word % 1536) + 512;
    }
    return 0;
}
template<class T> T input_value(std::int64_t n) {
    if constexpr (std::is_floating_point_v<T>) return static_cast<T>(n) / static_cast<T>(1024);
    else return T::from_raw(n * static_cast<std::int64_t>(T::scale / 1024));
}
template<class T> struct Data {
    std::vector<T> a, b;
    Data(std::size_t size, Domain domain) : a(size), b(size) {
        std::uint64_t state = UINT64_C(0x46585042454e4348);
        for (std::size_t i = 0; i < size; ++i) {
            a[i] = input_value<T>(numerator(domain, random_word(state), false));
            b[i] = input_value<T>(numerator(domain, random_word(state), true));
        }
    }
};
struct Copy { template<class T> T operator()(T a, T) const { return a; } };
struct Add { template<class T> T operator()(T a, T b) const { return a + b; } };
struct Subtract { template<class T> T operator()(T a, T b) const { return a - b; } };
struct Multiply { template<class T> T operator()(T a, T b) const { return a * b; } };
struct Divide { template<class T> T operator()(T a, T b) const { return a / b; } };
struct Less { template<class T> std::uint8_t operator()(T a, T b) const { return static_cast<std::uint8_t>(a < b); } };
struct BatchAdd : Add {};
struct BatchSubtract : Subtract {};
// ADL selects fxp for fixed point; floating point uses the std overload for its
// own type, including float overloads, without promoting float to double.
#define FXP_UNARY(NAME, FUNCTION) \
    struct NAME { template<class T> T operator()(T a, T) const { using std::FUNCTION; return FUNCTION(a); } };
#define FXP_BINARY(NAME, FUNCTION) \
    struct NAME { template<class T> T operator()(T a, T b) const { using std::FUNCTION; return FUNCTION(a, b); } };
FXP_UNARY(Abs, abs) FXP_UNARY(Floor, floor) FXP_UNARY(Ceil, ceil)
FXP_UNARY(Trunc, trunc) FXP_UNARY(Round, round) FXP_UNARY(Nearbyint, nearbyint)
FXP_UNARY(Sin, sin) FXP_UNARY(Cos, cos) FXP_UNARY(Tan, tan)
FXP_UNARY(Asin, asin) FXP_UNARY(Acos, acos) FXP_UNARY(Atan, atan)
FXP_UNARY(Exp, exp) FXP_UNARY(Exp2, exp2) FXP_UNARY(Expm1, expm1)
FXP_UNARY(Log, log) FXP_UNARY(Log2, log2) FXP_UNARY(Log10, log10) FXP_UNARY(Log1p, log1p)
FXP_UNARY(Sqrt, sqrt) FXP_UNARY(Cbrt, cbrt)
FXP_BINARY(Fmod, fmod) FXP_BINARY(Remainder, remainder) FXP_BINARY(Copysign, copysign)
FXP_BINARY(Atan2, atan2) FXP_BINARY(Pow, pow) FXP_BINARY(Hypot, hypot)
#undef FXP_UNARY
#undef FXP_BINARY

template<class T, class Operation>
FXP_BENCH_NOINLINE void kernel(const T* a, const T* b,
    std::invoke_result_t<Operation, T, T>* out, std::size_t count) {
    if constexpr (!std::is_floating_point_v<T> && std::is_same_v<Operation, BatchAdd>)
        fxp::batch::add(a, b, out, count);
    else if constexpr (!std::is_floating_point_v<T> && std::is_same_v<Operation, BatchSubtract>)
        fxp::batch::subtract(a, b, out, count);
    else for (std::size_t i = 0; i < count; ++i) out[i] = Operation{}(a[i], b[i]);
}
template<class T> std::uint64_t output_bits(T value) {
    if constexpr (std::is_floating_point_v<T>) {
        static_assert(sizeof(T) <= sizeof(std::uint64_t));
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(value));
        return bits;
    } else if constexpr (std::is_integral_v<T>) return value;
    else return static_cast<std::uint64_t>(value.raw_value());
}
template<class T, class Operation> struct Runner {
    using Output = std::invoke_result_t<Operation, T, T>;
    Data<T> data;
    std::vector<Output> output;
    Runner(std::size_t size, Domain domain) : data(size, domain), output(size) {}
    std::uint64_t time(std::size_t repetitions) {
        observe(data.a.data()); observe(data.b.data());
        const auto begin = Clock::now();
        for (std::size_t r = 0; r < repetitions; ++r) {
            kernel<T, Operation>(data.a.data(), data.b.data(), output.data(), output.size());
            observe(output.data());
        }
        return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - begin).count());
    }
    std::uint64_t checksum() const {
        std::uint64_t hash = UINT64_C(14695981039346656037);
        for (auto value : output) {
            if constexpr (std::is_floating_point_v<Output>) {
                if (!std::isfinite(value)) throw std::runtime_error("benchmark produced nonfinite output");
            }
            hash = (hash ^ output_bits(value)) * UINT64_C(1099511628211);
        }
        return hash;
    }
};
struct Measurement {
    std::function<std::uint64_t(std::size_t)> time;
    std::function<std::uint64_t()> checksum;
    std::size_t repetitions = 1;
    std::uint64_t expected_checksum = 0;
    std::vector<std::uint64_t> elapsed;
    double median(std::size_t size) const {
        auto sorted = elapsed;
        std::sort(sorted.begin(), sorted.end());
        return static_cast<double>(sorted[sorted.size() / 2]) /
            (static_cast<double>(repetitions) * static_cast<double>(size));
    }
};
struct Benchmark {
    std::string category, name;
    Domain domain;
    std::array<Measurement, 4> types;
};
constexpr std::array<const char*, 4> type_names{"real", "fine", "float", "double"};
template<class T, class Operation> Measurement measurement(const Options& options, Domain domain) {
    auto runner = std::make_shared<Runner<T, Operation>>(options.size, domain);
    return {[runner](std::size_t count) { return runner->time(count); }, [runner] { return runner->checksum(); }, 1, 0, {}};
}
template<class Operation>
void add(std::vector<Benchmark>& benchmarks, const Options& options, const char* category, const char* name, Domain domain) {
    benchmarks.push_back({category, name, domain, {
        measurement<fxp::real, Operation>(options, domain), measurement<fxp::fine, Operation>(options, domain),
        measurement<float, Operation>(options, domain), measurement<double, Operation>(options, domain)}});
}
std::vector<Benchmark> make_benchmarks(const Options& options) {
    std::vector<Benchmark> result;
#define BASIC(OP, NAME) add<OP>(result, options, "Basic operations", NAME, Domain::signed_values)
    BASIC(Copy, "copy (baseline)"); BASIC(Add, "+"); BASIC(Subtract, "-");
    BASIC(Multiply, "*"); BASIC(Divide, "/"); BASIC(Less, "< (byte output)");
    BASIC(BatchAdd, "batch add (automatic)"); BASIC(BatchSubtract, "batch subtract (automatic)");
#undef BASIC
#define MATH(OP, NAME, DOMAIN) add<OP>(result, options, "Math functions", NAME, Domain::DOMAIN)
    MATH(Abs, "abs", signed_values); MATH(Fmod, "fmod", signed_values);
    MATH(Remainder, "remainder", signed_values); MATH(Copysign, "copysign", signed_values);
    MATH(Floor, "floor", signed_values); MATH(Ceil, "ceil", signed_values);
    MATH(Trunc, "trunc", signed_values); MATH(Round, "round", signed_values); MATH(Nearbyint, "nearbyint", signed_values);
    MATH(Sin, "sin", signed_values); MATH(Cos, "cos", signed_values); MATH(Tan, "tan", tangent);
    MATH(Asin, "asin", unit); MATH(Acos, "acos", unit); MATH(Atan, "atan", signed_values); MATH(Atan2, "atan2", signed_values);
    MATH(Exp, "exp", exponential); MATH(Exp2, "exp2", exponential); MATH(Expm1, "expm1", exponential);
    MATH(Log, "log", positive); MATH(Log10, "log10", positive); MATH(Log2, "log2", positive); MATH(Log1p, "log1p", log1p);
    MATH(Pow, "pow (noninteger exponent)", power); MATH(Sqrt, "sqrt", positive);
    MATH(Cbrt, "cbrt", signed_values); MATH(Hypot, "hypot", signed_values);
#undef MATH
    return result;
}
void run(std::vector<Benchmark>& benchmarks, const Options& options) {
    const auto target_ns = static_cast<std::uint64_t>(options.minimum_ms) * UINT64_C(1000000);
    for (auto& benchmark : benchmarks) {
        std::cerr << "Measuring " << benchmark.name << "...\n";
        for (auto& type : benchmark.types) {
            type.time(1);
            while (type.time(type.repetitions) < target_ns) {
                if (type.repetitions >= (std::size_t{1} << 28)) throw std::runtime_error("benchmark calibration failed");
                type.repetitions *= 2;
            }
            type.expected_checksum = type.checksum();
        }
        for (std::size_t sample = 0; sample < options.samples; ++sample) {
            for (std::size_t offset = 0; offset < type_names.size(); ++offset) {
                auto& type = benchmark.types[(sample + offset) % type_names.size()];
                type.elapsed.push_back(type.time(type.repetitions));
                const auto hash = type.checksum();
                if (hash != type.expected_checksum) throw std::runtime_error("benchmark output changed between samples");
                sink = hash;
            }
        }
    }
    for (std::size_t type = 0; type < type_names.size(); ++type) {
        if (benchmarks[1].types[type].expected_checksum != benchmarks[6].types[type].expected_checksum ||
            benchmarks[2].types[type].expected_checksum != benchmarks[7].types[type].expected_checksum)
            throw std::runtime_error("batch output disagrees with ordinary operators");
    }
}
std::string markdown_text(std::string text) {
    for (auto& c : text) if (c == '\n' || c == '\r' || c == '|') c = ' ';
    return text;
}
void markdown(std::ostream& out, const std::vector<Benchmark>& benchmarks, const Options& options) {
    out << "# Fixed point versus floating point benchmarks\n\n"
        << "Measured array throughput in **nanoseconds per output value**; lower is faster. "
        << "Each table reports the median of " << options.samples << " samples.\n\n"
        << "| Setting | Value |\n| --- | --- |\n"
        << "| Run started (UTC) | " << options.recorded << " |\n"
        << "| CPU | " << markdown_text(options.cpu) << " |\n"
        << "| CPU affinity | " << markdown_text(options.affinity) << " |\n"
        << "| Platform | " << fxp_bench_config::system << " |\n"
        << "| Compiler | " << fxp_bench_config::compiler << " |\n"
        << "| Build configuration | " << FXP_BENCHMARK_CONFIG << " |\n"
        << "| Compiler flags | " << markdown_text(fxp_bench_config::flags) << " |\n"
        << "| real / fine / float / double size | " << sizeof(fxp::real) << " / " << sizeof(fxp::fine) << " / "
        << sizeof(float) << " / " << sizeof(double) << " bytes |\n"
        << "| Input count per pass | " << options.size << " |\n"
        << "| Minimum calibration time per sample | " << options.minimum_ms << " ms |\n"
        << "| Samples per type and operation | " << options.samples << " |\n"
        << "| Fixed wide arithmetic | "
#if defined(FXP_PORTABLE_ONLY)
        << "portable standard C++"
#else
        << "native where supported"
#endif
        << " |\n| Explicit SSE2 batch implementation | " << (fxp::batch::has_sse2 ? "enabled" : "disabled/unavailable") << " |\n"
        << "| Floating point rounding mode | nearest |\n";
#if defined(__GLIBC__)
    out << "| glibc headers | " << __GLIBC__ << '.' << __GLIBC_MINOR__ << " |\n";
#endif
#if defined(_GLIBCXX_RELEASE)
    out << "| libstdc++ headers | " << _GLIBCXX_RELEASE << " |\n";
#elif defined(_LIBCPP_VERSION)
    out << "| libc++ headers | " << _LIBCPP_VERSION << " |\n";
#elif defined(_MSVC_STL_VERSION)
    out << "| MSVC STL | " << _MSVC_STL_VERSION << " |\n";
#endif
    out << "\n## Method\n\n"
        << "Inputs are deterministic pseudorandom multiples of 1/1024, exactly representable "
        << "by all four types. Setup, conversion, allocation, and checksum calculation are "
        << "outside the timed regions. Each sample repeats full array passes, with repetition "
        << "counts calibrated separately for each implementation. Type order rotates between "
        << "samples. Output barriers and a non-inlined kernel prevent elimination of repeated "
        << "work; output checksums must remain stable.\n\n"
        << "Ordinary operators and math functions are called per element. Floating point uses "
        << "the std overload for the measured type, including float overloads. The explicit "
        << "batch rows use the library's automatic backend for real/fine; float/double use "
        << "ordinary array loops. The compiler may auto-vectorize any loop. Comparisons "
        << "write one byte per element for all four types. Other outputs have the measured "
        << "type's size, so float uses half the output/input bandwidth of the 64-bit types.\n\n"
        << "The inputs avoid division by zero, invalid math domains, overflow, floating point underflow, "
        << "subnormal floating point, and tangent poles. Fixed-point results still undergo normal quantization. These numbers describe ordinary "
        << "finite inputs, not worst-case timings, saturated arithmetic, very large trig "
        << "arguments, or dependency-chain latency. The copy row provides a loop/memory baseline.\n\n"
        << "Fixed point includes its saturation and deterministic rounding rules. Floating "
        << "point has different range, precision, exception, and cross-platform math behavior. "
        << "This is a speed comparison, not a claim of equivalent numerical results. "
        << "Transcendental accuracy is documented separately in numerics.md.\n\n"
        << "Ratios real/float and fine/double compare the median costs: above 1 means fixed "
        << "point took longer. Scheduling, turbo, cache state, and libm implementation affect "
        << "these measurements; sub-nanosecond values reflect vectorized/amortized throughput.\n";
    std::string category;
    out << std::fixed << std::setprecision(3);
    for (const auto& benchmark : benchmarks) {
        if (category != benchmark.category) {
            category = benchmark.category;
            out << "\n## " << category << "\n\n"
                << "| Operation | real ns | fine ns | float ns | double ns | real/float | fine/double |\n"
                << "| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n";
        }
        std::array<double, 4> medians{};
        out << "| " << benchmark.name << " |";
        for (std::size_t i = 0; i < medians.size(); ++i) {
            medians[i] = benchmark.types[i].median(options.size);
            out << ' ' << medians[i] << " |";
        }
        out << ' ' << medians[0] / medians[2] << "x | " << medians[1] / medians[3] << "x |\n";
    }
    struct Variation { const Benchmark* benchmark; std::size_t type; double minimum, maximum, ratio; };
    std::vector<Variation> variations;
    for (const auto& benchmark : benchmarks) for (std::size_t t = 0; t < type_names.size(); ++t) {
        const auto& type = benchmark.types[t];
        const auto extremes = std::minmax_element(type.elapsed.begin(), type.elapsed.end());
        const double count = static_cast<double>(type.repetitions) * static_cast<double>(options.size);
        const auto minimum = static_cast<double>(*extremes.first) / count;
        const auto maximum = static_cast<double>(*extremes.second) / count;
        variations.push_back({&benchmark, t, minimum, maximum, maximum / minimum});
    }
    std::sort(variations.begin(), variations.end(), [](const Variation& a, const Variation& b) { return a.ratio > b.ratio; });
    out << "\n## Sample variability\n\n"
        << "The five largest within-case max/min ratios are shown below. These are observed "
        << "sample ranges, not confidence intervals. All samples remain in the CSV; none are "
        << "discarded. Pinning prevents CPU migration but does not remove scheduling or clock-frequency variation.\n\n"
        << "| Operation | Type | Minimum ns | Median ns | Maximum ns | Max/min |\n"
        << "| --- | --- | ---: | ---: | ---: | ---: |\n";
    for (std::size_t i = 0; i < std::min<std::size_t>(5, variations.size()); ++i) {
        const auto& v = variations[i];
        out << "| " << v.benchmark->name << " | " << type_names[v.type] << " | " << v.minimum << " | "
            << v.benchmark->types[v.type].median(options.size) << " | " << v.maximum << " | " << v.ratio << "x |\n";
    }
    out << "\n## Input domains\n\n| Operation | Input domain |\n| --- | --- |\n";
    for (const auto& benchmark : benchmarks) out << "| " << benchmark.name << " | " << domain_name(benchmark.domain) << " |\n";
    out << "\n## Reproduction\n\n"
        << "Build the fxp_benchmark target with FXP_BUILD_BENCHMARKS=ON and a Release "
        << "configuration. Run --help for options. No fast-math or architecture-specific "
        << "optimization flags are added by the benchmark target; the compiler flags above "
        << "record the configured flags. Pinning is performed externally (for example with "
        << "taskset on Linux); --cpu and --affinity are labels supplied by the caller.\n\n"
        << "```text\n" << options.command << "\n```\n";
    if (!options.csv.empty()) out << "\nRaw sample timings and output checksums: " << markdown_text(options.csv) << ".\n";
}
void csv(std::ostream& out, const std::vector<Benchmark>& benchmarks, const Options& options) {
    out << "operation,type,sample,repetitions,elements,elapsed_ns,ns_per_value,checksum\n" << std::fixed << std::setprecision(9);
    for (const auto& benchmark : benchmarks) for (std::size_t t = 0; t < type_names.size(); ++t) {
        const auto& type = benchmark.types[t];
        for (std::size_t sample = 0; sample < type.elapsed.size(); ++sample)
            out << benchmark.name << ',' << type_names[t] << ',' << sample + 1 << ',' << type.repetitions << ',' << options.size << ','
                << type.elapsed[sample] << ',' << static_cast<double>(type.elapsed[sample]) /
                (static_cast<double>(type.repetitions) * static_cast<double>(options.size)) << ',' << type.expected_checksum << '\n';
    }
}
std::size_t number(const std::string& text) {
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("expected a positive integer: " + text);
    const auto n = std::stoull(text);
    if (!n || n > 1000000) throw std::invalid_argument("number must be in [1,1000000]");
    return static_cast<std::size_t>(n);
}
} // namespace

int main(int argc, char** argv) {
    try {
        Options options;
        for (int i = 0; i < argc; ++i) {
            if (i) options.command += ' ';
            options.command += '"' + std::string(argv[i]) + '"';
        }
        for (int i = 1; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--help") {
                std::cout << "usage: fxp_benchmark [--size 4096] [--samples 7] [--min-ms 10]\n"
                    << "                     [--markdown report.md] [--csv samples.csv]\n"
                    << "                     [--cpu description] [--affinity description]\n"
                    << "Samples must be odd. CPU/affinity are report labels, not system settings.\n";
                return 0;
            }
            if (++i == argc) throw std::invalid_argument("missing argument for " + option);
            const std::string value = argv[i];
            if (option == "--size") options.size = number(value);
            else if (option == "--samples") options.samples = number(value);
            else if (option == "--min-ms") options.minimum_ms = number(value);
            else if (option == "--markdown") options.markdown = value;
            else if (option == "--csv") options.csv = value;
            else if (option == "--cpu") options.cpu = value;
            else if (option == "--affinity") options.affinity = value;
            else throw std::invalid_argument("unknown option: " + option);
        }
        if (options.samples % 2 == 0) throw std::invalid_argument("--samples must be odd for an unambiguous median");
        if (!options.markdown.empty() && options.markdown == options.csv)
            throw std::invalid_argument("Markdown and CSV outputs must be different files");
        if (std::fesetround(FE_TONEAREST) != 0) throw std::runtime_error("cannot select nearest rounding mode");
        const auto started = std::time(nullptr);
        if (const auto* utc = std::gmtime(&started)) {
            std::array<char, 32> text{};
            if (std::strftime(text.data(), text.size(), "%Y-%m-%d %H:%M:%S", utc)) options.recorded = text.data();
        }
        std::ofstream markdown_file, csv_file;
        if (!options.markdown.empty()) {
            markdown_file.open(options.markdown);
            if (!markdown_file) throw std::runtime_error("cannot open Markdown output");
        }
        if (!options.csv.empty()) {
            csv_file.open(options.csv);
            if (!csv_file) throw std::runtime_error("cannot open CSV output");
        }
        auto benchmarks = make_benchmarks(options);
        run(benchmarks, options);
        markdown(std::cout, benchmarks, options);
        if (markdown_file.is_open()) { markdown(markdown_file, benchmarks, options); markdown_file.flush();
            if (!markdown_file) throw std::runtime_error("could not write Markdown report"); }
        if (csv_file.is_open()) { csv(csv_file, benchmarks, options); csv_file.flush();
            if (!csv_file) throw std::runtime_error("could not write CSV report"); }
    } catch (const std::exception& error) { std::cerr << "benchmark: " << error.what() << '\n'; return 1; }
}
