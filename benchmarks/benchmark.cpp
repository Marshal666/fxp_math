#include <fxp/fxp.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace {
volatile std::uint64_t sink = 0;
inline void observe(const void* memory) {
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : : "g"(memory) : "memory");
#elif defined(_MSC_VER)
    (void)memory;
    _ReadWriteBarrier();
#else
    sink = reinterpret_cast<std::uintptr_t>(memory);
#endif
}
template<class T, class Function>
void measure(const char* name, std::vector<T>& output, std::size_t repetitions, Function function) {
    function();
    const auto begin = std::chrono::steady_clock::now();
    for (std::size_t r = 0; r < repetitions; ++r) { function(); observe(output.data()); }
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begin).count();
    const auto operations = output.size() * repetitions;
    const auto picoseconds = static_cast<std::uint64_t>(ns) * 1000 / operations;
    std::uint64_t checksum = 0;
    for (auto x : output) checksum = (checksum * 33) ^ static_cast<std::uint64_t>(x.raw_value());
    sink = checksum;
    std::cout << std::setw(16) << name << ' ' << picoseconds / 1000 << '.' << std::setw(3)
              << std::setfill('0') << picoseconds % 1000 << std::setfill(' ') << " ns/value; checksum=" << checksum << '\n';
}
template<class T> void run(const char* format) {
    std::cout << format << " (SSE2 available: " << fxp::batch::has_sse2 << ")\n";
    std::vector<T> a(4096), b(4096), out(4096);
    for (std::size_t i = 0; i < a.size(); ++i) {
        a[i] = T::from_raw(static_cast<std::int64_t>((i + 1) * T::scale / 1024));
        b[i] = T::from_raw(static_cast<std::int64_t>((i + 511) * T::scale / 2048));
    }
    measure("add scalar", out, 2000, [&] { fxp::batch::add(a.data(), b.data(), out.data(), out.size(), fxp::batch::backend::scalar); });
    measure("add automatic", out, 2000, [&] { fxp::batch::add(a.data(), b.data(), out.data(), out.size()); });
    measure("subtract scalar", out, 2000, [&] { fxp::batch::subtract(a.data(), b.data(), out.data(), out.size(), fxp::batch::backend::scalar); });
    measure("subtract auto", out, 2000, [&] { fxp::batch::subtract(a.data(), b.data(), out.data(), out.size()); });
    measure("multiply", out, 500, [&] { fxp::batch::multiply(a.data(), b.data(), out.data(), out.size()); });
    measure("divide", out, 500, [&] { fxp::batch::divide(a.data(), b.data(), out.data(), out.size()); });
    measure("sin", out, 80, [&] { for (std::size_t i = 0; i < out.size(); ++i) out[i] = fxp::sin(a[i]); });
    measure("sqrt", out, 80, [&] { for (std::size_t i = 0; i < out.size(); ++i) out[i] = fxp::sqrt(a[i]); });
    measure("exp", out, 80, [&] { for (std::size_t i = 0; i < out.size(); ++i) out[i] = fxp::exp(a[i]); });
    measure("log", out, 80, [&] { for (std::size_t i = 0; i < out.size(); ++i) out[i] = fxp::log(a[i]); });
}
}
int main() { run<fxp::real>("Q48.16"); run<fxp::fine>("Q32.32"); }
