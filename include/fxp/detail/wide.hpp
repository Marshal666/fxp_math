#pragma once

#include <cstdint>
#include <limits>
#include <stdexcept>

#if defined(_MSC_VER) && defined(_M_X64) && !defined(FXP_PORTABLE_ONLY)
#include <intrin.h>
#endif

namespace fxp::detail {

// Unsigned two-word arithmetic is the portable definition of every wide operation.
struct u128 { std::uint64_t hi = 0, lo = 0; };
constexpr bool operator==(u128 a, u128 b) { return a.hi == b.hi && a.lo == b.lo; }
constexpr bool operator!=(u128 a, u128 b) { return !(a == b); }
constexpr bool operator<(u128 a, u128 b) {
    return a.hi < b.hi || (a.hi == b.hi && a.lo < b.lo);
}
constexpr bool operator>=(u128 a, u128 b) { return !(a < b); }
constexpr u128 add(u128 a, u128 b) {
    const auto lo = a.lo + b.lo;
    return {a.hi + b.hi + (lo < a.lo), lo};
}
constexpr u128 sub(u128 a, u128 b) {
    return {a.hi - b.hi - (a.lo < b.lo), a.lo - b.lo};
}
constexpr u128 shl(u128 a, unsigned n) {
    return n == 0 ? a : n >= 128 ? u128{} : n >= 64 ? u128{a.lo << (n - 64), 0}
        : u128{(a.hi << n) | (a.lo >> (64 - n)), a.lo << n};
}
constexpr u128 shr(u128 a, unsigned n) {
    return n == 0 ? a : n >= 128 ? u128{} : n >= 64 ? u128{0, a.hi >> (n - 64)}
        : u128{a.hi >> n, (a.lo >> n) | (a.hi << (64 - n))};
}
constexpr unsigned bit_width(std::uint64_t x) {
    unsigned n = 0;
    while (x) { ++n; x >>= 1; }
    return n;
}
constexpr unsigned bit_width(u128 x) { return x.hi ? 64 + bit_width(x.hi) : bit_width(x.lo); }

struct u192 { std::uint64_t hi = 0; u128 low{}; };
constexpr bool equal(u192 a, u192 b) { return a.hi == b.hi && a.low == b.low; }
constexpr bool less(u192 a, u192 b) { return a.hi < b.hi || (a.hi == b.hi && a.low < b.low); }
constexpr unsigned bit_width192(u192 x) { return x.hi ? 128 + bit_width(x.hi) : bit_width(x.low); }
constexpr u192 sub192(u192 a, u192 b) { return {a.hi - b.hi - (a.low < b.low), sub(a.low, b.low)}; }
constexpr u192 shl192(u192 x, unsigned n) {
    if (!n) return x;
    if (n >= 192) return {};
    if (n >= 128) return {x.low.lo << (n - 128), {}};
    if (n >= 64) return shl192(u192{x.low.hi, {x.low.lo, 0}}, n - 64);
    return {(x.hi << n) | (x.low.hi >> (64 - n)), shl(x.low, n)};
}
constexpr u192 shr192(u192 x, unsigned n) {
    if (!n) return x;
    if (n >= 192) return {};
    if (n >= 128) return {0, {0, x.hi >> (n - 128)}};
    if (n >= 64) return shr192(u192{0, {x.hi, x.low.hi}}, n - 64);
    const auto low = shr(x.low, n);
    return {x.hi >> n, {low.hi | (x.hi << (64 - n)), low.lo}};
}
struct division192 { u192 quotient, remainder; };
constexpr division192 div192(u192 n, u192 d) {
    if (equal(d, {})) throw std::domain_error("fixed point division by zero");
    if (less(n, d)) return {{}, n};
    const auto shift = bit_width192(n) - bit_width192(d);
    auto divisor = shl192(d, shift);
    u192 quotient{};
    for (unsigned i = shift + 1; i != 0; --i) {
        if (!less(n, divisor)) {
            n = sub192(n, divisor);
            const auto bit = shl192(u192{0, {0, 1}}, i - 1);
            quotient.hi |= bit.hi; quotient.low.hi |= bit.low.hi; quotient.low.lo |= bit.low.lo;
        }
        divisor = shr192(divisor, 1);
    }
    return {quotient, n};
}

constexpr u128 mul_portable(std::uint64_t a, std::uint64_t b) {
    constexpr std::uint64_t mask = UINT64_C(0xffffffff);
    const auto a0 = a & mask, a1 = a >> 32, b0 = b & mask, b1 = b >> 32;
    const auto w0 = a0 * b0;
    const auto t = a1 * b0 + (w0 >> 32);
    auto w1 = t & mask;
    const auto w2 = t >> 32;
    w1 += a0 * b1;
    return {a1 * b1 + w2 + (w1 >> 32), (w1 << 32) | (w0 & mask)};
}

inline u128 mul(std::uint64_t a, std::uint64_t b) {
#if defined(__SIZEOF_INT128__) && !defined(FXP_PORTABLE_ONLY)
    __extension__ using native_u128 = unsigned __int128;
    const native_u128 p = static_cast<native_u128>(a) * b;
    return {static_cast<std::uint64_t>(p >> 64), static_cast<std::uint64_t>(p)};
#elif defined(_MSC_VER) && defined(_M_X64) && !defined(FXP_PORTABLE_ONLY)
    unsigned __int64 high = 0; // Required by the MSVC intrinsic signature.
    const auto low = _umul128(a, b, &high);
    return {high, low};
#else
    return mul_portable(a, b);
#endif
}

struct division { u128 quotient, remainder; };
constexpr division div_portable(u128 n, u128 d) {
    if (d == u128{}) throw std::domain_error("fixed point division by zero");
    if (n < d) return {{}, n};
    const unsigned shift = bit_width(n) - bit_width(d);
    auto divisor = shl(d, shift);
    u128 q{};
    for (unsigned i = shift + 1; i != 0; --i) {
        if (n >= divisor) {
            n = sub(n, divisor);
            q = add(q, shl({0, 1}, i - 1));
        }
        divisor = shr(divisor, 1);
    }
    return {q, n};
}
inline division div(u128 n, std::uint64_t d) {
    if (!d) throw std::domain_error("fixed point division by zero");
    if (!n.hi) return {{0, n.lo / d}, {0, n.lo % d}};
#if defined(__SIZEOF_INT128__) && !defined(FXP_PORTABLE_ONLY)
    __extension__ using native_u128 = unsigned __int128;
    const native_u128 value = (static_cast<native_u128>(n.hi) << 64) | n.lo;
    const auto q = value / d;
    return {{static_cast<std::uint64_t>(q >> 64), static_cast<std::uint64_t>(q)},
            {0, static_cast<std::uint64_t>(value % d)}};
#elif defined(_MSC_VER) && defined(_M_X64) && !defined(FXP_PORTABLE_ONLY)
    unsigned __int64 remainder = 0;
    const auto qhi = n.hi / d;
    const auto qlo = _udiv128(n.hi % d, n.lo, d, &remainder);
    return {{qhi, qlo}, {0, remainder}};
#else
    return div_portable(n, {0, d});
#endif
}

constexpr std::uint64_t magnitude(std::int64_t x) {
    return x < 0 ? std::uint64_t{0} - static_cast<std::uint64_t>(x) : static_cast<std::uint64_t>(x);
}
constexpr std::int64_t signed_magnitude(u128 x, bool negative) {
    constexpr auto max = std::numeric_limits<std::int64_t>::max();
    constexpr auto min = std::numeric_limits<std::int64_t>::min();
    const auto limit = static_cast<std::uint64_t>(max) + static_cast<std::uint64_t>(negative);
    if (x.hi || x.lo >= limit) return negative ? min : max;
    return negative ? -static_cast<std::int64_t>(x.lo) : static_cast<std::int64_t>(x.lo);
}
constexpr std::int64_t sat_add(std::int64_t a, std::int64_t b) {
    constexpr auto max = std::numeric_limits<std::int64_t>::max();
    constexpr auto min = std::numeric_limits<std::int64_t>::min();
    if (b > 0 && a > max - b) return max;
    if (b < 0 && a < min - b) return min;
    return a + b;
}
constexpr std::int64_t sat_sub(std::int64_t a, std::int64_t b) {
    constexpr auto max = std::numeric_limits<std::int64_t>::max();
    constexpr auto min = std::numeric_limits<std::int64_t>::min();
    if (b < 0 && a > max + b) return max;
    if (b > 0 && a < min + b) return min;
    return a - b;
}
constexpr u128 round_shift(u128 x, unsigned n) {
    if (!n) return x;
    if (n > 128) return {};
    const auto q = shr(x, n);
    const auto r = sub(x, shl(q, n));
    const auto half = shl({0, 1}, n - 1);
    return add(q, {0, static_cast<std::uint64_t>(r >= half && (r != half || (q.lo & 1)))});
}
inline u128 round_div(u128 n, std::uint64_t d) {
    const auto v = div(n, d);
    const auto r = v.remainder.lo;
    return add(v.quotient, {0, static_cast<std::uint64_t>(r > d - r ||
        (r == d - r && (v.quotient.lo & 1)))});
}
inline std::int64_t mul_shift(std::int64_t a, std::int64_t b, unsigned shift) {
    return signed_magnitude(round_shift(mul(magnitude(a), magnitude(b)), shift), (a < 0) != (b < 0));
}
inline std::int64_t div_shift(std::int64_t a, std::int64_t b, unsigned shift) {
    return signed_magnitude(round_div(shl({0, magnitude(a)}, shift), magnitude(b)), (a < 0) != (b < 0));
}

// Restoring square root, returning floor(sqrt(n)). No floating point seed.
inline std::uint64_t isqrt(u128 n) {
    u128 result{};
    auto bit = shl({0, 1}, 126);
    while (bit != u128{} && n < bit) bit = shr(bit, 2);
    while (bit != u128{}) {
        const auto trial = add(result, bit);
        if (n >= trial) { n = sub(n, trial); result = add(shr(result, 1), bit); }
        else result = shr(result, 1);
        bit = shr(bit, 2);
    }
    return result.lo;
}
inline std::uint64_t rounded_sqrt(u128 n) {
    auto r = isqrt(n);
    // n - r*r > r is exactly the nearest-integer decision for integer n.
    if (sub(n, mul(r, r)) >= u128{0, r + 1}) ++r;
    return r;
}
} // namespace fxp::detail
