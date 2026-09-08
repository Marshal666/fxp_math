#pragma once
#include "wide.hpp"
#include <array>

namespace fxp::detail {
// All constants are rounded from high precision decimal by tools/generate_constants.py.
constexpr std::int64_t one = INT64_C(1152921504606846976); // Q4.60
constexpr std::int64_t pi = INT64_C(3622009729038561421);
constexpr std::int64_t half_pi = INT64_C(1811004864519280711);
constexpr std::int64_t quarter_pi = INT64_C(905502432259640355);
constexpr std::int64_t ln2 = INT64_C(799144290325165979);
constexpr std::int64_t ln10 = INT64_C(2654699869899991814);
constexpr std::int64_t tan_pi_8 = INT64_C(477555723559750801);
// pi/2 in Q128, including enough guard bits to preserve phase near poles.
constexpr u192 half_pi_q128{1, {UINT64_C(10529333758598939753), UINT64_C(9911513582539389346)}};

inline std::int64_t mul60(std::int64_t a, std::int64_t b) { return mul_shift(a, b, 60); }
inline std::int64_t ratio60(std::uint64_t a, std::uint64_t b) {
    return signed_magnitude(round_div(shl({0, a}, 60), b), false);
}
constexpr std::int64_t reciprocal(std::uint64_t denominator) {
    const auto q = static_cast<std::uint64_t>(one) / denominator;
    const auto r = static_cast<std::uint64_t>(one) % denominator;
    return static_cast<std::int64_t>(q + (r > denominator - r || (r == denominator - r && (q & 1))));
}
constexpr auto factorial_coefficients() {
    std::array<std::int64_t, 19> result{};
    std::uint64_t factorial = 1;
    for (std::size_t n = 0; n < result.size(); ++n) {
        if (n) factorial *= n;
        result[n] = reciprocal(factorial);
    }
    return result;
}
constexpr auto odd_coefficients() {
    std::array<std::int64_t, 25> result{};
    for (std::size_t n = 0; n < result.size(); ++n) result[n] = reciprocal(2 * n + 1);
    return result;
}
inline constexpr auto factorial_inv = factorial_coefficients();
inline constexpr auto odd_inv = odd_coefficients();
inline std::int64_t atan_unit(std::int64_t x) {
    bool shifted = x > tan_pi_8;
    if (shifted) x = div_shift(x - one, x + one, 60);
    const auto square = mul60(x, x);
    auto polynomial = odd_inv[24];
    for (std::size_t n = 24; n != 0; --n) polynomial = odd_inv[n - 1] - mul60(polynomial, square);
    const auto sum = mul60(x, polynomial);
    return shifted ? quarter_pi + sum : sum;
}
inline std::int64_t atan2_kernel(std::uint64_t y, bool y_negative, std::uint64_t x, bool x_negative) {
    if (!x && !y) throw std::domain_error("fixed point atan2(0, 0)");
    auto angle = y <= x ? atan_unit(ratio60(y, x)) : half_pi - atan_unit(ratio60(x, y));
    if (x_negative) angle = pi - angle;
    return y_negative ? -angle : angle;
}
struct sincos_pair { std::int64_t sine, cosine; };
inline division192 reduce_angle(std::int64_t raw, unsigned fractional_bits) {
    return div192(shl192({0, {0, magnitude(raw)}}, 128 - fractional_bits), half_pi_q128);
}
inline std::int64_t q128_to_q60(u192 value) {
    auto q = shr192(value, 68).low.lo;
    const bool half = (value.low.hi & 8) != 0;
    const bool sticky = (value.low.hi & 7) != 0 || value.low.lo != 0;
    if (half && (sticky || (q & 1))) ++q;
    return static_cast<std::int64_t>(q);
}
inline sincos_pair sincos_reduced(division192 reduced, bool negative) {
    auto r = q128_to_q60(reduced.remainder);
    const bool complement = r > quarter_pi;
    if (complement) r = half_pi - r;
    const auto square = mul60(r, r);
    auto sp = factorial_inv[17], cp = -factorial_inv[18];
    for (std::size_t n = 8; n > 0; --n) {
        const auto coefficient = factorial_inv[2 * n - 1];
        sp = (n % 2 ? coefficient : -coefficient) + mul60(sp, square);
    }
    for (std::size_t n = 9; n > 0; --n) {
        const auto coefficient = factorial_inv[2 * n - 2];
        cp = (n % 2 ? coefficient : -coefficient) + mul60(cp, square);
    }
    auto s = mul60(r, sp), c = cp;
    if (complement) { const auto temporary = s; s = c; c = temporary; }
    sincos_pair out{};
    switch (reduced.quotient.low.lo & 3) {
        case 0: out = {s, c}; break;
        case 1: out = {c, -s}; break;
        case 2: out = {-s, -c}; break;
        default: out = {-c, s}; break;
    }
    if (negative) out.sine = -out.sine;
    return out;
}
inline sincos_pair sincos_kernel(std::int64_t raw, unsigned fractional_bits) {
    return sincos_reduced(reduce_angle(raw, fractional_bits), raw < 0);
}
inline std::int64_t tan_kernel(std::int64_t raw, unsigned fractional_bits) {
    const auto reduced = reduce_angle(raw, fractional_bits);
    const bool odd = (reduced.quotient.low.lo & 1) != 0;
    const auto delta = odd ? reduced.remainder : sub192(half_pi_q128, reduced.remainder);
    // cot(delta) = 1/delta - delta/3 - delta^3/45 - 2*delta^5/945 - delta^7/4725.
    // Keep the reciprocal in a wide Q60 accumulator near a pole; converting its
    // tiny denominator to Q60 first could erase its sign or significant bits.
    if (less(delta, shl192({0, {0, 1}}, 120))) {
        const bool negative = odd != (raw < 0);
        if (equal(delta, {})) return signed_magnitude({1, 0}, negative);
        const auto inverse = div192(shl192({0, {0, 1}}, 188), delta).quotient;
        if (inverse.hi) return signed_magnitude({1, 0}, negative);
        const auto d = q128_to_q60(delta);
        const auto d2 = mul60(d, d);
        const auto correction = mul60(d, reciprocal(3) + mul60(d2, reciprocal(45) +
            mul60(d2, 2 * reciprocal(945) + mul60(d2, reciprocal(4725)))));
        const auto mag = sub(inverse.low, {0, static_cast<std::uint64_t>(correction)});
        return signed_magnitude(round_shift(mag, 60 - fractional_bits), negative);
    }
    const auto result = sincos_reduced(reduced, raw < 0);
    return div_shift(result.sine, result.cosine, fractional_bits);
}
struct signed_wide { u128 mag; bool negative; };
inline signed_wide log_kernel(std::uint64_t raw, unsigned fractional_bits) {
    if (!raw) throw std::domain_error("fixed point logarithm requires a positive argument");
    const auto top = bit_width(raw) - 1;
    const auto exponent = static_cast<int>(top) - static_cast<int>(fractional_bits);
    const auto mantissa = static_cast<std::int64_t>(top > 60 ? round_shift({0, raw}, top - 60).lo : raw << (60 - top));
    const auto z = div_shift(mantissa - one, mantissa + one, 60);
    const auto square = mul60(z, z);
    auto polynomial = odd_inv[21];
    for (std::size_t n = 21; n != 0; --n) polynomial = odd_inv[n - 1] + mul60(polynomial, square);
    const auto fractional = static_cast<std::uint64_t>(mul60(z, polynomial) * 2);
    const auto integral = mul(static_cast<std::uint64_t>(exponent < 0 ? -exponent : exponent), ln2);
    return exponent < 0 ? signed_wide{sub(integral, {0, fractional}), true}
        : signed_wide{add(integral, {0, fractional}), false};
}
inline std::int64_t exp_series(std::int64_t reduced) {
    auto polynomial = factorial_inv[18];
    for (std::size_t n = 18; n != 0; --n) polynomial = factorial_inv[n - 1] + mul60(polynomial, reduced);
    return polynomial;
}
inline std::int64_t exp_kernel(signed_wide x, unsigned fractional_bits, bool minus_one = false) {
    if (x.mag >= shl({0, 64}, 60)) {
        return x.negative ? (minus_one ? -static_cast<std::int64_t>(UINT64_C(1) << fractional_bits) : 0)
            : std::numeric_limits<std::int64_t>::max();
    }
    const auto d = div(x.mag, ln2);
    auto k = static_cast<int>(d.quotient.lo);
    auto residual = static_cast<std::int64_t>(d.remainder.lo);
    if (residual > ln2 / 2) { ++k; residual -= ln2; }
    if (x.negative) { k = -k; residual = -residual; }
    if (k + static_cast<int>(fractional_bits) >= 64) return std::numeric_limits<std::int64_t>::max();
    const auto series = static_cast<std::uint64_t>(exp_series(residual));
    if (!minus_one) {
        const auto shift = k + static_cast<int>(fractional_bits) - 60;
        const auto result = shift >= 0 ? shl({0, series}, static_cast<unsigned>(shift))
            : round_shift({0, series}, static_cast<unsigned>(-shift));
        return signed_magnitude(result, false);
    }
    // Keep cancellation and the subtraction inside the wider accumulator.
    const auto positive = k >= 0 ? shl({0, series}, static_cast<unsigned>(k))
        : round_shift({0, series}, static_cast<unsigned>(-k));
    const bool negative = positive < u128{0, static_cast<std::uint64_t>(one)};
    const auto mag = negative ? sub({0, static_cast<std::uint64_t>(one)}, positive)
        : sub(positive, {0, static_cast<std::uint64_t>(one)});
    return signed_magnitude(round_shift(mag, 60 - fractional_bits), negative);
}
inline u192 mul_wide(u128 a, std::uint64_t b) {
    const auto low = mul(a.lo, b), high = mul(a.hi, b);
    const auto mid = low.hi + high.lo;
    return {high.hi + (mid < low.hi), {mid, low.lo}};
}
inline u192 cube(std::uint64_t x) { return mul_wide(mul(x, x), x); }
inline std::uint64_t rounded_cbrt(u128 n) {
    std::uint64_t lo = 0, hi = UINT64_C(1) << ((bit_width(n) + 2) / 3);
    while (lo + 1 < hi) {
        const auto mid = lo + (hi - lo) / 2;
        if (less({0, n}, cube(mid))) hi = mid;
        else lo = mid;
    }
    const u192 eight_n{n.hi >> 61, shl(n, 3)};
    if (less(cube(2 * lo + 1), eight_n)) ++lo;
    return lo;
}
} // namespace fxp::detail
