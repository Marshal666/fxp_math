#pragma once
#include "fixed.hpp"
#include "detail/kernels.hpp"
#include <utility>

namespace fxp {
template<unsigned F> constexpr fixed<F> abs(fixed<F> x) noexcept { return x < 0 ? -x : x; }
template<unsigned F> constexpr fixed<F> fabs(fixed<F> x) noexcept { return abs(x); }
template<unsigned F> constexpr bool signbit(fixed<F> x) noexcept { return x.raw_value() < 0; }
template<unsigned F> constexpr bool isfinite(fixed<F>) noexcept { return true; }
template<unsigned F> constexpr bool isnan(fixed<F>) noexcept { return false; }
template<unsigned F> constexpr bool isinf(fixed<F>) noexcept { return false; }
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
constexpr auto copysign(A a, B b) noexcept {
    using R = detail::promoted<A, B>;
    return R::from_raw(detail::signed_magnitude({0, detail::magnitude(R(a).raw_value())}, R(b).raw_value() < 0));
}
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
auto fmod(A a, B b) { return a % b; }
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
auto remquo(A a, B b, int* quotient) {
    using R = detail::promoted<A, B>;
    if (!quotient) throw std::invalid_argument("fixed point remquo needs a quotient pointer");
    const auto x = R(a).raw_value(), y = R(b).raw_value();
    const auto ax = detail::magnitude(x), ay = detail::magnitude(y);
    if (!ay) throw std::domain_error("fixed point remainder by zero");
    const auto q = ax / ay, r = ax % ay;
    const bool up = r > ay - r || (r == ay - r && (q & 1));
    const auto low = static_cast<int>((q + static_cast<std::uint64_t>(up)) & 127);
    *quotient = (x < 0) != (y < 0) ? -low : low;
    return R::from_raw(detail::signed_magnitude({0, up ? ay - r : r}, (x < 0) != up));
}
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
auto remainder(A a, B b) { int quotient = 0; return remquo(a, b, &quotient); }
template<unsigned F> constexpr fixed<F> trunc(fixed<F> x) noexcept {
    return fixed<F>::from_raw(detail::signed_magnitude({0, (detail::magnitude(x.raw_value()) >> F) << F}, x < 0));
}
template<unsigned F> constexpr fixed<F> floor(fixed<F> x) noexcept {
    const auto t = trunc(x);
    return x < t ? t - 1 : t;
}
template<unsigned F> constexpr fixed<F> ceil(fixed<F> x) noexcept {
    const auto t = trunc(x);
    return x > t ? t + 1 : t;
}
template<unsigned F> constexpr fixed<F> nearbyint(fixed<F> x) noexcept { return fixed<F>(x.template to_integer<>()); }
template<unsigned F> constexpr fixed<F> rint(fixed<F> x) noexcept { return nearbyint(x); }
// Like std::round, this named operation rounds halfway cases away from zero.
template<unsigned F> constexpr fixed<F> round(fixed<F> x) noexcept {
    const auto mag = detail::magnitude(x.raw_value());
    return fixed<F>::from_raw(detail::signed_magnitude({0, ((mag + fixed<F>::scale / 2) >> F) << F}, x < 0));
}
template<unsigned F> fixed<F> modf(fixed<F> x, fixed<F>* integral) {
    if (!integral) throw std::invalid_argument("fixed point modf needs an integral pointer");
    *integral = trunc(x); return x - *integral;
}
template<unsigned F> constexpr fixed<F> ldexp(fixed<F> x, int exponent) noexcept {
    const detail::u128 mag{0, detail::magnitude(x.raw_value())};
    if (exponent >= 0) {
        if (exponent >= 64) return x == 0 ? x : x < 0 ? fixed<F>::min() : fixed<F>::max();
        return fixed<F>::from_raw(detail::signed_magnitude(detail::shl(mag, static_cast<unsigned>(exponent)), x < 0));
    }
    if (exponent <= -128) return {};
    return fixed<F>::from_raw(detail::signed_magnitude(detail::round_shift(mag, static_cast<unsigned>(-exponent)), x < 0));
}
template<unsigned F> constexpr fixed<F> scalbn(fixed<F> x, int exponent) noexcept { return ldexp(x, exponent); }
template<unsigned F> fixed<F> frexp(fixed<F> x, int* exponent) {
    if (!exponent) throw std::invalid_argument("fixed point frexp needs an exponent pointer");
    if (!x) { *exponent = 0; return {}; }
    *exponent = static_cast<int>(detail::bit_width(detail::magnitude(x.raw_value()))) - static_cast<int>(F);
    auto result = ldexp(x, -*exponent);
    if (abs(result) == 1) { result = ldexp(result, -1); ++*exponent; }
    return result;
}
template<unsigned F> constexpr fixed<F> nextafter(fixed<F> from, fixed<F> toward) noexcept {
    return from == toward ? toward : fixed<F>::from_raw(from.raw_value() + (from < toward ? 1 : -1));
}
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
constexpr auto fmin(A a, B b) noexcept { using R = detail::promoted<A, B>; return R(a) < R(b) ? R(a) : R(b); }
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
constexpr auto fmax(A a, B b) noexcept { using R = detail::promoted<A, B>; return R(a) > R(b) ? R(a) : R(b); }
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
constexpr auto fdim(A a, B b) noexcept { using R = detail::promoted<A, B>; return a > b ? a - b : R{}; }

template<class T = real> constexpr T pi_v = T::from_string("3.1415926535897932384626433832795028841971693993751");
template<class T = real> constexpr T e_v = T::from_string("2.7182818284590452353602874713526624977572470937000");
namespace detail {
template<unsigned F> constexpr fixed<F> from_q60(std::int64_t value) {
    return fixed<F>::from_raw(signed_magnitude(round_shift({0, magnitude(value)}, 60 - F), value < 0));
}
template<unsigned F> constexpr signed_wide to_q60(fixed<F> value) {
    return {shl({0, magnitude(value.raw_value())}, 60 - F), value < 0};
}
template<unsigned F> fixed<F> from_log(signed_wide value, std::uint64_t divisor = 0) {
    const auto mag = divisor ? round_div(shl(value.mag, F), divisor) : round_shift(value.mag, 60 - F);
    return fixed<F>::from_raw(signed_magnitude(mag, value.negative));
}
}
template<unsigned F> std::pair<fixed<F>, fixed<F>> sincos(fixed<F> x) {
    const auto result = detail::sincos_kernel(x.raw_value(), F);
    return {detail::from_q60<F>(result.sine), detail::from_q60<F>(result.cosine)};
}
template<unsigned F> fixed<F> sin(fixed<F> x) { return sincos(x).first; }
template<unsigned F> fixed<F> cos(fixed<F> x) { return sincos(x).second; }
template<unsigned F> fixed<F> tan(fixed<F> x) {
    return fixed<F>::from_raw(detail::tan_kernel(x.raw_value(), F));
}
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
auto atan2(A y, B x) {
    using R = detail::promoted<A, B>;
    const auto a = R(y).raw_value(), b = R(x).raw_value();
    return detail::from_q60<R::fractional_bits>(detail::atan2_kernel(detail::magnitude(a), a < 0, detail::magnitude(b), b < 0));
}
template<unsigned F> fixed<F> atan(fixed<F> x) { return atan2(x, fixed<F>(1)); }
template<unsigned F> fixed<F> asin(fixed<F> x) {
    const auto mag = detail::magnitude(x.raw_value());
    if (mag > fixed<F>::scale) throw std::domain_error("fixed point asin requires -1 <= x <= 1");
    const auto difference = detail::sub(detail::shl({0, 1}, 2 * F), detail::mul(mag, mag));
    const auto adjacent = detail::rounded_sqrt(detail::shl(difference, 2 * (60 - F)));
    return detail::from_q60<F>(detail::atan2_kernel(mag << (60 - F), x < 0, adjacent, false));
}
template<unsigned F> fixed<F> acos(fixed<F> x) {
    const auto mag = detail::magnitude(x.raw_value());
    if (mag > fixed<F>::scale) throw std::domain_error("fixed point acos requires -1 <= x <= 1");
    const auto difference = detail::sub(detail::shl({0, 1}, 2 * F), detail::mul(mag, mag));
    const auto opposite = detail::rounded_sqrt(detail::shl(difference, 2 * (60 - F)));
    return detail::from_q60<F>(detail::atan2_kernel(opposite, false, mag << (60 - F), x < 0));
}
template<unsigned F> fixed<F> exp(fixed<F> x) { return fixed<F>::from_raw(detail::exp_kernel(detail::to_q60(x), F)); }
template<unsigned F> fixed<F> expm1(fixed<F> x) { return fixed<F>::from_raw(detail::exp_kernel(detail::to_q60(x), F, true)); }
template<unsigned F> fixed<F> exp2(fixed<F> x) {
    // Reduce in base two first, preserving exact results at integer exponents.
    const auto raw = x.raw_value();
    if (raw >= static_cast<std::int64_t>(64 * fixed<F>::scale)) return fixed<F>::max();
    if (raw <= -static_cast<std::int64_t>(64 * fixed<F>::scale)) return {};
    auto k = static_cast<int>(raw / static_cast<std::int64_t>(fixed<F>::scale));
    auto rest = raw % static_cast<std::int64_t>(fixed<F>::scale);
    if (rest > static_cast<std::int64_t>(fixed<F>::scale / 2)) { ++k; rest -= static_cast<std::int64_t>(fixed<F>::scale); }
    if (rest < -static_cast<std::int64_t>(fixed<F>::scale / 2)) { --k; rest += static_cast<std::int64_t>(fixed<F>::scale); }
    const auto r = detail::mul_shift(rest, detail::ln2, F);
    const auto s = static_cast<std::uint64_t>(detail::exp_series(r));
    const auto shift = k + static_cast<int>(F) - 60;
    return fixed<F>::from_raw(detail::signed_magnitude(shift >= 0 ? detail::shl({0, s}, static_cast<unsigned>(shift))
        : detail::round_shift({0, s}, static_cast<unsigned>(-shift)), false));
}
template<unsigned F> fixed<F> log(fixed<F> x) {
    if (x <= 0) throw std::domain_error("fixed point log requires x > 0");
    return detail::from_log<F>(detail::log_kernel(static_cast<std::uint64_t>(x.raw_value()), F));
}
template<unsigned F> fixed<F> log2(fixed<F> x) {
    if (x <= 0) throw std::domain_error("fixed point log2 requires x > 0");
    return detail::from_log<F>(detail::log_kernel(static_cast<std::uint64_t>(x.raw_value()), F), detail::ln2);
}
template<unsigned F> fixed<F> log10(fixed<F> x) {
    if (x <= 0) throw std::domain_error("fixed point log10 requires x > 0");
    return detail::from_log<F>(detail::log_kernel(static_cast<std::uint64_t>(x.raw_value()), F), detail::ln10);
}
template<unsigned F> fixed<F> log1p(fixed<F> x) {
    if (x <= -1) throw std::domain_error("fixed point log1p requires x > -1");
    const auto raw = x.raw_value() < 0 ? fixed<F>::scale - detail::magnitude(x.raw_value())
        : static_cast<std::uint64_t>(x.raw_value()) + fixed<F>::scale;
    return detail::from_log<F>(detail::log_kernel(raw, F));
}
template<unsigned F> fixed<F> sqrt(fixed<F> x) {
    if (x < 0) throw std::domain_error("fixed point sqrt requires x >= 0");
    return fixed<F>::from_raw(static_cast<std::int64_t>(detail::rounded_sqrt(detail::shl({0, static_cast<std::uint64_t>(x.raw_value())}, F))));
}
template<unsigned F> fixed<F> cbrt(fixed<F> x) {
    const auto root = detail::rounded_cbrt(detail::shl({0, detail::magnitude(x.raw_value())}, 2 * F));
    return fixed<F>::from_raw(detail::signed_magnitude({0, root}, x < 0));
}
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
auto hypot(A a, B b) {
    using R = detail::promoted<A, B>;
    const auto x = detail::magnitude(R(a).raw_value()), y = detail::magnitude(R(b).raw_value());
    return R::from_raw(detail::signed_magnitude({0, detail::rounded_sqrt(detail::add(detail::mul(x, x), detail::mul(y, y)))}, false));
}
template<unsigned F> fixed<F> hypot(fixed<F> a, fixed<F> b, fixed<F> c) {
    const auto x = detail::magnitude(a.raw_value()), y = detail::magnitude(b.raw_value()), z = detail::magnitude(c.raw_value());
    const auto sum = detail::add(detail::add(detail::mul(x, x), detail::mul(y, y)), detail::mul(z, z));
    return fixed<F>::from_raw(detail::signed_magnitude({0, detail::rounded_sqrt(sum)}, false));
}
template<unsigned F, class I, std::enable_if_t<std::is_integral_v<I>, int> = 0>
fixed<F> pow(fixed<F> base, I exponent) {
    bool negative = false;
    if constexpr (std::is_signed_v<I>) negative = exponent < 0;
    auto n = static_cast<std::uint64_t>(exponent);
    if (negative) { n = std::uint64_t{0} - n; base = fixed<F>(1) / base; }
    fixed<F> result = 1;
    while (n) {
        if (n & 1) result *= base;
        n >>= 1;
        if (n) base *= base;
    }
    return result;
}
template<class A, class B, std::enable_if_t<detail::binary_v<A, B> && detail::is_fixed_v<B>, int> = 0>
auto pow(A a, B b) {
    using R = detail::promoted<A, B>;
    const R x(a), y(b);
    if (y.raw_value() % static_cast<std::int64_t>(R::scale) == 0)
        return pow(x, y.raw_value() / static_cast<std::int64_t>(R::scale));
    if (x < 0) throw std::domain_error("fixed point pow of a negative base needs an integer exponent");
    if (!x) {
        if (y < 0) throw std::domain_error("fixed point zero to a negative power");
        return R{};
    }
    const auto logarithm = detail::log_kernel(static_cast<std::uint64_t>(x.raw_value()), R::fractional_bits);
    const auto product = detail::mul_wide(logarithm.mag, detail::magnitude(y.raw_value()));
    constexpr auto F = R::fractional_bits;
    detail::u128 shifted{(product.low.hi >> F) | (product.hi << (64 - F)),
                         (product.low.lo >> F) | (product.low.hi << (64 - F))};
    const auto residual = product.low.lo & (R::scale - 1);
    if (residual > R::scale / 2 || (residual == R::scale / 2 && (shifted.lo & 1))) shifted = detail::add(shifted, {0, 1});
    return R::from_raw(detail::exp_kernel({shifted, logarithm.negative != (y < 0)}, F));
}
} // namespace fxp
