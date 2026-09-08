#pragma once
#include "detail/decimal.hpp"
#include <istream>
#include <functional>
#include <ostream>
#include <string>
#include <type_traits>

namespace fxp {
template<unsigned FractionBits> class fixed;
namespace detail {
template<class T> struct fixed_traits { static constexpr bool value = false; static constexpr unsigned bits = 0; };
template<unsigned F> struct fixed_traits<fixed<F>> { static constexpr bool value = true; static constexpr unsigned bits = F; };
template<class T> constexpr bool is_fixed_v = fixed_traits<std::decay_t<T>>::value;
template<class A, class B> constexpr bool binary_v =
    (is_fixed_v<A> || is_fixed_v<B>) &&
    (is_fixed_v<A> || std::is_integral_v<A>) && (is_fixed_v<B> || std::is_integral_v<B>);
template<class A, class B> using promoted = fixed<(fixed_traits<std::decay_t<A>>::bits > fixed_traits<std::decay_t<B>>::bits
    ? fixed_traits<std::decay_t<A>>::bits : fixed_traits<std::decay_t<B>>::bits)>;
}

template<unsigned FractionBits>
class fixed {
    static_assert(FractionBits == 16 || FractionBits == 32, "supported formats are Q48.16 and Q32.32");
    std::int64_t value_ = 0;
public:
    static constexpr unsigned fractional_bits = FractionBits;
    static constexpr std::uint64_t scale = UINT64_C(1) << FractionBits;
    constexpr fixed() noexcept = default;
    template<class I, std::enable_if_t<std::is_integral_v<I>, int> = 0>
    constexpr fixed(I value) noexcept {
        static_assert(sizeof(I) <= sizeof(std::uint64_t), "integer exceeds 64 bits");
        bool negative = false;
        if constexpr (std::is_signed_v<I>) negative = value < 0;
        const auto bits = static_cast<std::uint64_t>(value);
        const auto mag = negative ? std::uint64_t{0} - bits : bits;
        value_ = detail::signed_magnitude(detail::shl({0, mag}, FractionBits), negative);
    }
    // Deleted overloads reject construction, assignment, and accidental float operands.
    template<class Floating, std::enable_if_t<std::is_floating_point_v<Floating>, int> = 0>
    fixed(Floating floating_point_types_cannot_be_used_with_fixed_types) = delete;

    template<unsigned G>
    constexpr fixed(fixed<G> other) noexcept {
        const detail::u128 mag{0, detail::magnitude(other.raw_value())};
        if constexpr (G < FractionBits)
            value_ = detail::signed_magnitude(detail::shl(mag, FractionBits - G), other.raw_value() < 0);
        else value_ = detail::signed_magnitude(detail::round_shift(mag, G - FractionBits), other.raw_value() < 0);
    }
    explicit constexpr fixed(std::string_view decimal) : value_(detail::parse_decimal<FractionBits>(decimal)) {}
    static constexpr fixed from_raw(std::int64_t raw) noexcept { fixed result; result.value_ = raw; return result; }
    static constexpr fixed from_string(std::string_view decimal) { return fixed(decimal); }
    constexpr std::int64_t raw_value() const noexcept { return value_; }
    static constexpr fixed min() noexcept { return from_raw(std::numeric_limits<std::int64_t>::min()); }
    static constexpr fixed max() noexcept { return from_raw(std::numeric_limits<std::int64_t>::max()); }
    static constexpr fixed epsilon() noexcept { return from_raw(1); }

    template<class I = std::int64_t, std::enable_if_t<std::is_integral_v<I>, int> = 0>
    constexpr I to_integer() const noexcept {
        const auto mag = detail::round_shift({0, detail::magnitude(value_)}, FractionBits).lo;
        if constexpr (std::is_same_v<I, bool>) return mag != 0;
        else if constexpr (std::is_unsigned_v<I>) {
            if (value_ < 0) return 0;
            return mag > std::numeric_limits<I>::max() ? std::numeric_limits<I>::max() : static_cast<I>(mag);
        } else {
            const auto limit = static_cast<std::uint64_t>(std::numeric_limits<I>::max()) + (value_ < 0);
            if (mag >= limit) return value_ < 0 ? std::numeric_limits<I>::min() : std::numeric_limits<I>::max();
            return static_cast<I>(value_ < 0 ? -static_cast<std::int64_t>(mag) : static_cast<std::int64_t>(mag));
        }
    }
    template<class I, std::enable_if_t<std::is_integral_v<I> && !std::is_same_v<I, bool>, int> = 0>
    explicit constexpr operator I() const noexcept { return to_integer<I>(); }
    explicit constexpr operator bool() const noexcept { return value_ != 0; }
    std::string to_string() const {
        const auto mag = detail::magnitude(value_);
        auto result = std::to_string(mag >> FractionBits);
        auto fraction = mag & (scale - 1);
        if (fraction) {
            result += '.';
            while (fraction) {
                fraction *= 10;
                result += static_cast<char>('0' + (fraction >> FractionBits));
                fraction &= scale - 1;
            }
        }
        if (value_ < 0) result.insert(result.begin(), '-');
        return result;
    }
    explicit operator std::string() const { return to_string(); }
    constexpr fixed operator+() const noexcept { return *this; }
    constexpr fixed operator-() const noexcept {
        return from_raw(detail::signed_magnitude({0, detail::magnitude(value_)}, value_ >= 0));
    }
    template<class T> fixed& operator+=(T value) { return *this = *this + value; }
    template<class T> fixed& operator-=(T value) { return *this = *this - value; }
    template<class T> fixed& operator*=(T value) { return *this = *this * value; }
    template<class T> fixed& operator/=(T value) { return *this = *this / value; }
    template<class T> fixed& operator%=(T value) { return *this = *this % value; }
    fixed& operator++() { return *this += 1; }
    fixed operator++(int) { auto old = *this; ++*this; return old; }
    fixed& operator--() { return *this -= 1; }
    fixed operator--(int) { auto old = *this; --*this; return old; }
};

using q48_16 = fixed<16>;
using q32_32 = fixed<32>;
using real = q48_16;
using fine = q32_32;

template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
constexpr auto operator+(A a, B b) noexcept {
    using R = detail::promoted<A, B>;
    return R::from_raw(detail::sat_add(R(a).raw_value(), R(b).raw_value()));
}
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
constexpr auto operator-(A a, B b) noexcept {
    using R = detail::promoted<A, B>;
    return R::from_raw(detail::sat_sub(R(a).raw_value(), R(b).raw_value()));
}
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
auto operator*(A a, B b) noexcept {
    using R = detail::promoted<A, B>;
    return R::from_raw(detail::mul_shift(R(a).raw_value(), R(b).raw_value(), R::fractional_bits));
}
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
auto operator/(A a, B b) {
    using R = detail::promoted<A, B>;
    return R::from_raw(detail::div_shift(R(a).raw_value(), R(b).raw_value(), R::fractional_bits));
}
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0>
auto operator%(A a, B b) {
    using R = detail::promoted<A, B>;
    const auto x = R(a).raw_value(), y = R(b).raw_value();
    if (!y) throw std::domain_error("fixed point fmod by zero");
    return R::from_raw(detail::signed_magnitude({0, detail::magnitude(x) % detail::magnitude(y)}, x < 0));
}
#define FXP_COMPARE(OP) \
template<class A, class B, std::enable_if_t<detail::binary_v<A, B>, int> = 0> \
constexpr bool operator OP(A a, B b) noexcept { \
    using R = detail::promoted<A, B>; return R(a).raw_value() OP R(b).raw_value(); \
}
FXP_COMPARE(==) FXP_COMPARE(!=) FXP_COMPARE(<) FXP_COMPARE(<=) FXP_COMPARE(>) FXP_COMPARE(>=)
#undef FXP_COMPARE

template<unsigned F> std::string to_string(fixed<F> value) { return value.to_string(); }
template<class T = real> constexpr T from_string(std::string_view text) { return T::from_string(text); }
template<unsigned F> std::ostream& operator<<(std::ostream& out, fixed<F> value) { return out << value.to_string(); }
template<unsigned F> std::istream& operator>>(std::istream& in, fixed<F>& value) {
    std::string text;
    if (in >> text) {
        try { value = fixed<F>::from_string(text); }
        catch (const std::invalid_argument&) { in.setstate(std::ios::failbit); }
    }
    return in;
}
namespace literals {
template<char... C> constexpr real operator""_r() {
    constexpr char text[] = {C..., '\0'};
    return real::from_string(std::string_view(text, sizeof...(C)));
}
template<char... C> constexpr fine operator""_fine() {
    constexpr char text[] = {C..., '\0'};
    return fine::from_string(std::string_view(text, sizeof...(C)));
}
} // namespace literals
} // namespace fxp

namespace std {
template<unsigned F> struct numeric_limits<fxp::fixed<F>> {
    using T = fxp::fixed<F>;
    static constexpr bool is_specialized = true, is_signed = true, is_integer = false,
        is_exact = true, has_infinity = false, has_quiet_NaN = false, has_signaling_NaN = false,
        is_iec559 = false, is_bounded = true, is_modulo = false, traps = false,
        tinyness_before = false, has_denorm_loss = false;
    static constexpr int radix = 2, digits = 63, digits10 = 18, max_digits10 = 20,
        min_exponent = 0, max_exponent = 0, min_exponent10 = 0, max_exponent10 = 0;
    static constexpr float_denorm_style has_denorm = denorm_absent;
    static constexpr float_round_style round_style = round_to_nearest;
    static constexpr T min() noexcept { return T::epsilon(); }
    static constexpr T lowest() noexcept { return T::min(); }
    static constexpr T max() noexcept { return T::max(); }
    static constexpr T epsilon() noexcept { return T::epsilon(); }
    static constexpr T round_error() noexcept { return T::from_raw(T::scale / 2); }
    static constexpr T infinity() noexcept { return {}; }
    static constexpr T quiet_NaN() noexcept { return {}; }
    static constexpr T signaling_NaN() noexcept { return {}; }
    static constexpr T denorm_min() noexcept { return {}; }
};
template<unsigned F, unsigned G> struct common_type<fxp::fixed<F>, fxp::fixed<G>> {
    using type = fxp::fixed<(F > G ? F : G)>;
};
template<unsigned F> struct hash<fxp::fixed<F>> {
    size_t operator()(fxp::fixed<F> value) const noexcept { return hash<int64_t>{}(value.raw_value()); }
};
} // namespace std
