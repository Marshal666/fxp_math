#pragma once
#include "fixed.hpp"
#include <array>
#include <cstddef>

namespace fxp {
enum class byte_order { little_endian, big_endian };
namespace detail {
template<class T, unsigned Mantissa, unsigned Exponent, int Bias>
constexpr T decode_ieee(std::uint64_t bits) {
    constexpr auto fraction_mask = (UINT64_C(1) << Mantissa) - 1;
    constexpr auto exponent_mask = (UINT64_C(1) << Exponent) - 1;
    const bool negative = ((bits >> (Mantissa + Exponent)) & 1) != 0;
    const auto exponent = (bits >> Mantissa) & exponent_mask;
    auto significand = bits & fraction_mask;
    if (exponent == exponent_mask) {
        if (significand) throw std::domain_error("IEEE NaN cannot be converted to fixed point");
        return negative ? T::min() : T::max();
    }
    if (exponent) significand |= UINT64_C(1) << Mantissa;
    if (!significand) return {};
    const int shift = static_cast<int>(exponent ? exponent : 1) - Bias - static_cast<int>(Mantissa)
        + static_cast<int>(T::fractional_bits);
    if (shift >= 0 && static_cast<unsigned>(shift) + bit_width(significand) > 64)
        return negative ? T::min() : T::max();
    const auto mag = shift >= 0 ? shl({0, significand}, static_cast<unsigned>(shift))
        : round_shift({0, significand}, static_cast<unsigned>(-shift));
    return T::from_raw(signed_magnitude(mag, negative));
}
template<unsigned Mantissa, unsigned Exponent, int Bias, unsigned F>
constexpr std::uint64_t encode_ieee(fixed<F> value) {
    const auto mag = magnitude(value.raw_value());
    if (!mag) return 0; // Fixed point has one zero; emit positive zero.
    auto top = bit_width(mag) - 1;
    auto significand = top > Mantissa ? round_shift({0, mag}, top - Mantissa).lo : mag << (Mantissa - top);
    if (significand == (UINT64_C(1) << (Mantissa + 1))) { significand >>= 1; ++top; }
    const auto exponent = static_cast<std::uint64_t>(static_cast<int>(top) - static_cast<int>(F) + Bias);
    return (static_cast<std::uint64_t>(value.raw_value() < 0) << (Mantissa + Exponent)) |
        (exponent << Mantissa) | (significand & ((UINT64_C(1) << Mantissa) - 1));
}
template<std::size_t N> constexpr std::uint64_t read_bytes(const std::array<std::uint8_t, N>& bytes, byte_order order) {
    std::uint64_t bits = 0;
    for (std::size_t i = 0; i < N; ++i) {
        const auto index = order == byte_order::big_endian ? i : N - 1 - i;
        bits = (bits << 8) | bytes[index];
    }
    return bits;
}
template<std::size_t N> constexpr std::array<std::uint8_t, N> write_bytes(std::uint64_t bits, byte_order order) {
    std::array<std::uint8_t, N> result{};
    for (std::size_t i = 0; i < N; ++i) {
        const auto index = order == byte_order::little_endian ? i : N - 1 - i;
        result[index] = static_cast<std::uint8_t>(bits & 255);
        bits >>= 8;
    }
    return result;
}
} // namespace detail
template<class T = real> constexpr T from_ieee754_binary32_bits(std::uint32_t bits) {
    return detail::decode_ieee<T, 23, 8, 127>(bits);
}
template<class T = real> constexpr T from_ieee754_binary64_bits(std::uint64_t bits) {
    return detail::decode_ieee<T, 52, 11, 1023>(bits);
}
template<unsigned F> constexpr std::uint32_t to_ieee754_binary32_bits(fixed<F> value) {
    return static_cast<std::uint32_t>(detail::encode_ieee<23, 8, 127>(value));
}
template<unsigned F> constexpr std::uint64_t to_ieee754_binary64_bits(fixed<F> value) {
    return detail::encode_ieee<52, 11, 1023>(value);
}
template<class T = real> constexpr T from_ieee754_binary32_bytes(const std::array<std::uint8_t, 4>& bytes, byte_order order) {
    return from_ieee754_binary32_bits<T>(static_cast<std::uint32_t>(detail::read_bytes(bytes, order)));
}
template<class T = real> constexpr T from_ieee754_binary64_bytes(const std::array<std::uint8_t, 8>& bytes, byte_order order) {
    return from_ieee754_binary64_bits<T>(detail::read_bytes(bytes, order));
}
template<unsigned F> constexpr auto to_ieee754_binary32_bytes(fixed<F> value, byte_order order) {
    return detail::write_bytes<4>(to_ieee754_binary32_bits(value), order);
}
template<unsigned F> constexpr auto to_ieee754_binary64_bytes(fixed<F> value, byte_order order) {
    return detail::write_bytes<8>(to_ieee754_binary64_bits(value), order);
}
} // namespace fxp
