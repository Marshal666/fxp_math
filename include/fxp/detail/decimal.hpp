#pragma once
#include "wide.hpp"
#include <array>
#include <string_view>

namespace fxp::detail {
constexpr bool digit(char c) { return c >= '0' && c <= '9'; }

// A rounding midpoint at F binary places terminates within F+1 decimal places.
// Keep that many places and a sticky bit; even very long decimals round exactly.
template<unsigned F>
constexpr std::int64_t parse_decimal(std::string_view text) {
    if (text.empty() || text.size() > 1000000)
        throw std::invalid_argument("invalid fixed point decimal length");
    std::size_t start = 0;
    bool negative = false;
    if (text[0] == '+' || text[0] == '-') { negative = text[0] == '-'; ++start; }
    bool point = false;
    std::int64_t digits = 0, before = 0;
    auto end = start;
    for (; end < text.size() && text[end] != 'e' && text[end] != 'E'; ++end) {
        const char c = text[end];
        if (digit(c)) { ++digits; if (!point) ++before; }
        else if (c == '.' && !point) point = true;
        else if (c == '\'' && end > start && end + 1 < text.size() &&
                 digit(text[end - 1]) && digit(text[end + 1])) {}
        else throw std::invalid_argument("invalid fixed point decimal");
    }
    if (!digits) throw std::invalid_argument("fixed point decimal needs digits");
    std::int64_t exponent = 0;
    if (end < text.size()) {
        auto i = end + 1;
        bool exp_negative = false;
        if (i < text.size() && (text[i] == '+' || text[i] == '-')) {
            exp_negative = text[i] == '-'; ++i;
        }
        if (i == text.size()) throw std::invalid_argument("fixed point exponent needs digits");
        for (; i < text.size(); ++i) {
            if (!digit(text[i])) throw std::invalid_argument("invalid fixed point exponent");
            if (exponent < 1000000) exponent = exponent * 10 + (text[i] - '0');
        }
        if (exp_negative) exponent = -exponent;
    }
    const auto whole_count = before + exponent;
    std::uint64_t whole = 0;
    bool overflow = false, sticky = false;
    constexpr auto whole_limit = UINT64_C(1) << (63 - F);
    std::array<unsigned, F + 1> decimal{};
    std::int64_t index = 0;
    for (auto i = start; i < end; ++i) {
        if (!digit(text[i])) continue;
        const auto d = static_cast<unsigned>(text[i] - '0');
        if (index < whole_count) {
            if (whole > whole_limit / 10 || whole * 10 + d > whole_limit) overflow = true;
            if (!overflow) whole = whole * 10 + d;
        } else {
            const auto place = index - whole_count;
            if (place <= F) decimal[static_cast<std::size_t>(place)] = d;
            else sticky = sticky || d != 0;
        }
        ++index;
    }
    if (whole) {
        for (auto i = digits; i < whole_count && !overflow; ++i) {
            if (whole > whole_limit / 10) overflow = true;
            else whole *= 10;
        }
    }
    if (overflow) return signed_magnitude({1, 0}, negative);
    u128 numerator{}, denominator{0, 1};
    for (const auto d : decimal) {
        numerator = add(add(shl(numerator, 3), shl(numerator, 1)), {0, d});
        denominator = add(shl(denominator, 3), shl(denominator, 1));
    }
    std::uint64_t fraction = 0;
    for (unsigned i = 0; i < F; ++i) {
        numerator = shl(numerator, 1);
        fraction <<= 1;
        if (numerator >= denominator) { numerator = sub(numerator, denominator); ++fraction; }
    }
    numerator = shl(numerator, 1);
    if (numerator >= denominator && (numerator != denominator || sticky || (fraction & 1))) ++fraction;
    return signed_magnitude(add(shl({0, whole}, F), {0, fraction}), negative);
}
} // namespace fxp::detail
