#pragma once
#include "fixed.hpp"
#include <cstddef>
#include <cstring>

#if !defined(FXP_DISABLE_SSE2) && (defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2))
#define FXP_DETAIL_HAS_SSE2 1
#include <emmintrin.h>
#else
#define FXP_DETAIL_HAS_SSE2 0
#endif

namespace fxp::batch {
inline constexpr bool has_sse2 = FXP_DETAIL_HAS_SSE2 != 0;
enum class backend { automatic, scalar };
namespace detail {
#if FXP_DETAIL_HAS_SSE2
inline __m128i signs(__m128i x) {
    return _mm_shuffle_epi32(_mm_srai_epi32(x, 31), _MM_SHUFFLE(3, 3, 1, 1));
}
template<bool Subtract> inline __m128i saturating(__m128i a, __m128i b) {
    const auto result = Subtract ? _mm_sub_epi64(a, b) : _mm_add_epi64(a, b);
    const auto overflow = signs(_mm_and_si128(_mm_xor_si128(a, result),
        Subtract ? _mm_xor_si128(a, b) : _mm_xor_si128(b, result)));
    const auto maximum = _mm_set_epi32(0x7fffffff, -1, 0x7fffffff, -1);
    const auto limit = _mm_xor_si128(signs(a), maximum);
    return _mm_or_si128(_mm_and_si128(overflow, limit), _mm_andnot_si128(overflow, result));
}
#endif
template<bool Subtract, unsigned F>
inline void add_sub(const fixed<F>* a, const fixed<F>* b, fixed<F>* out, std::size_t count, backend mode) {
    std::size_t i = 0;
#if FXP_DETAIL_HAS_SSE2
    static_assert(sizeof(fixed<F>) == 8 && std::is_trivially_copyable_v<fixed<F>>);
    if (mode == backend::automatic) {
        for (; count - i >= 2; i += 2) {
            __m128i av, bv;
            std::memcpy(&av, a + i, sizeof(av));
            std::memcpy(&bv, b + i, sizeof(bv));
            const auto result = saturating<Subtract>(av, bv);
            std::memcpy(static_cast<void*>(out + i), &result, sizeof(result));
        }
    }
#else
    (void)mode;
#endif
    for (; i < count; ++i) out[i] = Subtract ? a[i] - b[i] : a[i] + b[i];
}
} // namespace detail
// Exact in-place aliasing (out == a or out == b) is supported. Other overlap is not.
// Pointers may be null when count is zero. No alignment requirement beyond fixed<F>.
template<unsigned F> void add(const fixed<F>* a, const fixed<F>* b, fixed<F>* out, std::size_t count, backend mode = backend::automatic) {
    detail::add_sub<false>(a, b, out, count, mode);
}
template<unsigned F> void subtract(const fixed<F>* a, const fixed<F>* b, fixed<F>* out, std::size_t count, backend mode = backend::automatic) {
    detail::add_sub<true>(a, b, out, count, mode);
}
template<unsigned F> void multiply(const fixed<F>* a, const fixed<F>* b, fixed<F>* out, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) out[i] = a[i] * b[i];
}
template<unsigned F> void divide(const fixed<F>* a, const fixed<F>* b, fixed<F>* out, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) out[i] = a[i] / b[i];
}
} // namespace fxp::batch
#undef FXP_DETAIL_HAS_SSE2
