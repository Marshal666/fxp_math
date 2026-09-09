#pragma once
#include "math.hpp"
#include <array>
#include <algorithm>
#include <cstddef>
#include <functional>

namespace fxp {
namespace geometry_detail {
template <class T> T &output(T *p) {
    if (!p)
        throw std::invalid_argument("fixed geometry needs an output pointer");
    return *p;
}
template <class T> constexpr int sign(T value) {
    return (value > 0) - (value < 0);
}
template <class T> constexpr T clamp(T value, T low, T high) {
    return value < low ? low : value > high ? high : value;
}
template <class T> constexpr T tolerance(const char *decimal) {
    return (std::max)(T::epsilon(), T::from_string(decimal));
}
template <class T, std::size_t N> struct vector_storage;
template <class T> struct vector_storage<T, 2> {
    T x{}, y{};
    constexpr T &component(std::size_t i) {
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        throw std::out_of_range("vec2 index");
    }
    constexpr const T &component(std::size_t i) const {
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        throw std::out_of_range("vec2 index");
    }
};
template <class T> struct vector_storage<T, 3> {
    T x{}, y{}, z{};
    constexpr T &component(std::size_t i) {
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        if (i == 2)
            return z;
        throw std::out_of_range("vec3 index");
    }
    constexpr const T &component(std::size_t i) const {
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        if (i == 2)
            return z;
        throw std::out_of_range("vec3 index");
    }
};
template <class T> struct vector_storage<T, 4> {
    T x{}, y{}, z{}, w{};
    constexpr T &component(std::size_t i) {
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        if (i == 2)
            return z;
        if (i == 3)
            return w;
        throw std::out_of_range("vec4 index");
    }
    constexpr const T &component(std::size_t i) const {
        if (i == 0)
            return x;
        if (i == 1)
            return y;
        if (i == 2)
            return z;
        if (i == 3)
            return w;
        throw std::out_of_range("vec4 index");
    }
};
} // namespace geometry_detail

template <class T, std::size_t N> class vector : public geometry_detail::vector_storage<T, N> {
    static_assert(detail::is_fixed_v<T>,
                  "geometry requires real or fine; floating point types cannot be used");
    static_assert(N >= 2 && N <= 4, "vectors have two, three, or four components");

  public:
    using value_type = T;
    static constexpr std::size_t size = N;
    constexpr vector() = default;
    template <class... A,
              std::enable_if_t<sizeof...(A) == N && (std::is_convertible_v<A, T> && ...), int> = 0>
    constexpr vector(A... values) {
        const std::array<T, N> a{T(values)...};
        for (std::size_t i = 0; i < N; ++i)
            (*this)[i] = a[i];
    }
    template <class U> constexpr vector(const vector<U, N> &value) {
        for (std::size_t i = 0; i < N; ++i)
            (*this)[i] = T(value[i]);
    }
    template <
        class U, std::size_t M, class... A,
        std::enable_if_t<(M < N) && M + sizeof...(A) == N && (std::is_convertible_v<A, T> && ...), int> = 0>
    constexpr vector(const vector<U, M> &prefix, A... tail) {
        for (std::size_t i = 0; i < M; ++i)
            (*this)[i] = T(prefix[i]);
        const std::array<T, sizeof...(A)> a{T(tail)...};
        for (std::size_t i = M; i < N; ++i)
            (*this)[i] = a[i - M];
    }
    constexpr T &operator[](std::size_t i) { return this->component(i); }
    constexpr const T &operator[](std::size_t i) const { return this->component(i); }
    // Alternative component names are accessors, never inactive union members.
    constexpr T &u() { return (*this)[0]; }
    constexpr const T &u() const { return (*this)[0]; }
    constexpr T &v() { return (*this)[1]; }
    constexpr const T &v() const { return (*this)[1]; }
    constexpr T &r() { return (*this)[0]; }
    constexpr const T &r() const { return (*this)[0]; }
    constexpr T &g() { return (*this)[1]; }
    constexpr const T &g() const { return (*this)[1]; }
    template <std::size_t M = N, std::enable_if_t<(M >= 3), int> = 0> constexpr T &b() { return (*this)[2]; }
    template <std::size_t M = N, std::enable_if_t<(M >= 3), int> = 0> constexpr const T &b() const {
        return (*this)[2];
    }
    template <std::size_t M = N, std::enable_if_t<(M >= 3), int> = 0> constexpr T &q() { return (*this)[2]; }
    template <std::size_t M = N, std::enable_if_t<(M >= 3), int> = 0> constexpr const T &q() const {
        return (*this)[2];
    }
    template <std::size_t M = N, std::enable_if_t<M == 4, int> = 0> constexpr T &a() { return (*this)[3]; }
    template <std::size_t M = N, std::enable_if_t<M == 4, int> = 0> constexpr const T &a() const {
        return (*this)[3];
    }
    template <class... A> constexpr void set(A... values) { *this = vector(values...); }
    void maximize(const vector &other) {
        for (std::size_t i = 0; i < N; ++i)
            (*this)[i] = (std::max)((*this)[i], other[i]);
    }
    void minimize(const vector &other) {
        for (std::size_t i = 0; i < N; ++i)
            (*this)[i] = (std::min)((*this)[i], other[i]);
    }
    void negate() { *this = -*this; }
    void negate(const vector &value) { *this = -value; }
    void add(const vector &a, const vector &b) { *this = a + b; }
    void sub(const vector &a, const vector &b) { *this = a - b; }
    void displace(const vector &a, const vector &b, T t) { *this = a + t * b; }
    void displace(const vector &b, T t) { *this += t * b; }
    void lerp(T t, const vector &a, const vector &b) { *this = t * b + (T(1) - t) * a; }
    void interpolate(const vector &a, const vector &b, T t) { lerp(t, a, b); }
    template <class U> vector &operator+=(const vector<U, N> &other) { return *this = *this + other; }
    template <class U> vector &operator-=(const vector<U, N> &other) { return *this = *this - other; }
    template <class S, std::enable_if_t<detail::binary_v<T, S>, int> = 0> vector &operator*=(S value) {
        return *this = *this * value;
    }
    template <class S, std::enable_if_t<detail::binary_v<T, S>, int> = 0> vector &operator/=(S value) {
        return *this = *this / value;
    }
    template <std::size_t M = N, std::enable_if_t<M == 2, int> = 0> vector &operator^=(const vector &other) {
        return *this = *this ^ other;
    }
    template <std::size_t M = N, std::enable_if_t<M == 2, int> = 0>
    vector &complex_product(const vector &other) {
        return *this ^= other;
    }
};
template <class T = real> using vec2 = vector<T, 2>;
template <class T = real> using vec3 = vector<T, 3>;
template <class T = real> using vec4 = vector<T, 4>;
template <class T = real> using point2 = vec2<T>;
using real2 = vec2<real>;
using real3 = vec3<real>;
using real4 = vec4<real>;
using fine2 = vec2<fine>;
using fine3 = vec3<fine>;
using fine4 = vec4<fine>;

template <class T, std::size_t N> constexpr vector<T, N> operator-(const vector<T, N> &a) {
    vector<T, N> r;
    for (std::size_t i = 0; i < N; ++i)
        r[i] = -a[i];
    return r;
}
template <class A, class B, std::size_t N>
constexpr auto operator+(const vector<A, N> &a, const vector<B, N> &b) {
    vector<detail::promoted<A, B>, N> r;
    for (std::size_t i = 0; i < N; ++i)
        r[i] = a[i] + b[i];
    return r;
}
template <class A, class B, std::size_t N>
constexpr auto operator-(const vector<A, N> &a, const vector<B, N> &b) {
    vector<detail::promoted<A, B>, N> r;
    for (std::size_t i = 0; i < N; ++i)
        r[i] = a[i] - b[i];
    return r;
}
template <class A, class B, std::size_t N>
constexpr bool operator==(const vector<A, N> &a, const vector<B, N> &b) {
    for (std::size_t i = 0; i < N; ++i)
        if (a[i] != b[i])
            return false;
    return true;
}
template <class A, class B, std::size_t N>
constexpr bool operator!=(const vector<A, N> &a, const vector<B, N> &b) {
    return !(a == b);
}
template <class A, class B, std::size_t N> auto dot(const vector<A, N> &a, const vector<B, N> &b) {
    detail::promoted<A, B> r{};
    for (std::size_t i = 0; i < N; ++i)
        r += a[i] * b[i];
    return r;
}
template <class A, class B, std::size_t N> auto operator*(const vector<A, N> &a, const vector<B, N> &b) {
    return dot(a, b);
}
template <class T, class S, std::size_t N, std::enable_if_t<detail::binary_v<T, S>, int> = 0>
auto operator*(const vector<T, N> &a, S b) {
    vector<detail::promoted<T, S>, N> r;
    for (std::size_t i = 0; i < N; ++i)
        r[i] = a[i] * b;
    return r;
}
template <class T, class S, std::size_t N, std::enable_if_t<detail::binary_v<T, S>, int> = 0>
auto operator*(S a, const vector<T, N> &b) {
    return b * a;
}
template <class T, class S, std::size_t N, std::enable_if_t<detail::binary_v<T, S>, int> = 0>
auto operator/(const vector<T, N> &a, S b) {
    if (b == 0)
        throw std::domain_error("fixed vector division by zero");
    vector<detail::promoted<T, S>, N> r;
    for (std::size_t i = 0; i < N; ++i)
        r[i] = a[i] / b;
    return r;
}
template <class A, class B> auto cross(const vec2<A> &a, const vec2<B> &b) {
    return a.x * b.y - a.y * b.x;
}
template <class A, class B> auto cross(const vec3<A> &a, const vec3<B> &b) {
    return vec3<detail::promoted<A, B>>(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
template <class A, class B> auto complex_product(const vec2<A> &a, const vec2<B> &b) {
    return vec2<detail::promoted<A, B>>(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}
template <class A, class B> auto operator^(const vec2<A> &a, const vec2<B> &b) {
    return complex_product(a, b);
}
template <class A, class B> auto operator^(const vec3<A> &a, const vec3<B> &b) {
    return cross(a, b);
}
template <class T, std::size_t N> T length_squared(const vector<T, N> &a) {
    return dot(a, a);
}
template <class T, std::enable_if_t<detail::is_fixed_v<T>, int> = 0> T length_squared(T a) {
    return a * a;
}
template <class T, class... U, std::enable_if_t<detail::is_fixed_v<T> && (sizeof...(U) > 0), int> = 0>
T length_squared(T a, U... rest) {
    return length_squared(vector<T, 1 + sizeof...(U)>(a, rest...));
}
template <class T, std::size_t N> T length(const vector<T, N> &a) {
    detail::u128 sum{};
    const auto limit = detail::mul(UINT64_C(0x7fffffffffffffff), UINT64_C(0x7fffffffffffffff));
    for (std::size_t i = 0; i < N; ++i) {
        const auto mag = detail::magnitude(a[i].raw_value());
        sum = detail::add(sum, detail::mul(mag, mag));
        if (sum >= limit)
            return T::max();
    }
    return T::from_raw(detail::signed_magnitude({0, detail::rounded_sqrt(sum)}, false));
}
template <class T, std::enable_if_t<detail::is_fixed_v<T>, int> = 0> T length(T a) {
    return fxp::abs(a);
}
template <class T, class... U, std::enable_if_t<detail::is_fixed_v<T> && (sizeof...(U) > 0), int> = 0>
T length(T a, U... rest) {
    return length(vector<T, 1 + sizeof...(U)>(a, rest...));
}
template <class T, std::size_t N> bool normalize(vector<T, N> *value) {
    auto &v = geometry_detail::output(value);
    std::uint64_t largest = 0;
    for (std::size_t i = 0; i < N; ++i)
        largest = (std::max)(largest, detail::magnitude(v[i].raw_value()));
    if (!largest)
        return false;
    // Q4.60 ratios preserve tiny vectors and avoid a saturated public length.
    std::array<std::uint64_t, N> ratios{};
    detail::u128 squares{};
    for (std::size_t i = 0; i < N; ++i) {
        ratios[i] = detail::round_div(detail::shl({0, detail::magnitude(v[i].raw_value())}, 60), largest).lo;
        squares = detail::add(squares, detail::mul(ratios[i], ratios[i]));
    }
    const auto divisor = detail::rounded_sqrt(squares);
    for (std::size_t i = 0; i < N; ++i)
        v[i] = T::from_raw(
            detail::signed_magnitude(detail::round_div(detail::mul(ratios[i], T::scale), divisor), v[i] < 0));
    return true;
}
template <class T, std::size_t N> vector<T, N> normalized(vector<T, N> value) {
    normalize(&value);
    return value;
}
template <class T> bool normalize(T &x, T &y) {
    vec2<T> v(x, y);
    const bool ok = normalize(&v);
    x = v.x;
    y = v.y;
    return ok;
}
template <class T> bool normalize(T &x, T &y, T &z) {
    vec3<T> v(x, y, z);
    const bool ok = normalize(&v);
    x = v.x;
    y = v.y;
    z = v.z;
    return ok;
}
template <class T> bool normalize(T &x, T &y, T &z, T &w) {
    vec4<T> v(x, y, z, w);
    const bool ok = normalize(&v);
    x = v.x;
    y = v.y;
    z = v.z;
    w = v.w;
    return ok;
}
template <class A, class B, std::size_t N> auto distance(const vector<A, N> &a, const vector<B, N> &b) {
    return length(a - b);
}
template <class T, std::size_t N> T length_xy(const vector<T, N> &v) {
    return length(v.x, v.y);
}
template <class T, std::size_t N> T length_xy_squared(const vector<T, N> &v) {
    return length_squared(v.x, v.y);
}
template <class T> T length_xyz(const vec4<T> &v) {
    return length(v.x, v.y, v.z);
}
template <class T> T length_xyz_squared(const vec4<T> &v) {
    return length_squared(v.x, v.y, v.z);
}
template <class T = real> inline constexpr vec2<T> zero2_v{};
template <class T = real> inline constexpr vec2<T> axis2_x_v{1, 0};
template <class T = real> inline constexpr vec2<T> axis2_y_v{0, 1};
template <class T = real> inline constexpr vec3<T> zero3_v{};
template <class T = real> inline constexpr vec3<T> axis3_x_v{1, 0, 0};
template <class T = real> inline constexpr vec3<T> axis3_y_v{0, 1, 0};
template <class T = real> inline constexpr vec3<T> axis3_z_v{0, 0, 1};
template <class T = real> inline constexpr vec4<T> zero4_v{};
template <class T = real> inline constexpr vec4<T> axis4_x_v{1, 0, 0, 0};
template <class T = real> inline constexpr vec4<T> axis4_y_v{0, 1, 0, 0};
template <class T = real> inline constexpr vec4<T> axis4_z_v{0, 0, 1, 0};
template <class T = real> inline constexpr vec4<T> axis4_w_v{0, 0, 0, 1};
template <class T = real> struct vec3_hash {
    std::uint64_t operator()(const vec3<T> &v) const noexcept {
        std::uint64_t h = UINT64_C(14695981039346656037);
        for (std::size_t i = 0; i < 3; ++i)
            h = (h ^ static_cast<std::uint64_t>(v[i].raw_value())) * UINT64_C(1099511628211);
        return h;
    }
};
} // namespace fxp
