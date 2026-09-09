#pragma once
#include "geometry.hpp"
#include "ieee754.hpp"

namespace fxp {
template <class T = real> struct polar {
    static_assert(detail::is_fixed_v<T>, "polar requires real or fine");
    T latitude{}, longitude{};
    constexpr polar() = default;
    constexpr polar(T lat, T lon) : latitude(lat), longitude(lon) {}
    template <class U> constexpr polar(const polar<U> &p) : latitude(p.latitude), longitude(p.longitude) {}
    template <class U> polar &operator+=(const polar<U> &p) { return *this = *this + p; }
    template <class U> polar &operator-=(const polar<U> &p) { return *this = *this - p; }
};
template <class A, class B> auto operator+(const polar<A> &a, const polar<B> &b) {
    return polar<detail::promoted<A, B>>(a.latitude + b.latitude, a.longitude + b.longitude);
}
template <class A, class B> auto operator-(const polar<A> &a, const polar<B> &b) {
    return polar<detail::promoted<A, B>>(a.latitude - b.latitude, a.longitude - b.longitude);
}
template <class T = real> inline constexpr polar<T> zero_polar_v{};

namespace geometry_detail {
constexpr std::int32_t int32(std::int64_t n) {
    return n < INT32_MIN ? INT32_MIN : n > INT32_MAX ? INT32_MAX : static_cast<std::int32_t>(n);
}
inline void raster_bounds(std::int32_t x, std::int32_t y, std::int32_t r) {
    if (r < 0)
        throw std::domain_error("negative raster radius");
    if (std::int64_t(x) - r < INT32_MIN || std::int64_t(x) + r > INT32_MAX ||
        std::int64_t(y) - r < INT32_MIN || std::int64_t(y) + r > INT32_MAX)
        throw std::overflow_error("raster coordinates exceed int32");
}
} // namespace geometry_detail
struct int_vec2 {
    std::int32_t x{}, y{};
    constexpr int_vec2() = default;
    constexpr int_vec2(std::int32_t a, std::int32_t b) : x(a), y(b) {}
    template <class T>
    explicit int_vec2(const vec2<T> &v)
        : x(fxp::trunc(v.x).template to_integer<std::int32_t>()),
          y(fxp::trunc(v.y).template to_integer<std::int32_t>()) {}
    template <class T = real> vec2<T> to_vec2() const { return vec2<T>(x, y); }
    template <class T = real> vec2<T> norm() const { return fxp::normalized(to_vec2<T>()); }
    void turn_left() {
        const auto old = x;
        x = geometry_detail::int32(-std::int64_t(y));
        y = old;
    }
    void turn_right() {
        const auto old = y;
        y = geometry_detail::int32(-std::int64_t(x));
        x = old;
    }
    void turn_left_until90() { turn_left(); }
    void turn_right_until90() { turn_right(); }
    void turn_left_until_axis() {
        const auto sx = geometry_detail::sign(x), sy = geometry_detail::sign(y);
        if (sx && sx * sy >= 0) {
            x = 0;
            y = sx;
        } else {
            x = -sy;
            y = 0;
        }
    }
    void turn_right_until_axis() {
        const auto sx = geometry_detail::sign(x), sy = geometry_detail::sign(y);
        if (sx && sx * sy <= 0) {
            x = 0;
            y = -sx;
        } else {
            x = sy;
            y = 0;
        }
    }
    void turn_left_until45() {
        const auto sx = geometry_detail::sign(x), sy = geometry_detail::sign(y);
        if (sx && sx * sy >= 0) {
            x = detail::magnitude(x) > detail::magnitude(y) ? sx : 0;
            y = sx;
        } else {
            y = detail::magnitude(y) > detail::magnitude(x) ? sy : 0;
            x = -sy;
        }
    }
    void turn_right_until45() {
        const auto sx = geometry_detail::sign(x), sy = geometry_detail::sign(y);
        if (sx && sx * sy <= 0) {
            x = detail::magnitude(x) > detail::magnitude(y) ? sx : 0;
            y = -sx;
        } else {
            y = detail::magnitude(y) > detail::magnitude(x) ? sy : 0;
            x = sy;
        }
    }
    void turn_left_until135() {
        turn_left();
        turn_left();
        turn_right_until45();
    }
    void turn_right_until135() {
        turn_left();
        turn_left();
        turn_left_until45();
    }
    bool operator==(const int_vec2 &v) const { return x == v.x && y == v.y; }
    bool operator!=(const int_vec2 &v) const { return !(*this == v); }
    int_vec2 operator+(const int_vec2 &v) const {
        return {geometry_detail::int32(std::int64_t(x) + v.x), geometry_detail::int32(std::int64_t(y) + v.y)};
    }
    int_vec2 operator-(const int_vec2 &v) const {
        return {geometry_detail::int32(std::int64_t(x) - v.x), geometry_detail::int32(std::int64_t(y) - v.y)};
    }
    int_vec2 &operator+=(const int_vec2 &v) { return *this = *this + v; }
    int_vec2 &operator-=(const int_vec2 &v) { return *this = *this - v; }
    std::int32_t operator*(const int_vec2 &v) const {
        return geometry_detail::int32(detail::sat_add(std::int64_t(x) * v.x, std::int64_t(y) * v.y));
    }
    int_vec2 &operator*=(std::int32_t n) {
        x = geometry_detail::int32(std::int64_t(x) * n);
        y = geometry_detail::int32(std::int64_t(y) * n);
        return *this;
    }
    int_vec2 &operator/=(std::int32_t n) {
        if (!n)
            throw std::domain_error("integer vector division by zero");
        x = geometry_detail::int32(std::int64_t(x) / n);
        y = geometry_detail::int32(std::int64_t(y) / n);
        return *this;
    }
};
inline std::int32_t length_squared(const int_vec2 &v) {
    return v * v;
}
inline std::int32_t grid_distance(const int_vec2 &a, const int_vec2 &b) {
    return geometry_detail::int32(static_cast<std::int64_t>(
        (std::max)(detail::magnitude(std::int64_t(a.x) - b.x), detail::magnitude(std::int64_t(a.y) - b.y))));
}

constexpr std::uint32_t pack_dword(std::uint16_t high, std::uint16_t low) {
    return (std::uint32_t(high) << 16) | low;
}
constexpr std::uint32_t pack_dword(std::uint8_t b3, std::uint8_t b2, std::uint8_t b1, std::uint8_t b0) {
    return (std::uint32_t(b3) << 24) | (std::uint32_t(b2) << 16) | (std::uint32_t(b1) << 8) | b0;
}
constexpr std::uint16_t unpack_high_word(std::uint32_t n) {
    return static_cast<std::uint16_t>(n >> 16);
}
constexpr std::uint16_t unpack_low_word(std::uint32_t n) {
    return static_cast<std::uint16_t>(n);
}
constexpr std::uint8_t unpack_byte0(std::uint32_t n) {
    return static_cast<std::uint8_t>(n);
}
constexpr std::uint8_t unpack_byte1(std::uint32_t n) {
    return static_cast<std::uint8_t>(n >> 8);
}
constexpr std::uint8_t unpack_byte2(std::uint32_t n) {
    return static_cast<std::uint8_t>(n >> 16);
}
constexpr std::uint8_t unpack_byte3(std::uint32_t n) {
    return static_cast<std::uint8_t>(n >> 24);
}
template <class T> std::uint8_t norm_to_byte(T x) {
    x = geometry_detail::clamp(x, T{}, T(1));
    const auto n = (detail::magnitude(x.raw_value()) * 256) / T::scale;
    return static_cast<std::uint8_t>((std::min)(n, UINT64_C(255)));
}
template <class T> std::uint8_t fixed_to_byte(T x) {
    x = geometry_detail::clamp(x, T(-1), T(1));
    const auto n = fxp::trunc(x * 127).template to_integer<std::int64_t>();
    return static_cast<std::uint8_t>(n);
}
template <class T = real> T byte_to_fixed(std::uint8_t n) {
    return T(n >= 128 ? std::int32_t(n) - 256 : std::int32_t(n)) / 127;
}
template <class T> std::uint32_t vec3_to_dword(const vec3<T> &v) {
    return pack_dword(0, fixed_to_byte(v.z), fixed_to_byte(v.y), fixed_to_byte(v.x));
}
template <class T = real> vec3<T> dword_to_vec3(std::uint32_t n) {
    return {byte_to_fixed<T>(unpack_byte0(n)), byte_to_fixed<T>(unpack_byte1(n)),
            byte_to_fixed<T>(unpack_byte2(n))};
}
template <class T> std::uint64_t abs_bits(T x) {
    return detail::magnitude(x.raw_value());
}
template <class T> std::uint64_t sign_bit(T x) {
    return x < 0 ? UINT64_C(0x8000000000000000) : 0;
}
template <class T> std::uint32_t get_binary32_bits(T x) {
    return to_ieee754_binary32_bits(x);
}
template <class T> T exp_negative(T x) {
    return fxp::exp(-x);
}
template <class T> T reciprocal(T x) {
    return T(1) / x;
}

template <class Function>
void bresenham_circle(std::int32_t cx, std::int32_t cy, std::int32_t radius, Function &&function) {
    geometry_detail::raster_bounds(cx, cy, radius);
    if (!radius) {
        function(cx, cy);
        return;
    }
    auto emit = [&](std::int64_t x, std::int64_t y) {
        function(static_cast<std::int32_t>(x), static_cast<std::int32_t>(y));
    };
    std::int64_t x = 0, y = radius, d = 3 - 2 * y;
    do {
        if (d < 0)
            d += 4 * x + 6;
        else {
            d += 4 * (x - y) + 10;
            --y;
        }
        ++x;
        emit(cx - x, cy + y);
        emit(cx + x, cy + y);
        emit(cx - x, cy - y);
        emit(cx + x, cy - y);
        emit(cx - y, cy + x);
        emit(cx + y, cy + x);
        emit(cx - y, cy - x);
        emit(cx + y, cy - x);
    } while (x <= y);
    emit(std::int64_t(cx) - radius, cy);
    emit(std::int64_t(cx) + radius, cy);
    emit(cx, std::int64_t(cy) - radius);
    emit(cx, std::int64_t(cy) + radius);
}
template <class Function>
void bresenham_filled_circle(std::int32_t cx, std::int32_t cy, std::int32_t radius, Function &&function) {
    geometry_detail::raster_bounds(cx, cy, radius);
    if (!radius) {
        function(cx, cy);
        return;
    }
    auto emit = [&](std::int64_t x, std::int64_t y) {
        function(static_cast<std::int32_t>(x), static_cast<std::int32_t>(y));
    };
    std::int64_t x = 0, y = radius, d = 3 - 2 * y;
    do {
        if (d < 0)
            d += 4 * x + 6;
        else {
            d += 4 * (x - y) + 10;
            --y;
        }
        ++x;
        for (auto i = cx - x; i <= cx + x; ++i)
            emit(i, cy + y);
        for (auto i = cx - x; i <= cx + x; ++i)
            emit(i, cy - y);
        for (auto i = cx - y; i <= cx + y; ++i)
            emit(i, cy + x);
        for (auto i = cx - y; i <= cx + y; ++i)
            emit(i, cy - x);
    } while (x <= y);
    for (auto i = std::int64_t(cx) - radius; i <= std::int64_t(cx) + radius; ++i)
        emit(i, cy);
    emit(cx, std::int64_t(cy) - radius);
    emit(cx, std::int64_t(cy) + radius);
}
template <class T, class Function>
void bresenham_ellipse(std::int32_t cx, std::int32_t cy, std::int32_t radius, T ratio, Function &&function) {
    if (ratio < 0)
        throw std::domain_error("negative ellipse ratio");
    bresenham_circle(cx, 0, radius,
                     [&](std::int32_t x, std::int32_t y) { function(x, T(cy) + T(y) * ratio); });
}
template <class T>
inline void create_view_matrix(mat4<T> *pRes, const vec3<T> &vX, const vec3<T> &vY, const vec3<T> &vZ,
                               const vec3<T> &vO) {
    pRes->at(0, 0) = vX.x;
    pRes->at(0, 1) = vX.y;
    pRes->at(0, 2) = vX.z;
    pRes->at(0, 3) = -(vX * vO);
    pRes->at(1, 0) = vY.x;
    pRes->at(1, 1) = vY.y;
    pRes->at(1, 2) = vY.z;
    pRes->at(1, 3) = -(vY * vO);
    pRes->at(2, 0) = vZ.x;
    pRes->at(2, 1) = vZ.y;
    pRes->at(2, 2) = vZ.z;
    pRes->at(2, 3) = -(vZ * vO);
    pRes->at(3, 0) = pRes->at(3, 1) = pRes->at(3, 2) = T::from_string("0.0");
    pRes->at(3, 3) = T::from_string("1.0");
}

template <class T> inline void create_view_matrix_rh(mat4<T> *pRes, const vec3<T> &pos, const quat<T> &rot) {

    vec3<T> vX, vY, vZ;
    rot.get_x_axis(&vX);
    rot.get_y_axis(&vY);
    rot.get_z_axis(&vZ);

    create_view_matrix(pRes, vX, -vY, -vZ, pos);
}
template <class T> inline void create_view_matrix_lh(mat4<T> *pRes, const vec3<T> &pos, const quat<T> &rot) {

    vec3<T> vX, vY, vZ;
    rot.get_x_axis(&vX);
    rot.get_y_axis(&vY);
    rot.get_z_axis(&vZ);

    create_view_matrix(pRes, vX, vY, vZ, pos);
}

template <class T> inline void create_view_matrix_rh(mat4<T> *pRes, const vec3<T> &pos, const mat4<T> &rot) {

    vec3<T> vX, vY, vZ;
    rot.rotate_vector(&vX, vec3<T>(1, 0, 0));
    rot.rotate_vector(&vY, vec3<T>(0, 1, 0));
    rot.rotate_vector(&vZ, vec3<T>(0, 0, 1));

    create_view_matrix(pRes, vX, -vY, -vZ, pos);
}
template <class T> inline void create_view_matrix_lh(mat4<T> *pRes, const vec3<T> &pos, const mat4<T> &rot) {

    vec3<T> vX, vY, vZ;
    rot.rotate_vector(&vX, vec3<T>(1, 0, 0));
    rot.rotate_vector(&vY, vec3<T>(0, 1, 0));
    rot.rotate_vector(&vZ, vec3<T>(0, 0, 1));

    create_view_matrix(pRes, vX, vY, vZ, pos);
}

template <class T>
inline void create_view_matrix_rh(mat4<T> *pRes, const vec3<T> &vFrom, const vec3<T> &vTo,
                                  const vec3<T> &vUp) {
    vec3<T> vX, vY = vUp, vZ = vTo - vFrom;

    if (!normalize(&vZ))
        throw std::domain_error("look-at position equals target");

    vX = vY ^ vZ;
    if (!normalize(&vX))
        throw std::domain_error("look-at up is parallel to direction");

    vY = vZ ^ vX;
    normalize(&vY);

    create_view_matrix(pRes, vX, -vY, -vZ, vFrom);
}
template <class T>
inline void create_view_matrix_lh(mat4<T> *pRes, const vec3<T> &vFrom, const vec3<T> &vTo,
                                  const vec3<T> &vUp) {
    vec3<T> vX, vY = vUp, vZ = vTo - vFrom;

    if (!normalize(&vZ))
        throw std::domain_error("look-at position equals target");

    vX = vY ^ vZ;
    if (!normalize(&vX))
        throw std::domain_error("look-at up is parallel to direction");

    vY = vZ ^ vX;
    normalize(&vY);

    create_view_matrix(pRes, vX, vY, vZ, vFrom);
}

template <class T>
inline void create_direct_transform_matrix(mat4<T> *pRes, const T fWidth, const T fHeight) {
    *pRes = {};
    pRes->at(0, 0) = T::from_string("2.0") / fWidth;
    pRes->at(0, 3) = -(T::from_string("1.0") + T::from_string("1.0") / fWidth);
    pRes->at(1, 1) = -T::from_string("2.0") / fHeight;
    pRes->at(1, 3) = T::from_string("1.0") + T::from_string("1.0") / fHeight;
    pRes->at(2, 2) = pRes->at(3, 3) = T::from_string("1.0");
}

template <class T>
inline void create_perspective_projection_matrix_lh(mat4<T> *pRes, T fov, T fAspect, T fNear, T fFar) {
    const T c = static_cast<T>(cos(fov * T::from_string("0.5")));
    const T s = static_cast<T>(sin(fov * T::from_string("0.5")));
    const T Q = s * fFar / (fFar - fNear);

    *pRes = {};
    pRes->at(0, 0) = c / s / fAspect;
    pRes->at(1, 1) = c / s;
    pRes->at(2, 2) = Q / s;
    pRes->at(2, 3) = -Q * fNear / s;
    pRes->at(3, 2) = s / s;
}
template <class T>
inline void create_perspective_projection_matrix_rh(mat4<T> *pRes, T fov, T fAspect, T fNear, T fFar) {
    const T c = static_cast<T>(cos(fov * T::from_string("0.5")));
    const T s = static_cast<T>(sin(fov * T::from_string("0.5")));
    const T Q = s * fFar / (fFar - fNear);

    *pRes = {};
    pRes->at(0, 0) = c / s / fAspect;
    pRes->at(1, 1) = c / s;
    pRes->at(2, 2) = -Q / s;
    pRes->at(2, 3) = -Q * fNear / s;
    pRes->at(3, 2) = -s / s;
}

template <class T>
inline void create_orthographic_projection_matrix_lh(mat4<T> *pRes, T fWidth, T fHeight, T fNear, T fFar) {
    *pRes = {};
    pRes->at(0, 0) = T::from_string("2.0") / fWidth;
    pRes->at(0, 3) = 0;
    pRes->at(1, 1) = T::from_string("2.0") / fHeight;
    pRes->at(1, 3) = 0;
    pRes->at(2, 2) = T::from_string("1.0") / (fFar - fNear);
    pRes->at(2, 3) = -fNear / (fFar - fNear);
    pRes->at(3, 3) = T::from_string("1.0");
}
template <class T>
inline void create_orthographic_projection_matrix_rh(mat4<T> *pRes, T fWidth, T fHeight, T fNear, T fFar) {
    *pRes = {};
    pRes->at(0, 0) = T::from_string("2.0") / fWidth;
    pRes->at(0, 3) = 0;
    pRes->at(1, 1) = T::from_string("2.0") / fHeight;
    pRes->at(1, 3) = 0;
    pRes->at(2, 2) = -T::from_string("1.0") / (fFar - fNear);
    pRes->at(2, 3) = -fNear / (fFar - fNear);
    pRes->at(3, 3) = T::from_string("1.0");
}

} // namespace fxp
