#pragma once
#include <fxp/geometry_misc.hpp>
#include <string>
#include <vector>

namespace geometry_reference {
inline std::uint64_t next(std::uint64_t &state) {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return state;
}
template <class T> std::string fields(T value) {
    if constexpr (fxp::detail::is_fixed_v<T>)
        return std::to_string(value.raw_value());
    else
        return std::to_string(value);
}
template <class T, std::size_t N> std::string fields(const fxp::vector<T, N> &v) {
    std::string s;
    for (std::size_t i = 0; i < N; ++i) {
        if (i)
            s += ',';
        s += fields(v[i]);
    }
    return s;
}
template <class T> std::string fields(const fxp::mat4<T> &m) {
    std::string s;
    for (std::size_t r = 0; r < 4; ++r)
        for (std::size_t c = 0; c < 4; ++c) {
            if (!s.empty())
                s += ',';
            s += fields(m(r, c));
        }
    return s;
}
template <class T> std::string fields(const fxp::quat<T> &q) {
    return fields(q.get_internal_vector());
}
template <class T> void cases(std::vector<std::string> &rows) {
    using namespace fxp;
    std::uint64_t state = UINT64_C(0x67656f6d65747279);
    for (std::uint32_t i = 0; i < 96; ++i) {
        auto scalar = [&] {
            return T::from_raw((static_cast<std::int64_t>(next(state) % 8193) - 4096) *
                               static_cast<std::int64_t>(T::scale / 1024));
        };
        // Sequence draws explicitly: function-argument evaluation order differs
        // between C++17 compilers, even though each draw is deterministic.
        const T ax = scalar(), ay = scalar(), az = scalar(), bx = scalar(), by = scalar(), bz = scalar();
        const vec3<T> a(ax, ay, az), b(bx, by, bz);
        auto emit = [&](const char *operation, const auto &result) {
            rows.push_back(std::to_string(T::fractional_bits) + ',' + std::to_string(i) + ',' + operation +
                           ',' + fields(result));
        };
        emit("add", a + b);
        emit("sub", a - b);
        emit("dot", a * b);
        emit("cross", a ^ b);
        emit("length", length(a));
        emit("normalize", normalized(a));
        emit("scale", a * T::from_string("1.25"));
        emit("divide", a / T::from_string("1.75"));
        quat<T> q, r;
        q.from_euler_angles(ax / 4, ay / 4, az / 4);
        r.from_euler_angles(bx / 4, by / 4, bz / 4);
        emit("quaternion_product", q * r);
        emit("quaternion_rotate", q.rotate(b));
        emit("quaternion_log_exp", q.log().exp());
        quat<T> interpolated;
        interpolated.slerp(T::from_string("0.375"), q, r);
        emit("slerp", interpolated);
        mat4<T> m(a, q), n(b, r);
        emit("matrix_product", m * n);
        mat4<T> inverse;
        emit("inverse_success", invert(&inverse, m));
        emit("matrix_inverse", inverse);
        vec3<T> p;
        m.rotate_h_vector(&p, b);
        emit("point_transform", p);
        mat4<T> view;
        create_view_matrix_rh(&view, a, q);
        emit("view_rh", view);
        create_view_matrix_lh(&view, a, r);
        emit("view_lh", view);
        emit("direction", direction_by_vector(vec2<T>(ax, ay)));
        emit("direction_vector", vector_by_direction<T>(static_cast<std::uint16_t>(i * 683)));
        emit("packed_vector", vec3_to_dword(normalized(a)));
        oriented_rect<T> rect;
        rect.init_rect(vec2<T>{}, vec2<T>(1, 1), T(2), T(1));
        const vec2<T> point(ax, ay);
        emit("rectangle_contains", rect.is_point_inside(point));
        emit("rectangle_circle", rect.is_intersect_circle(point, T::from_string("0.5")));
        vec2<T> hit;
        emit("ray_hit", intersect_ray_rect(&hit, point, vec2<T>(bx, by), rect));
        emit("ray_point", hit);
        circle<T> circle(vec2<T>{}, T(1));
        vec2<T> t1, t2;
        emit("tangent_success", find_tangent_points(point, circle, &t1, &t2));
        emit("tangent1", t1);
        emit("tangent2", t2);
    }
    for (std::size_t i = 0; i < 5; ++i) {
        const T values[] = {T{}, T::epsilon(), T::from_raw(-1), T::min(), T::max()};
        const vec4<T> v(values[i], values[i], values[i], values[i]);
        rows.push_back(std::to_string(T::fractional_bits) + ",edge" + std::to_string(i) + ",normalize4," +
                       fields(normalized(v)));
    }
}
inline std::vector<std::string> generate() {
    std::vector<std::string> rows;
    cases<fxp::real>(rows);
    cases<fxp::fine>(rows);
    return rows;
}
} // namespace geometry_reference
